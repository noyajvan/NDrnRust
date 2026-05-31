//! ESP32-S3 DropCtrlV3 — no_std MAVLink Bridge
#![no_std]
#![no_main]

use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use esp_hal::uart::Uart;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
esp_bootloader_esp_idf::esp_app_desc!();

const SYS_ID: u8 = 1;
const COMP_ID: u8 = 0;

// ─── Макрос для UART write ────────────────────────────────────────────────
macro_rules! uart_write {
    ($uart:expr, $data:expr) => { let _ = ($uart).write($data); };
}

// ─── MAVLink V2 parser ────────────────────────────────────────────────────
struct MavParser { buf: [u8; 280], pos: usize, framing: bool }
struct MavPacket { msgid: u32, payload: [u8; 255], len: usize }

impl MavParser {
    fn new() -> Self { Self { buf: [0; 280], pos: 0, framing: false } }
    fn parse(&mut self, byte: u8) -> Option<MavPacket> {
        if !self.framing {
            if byte == 0xFD { self.framing = true; self.pos = 0; }
            if self.pos < 280 { self.buf[self.pos] = byte; }
            self.pos += 1; return None;
        }
        if self.pos < 280 { self.buf[self.pos] = byte; }
        self.pos += 1;
        if self.pos >= 12 && self.buf[0] == 0xFD {
            let len = self.buf[1] as usize;
            if self.pos >= len + 12 {
                self.framing = false;
                let msgid = self.buf[5] as u32 | (self.buf[6] as u32) << 8 | (self.buf[7] as u32) << 16;
                let mut payload = [0u8; 255];
                for i in 0..len.min(255) { payload[i] = self.buf[10 + i]; }
                return Some(MavPacket { msgid, payload, len });
            }
        }
        None
    }
}

// ─── MAVLink builders ─────────────────────────────────────────────────────
fn make_heartbeat(seq: u8) -> [u8; 21] {
    let mut buf = [0u8; 21];
    buf[0] = 0xFD; buf[1] = 9; buf[4] = seq; buf[5] = 0;
    buf[8] = SYS_ID; buf[9] = COMP_ID; buf[10] = 9; buf[18] = 3;
    buf
}

fn make_cmd(command: u16, p1: f32, p2: f32) -> [u8; 37] {
    let mut buf = [0u8; 37];
    buf[0] = 0xFD; buf[1] = 20; buf[5] = 76;
    buf[8] = SYS_ID; buf[9] = COMP_ID;
    buf[10] = 1; buf[11] = 1;
    buf[12] = command as u8; buf[13] = (command >> 8) as u8;
    buf[15..19].copy_from_slice(&p1.to_le_bytes());
    buf[19..23].copy_from_slice(&p2.to_le_bytes());
    buf
}

fn make_relay() -> [u8; 37] { make_cmd(189, 0.0, 1.0) }
fn make_disarm() -> [u8; 37] { make_cmd(400, 0.0, 21196.0) }

// ─── Crash detection ──────────────────────────────────────────────────────
struct Crash {
    flying: bool, emergency: bool, stuck_timer: Instant, gyro_timer: Instant,
    throttle: u16, groundspeed: f32,
    failsafe: bool, fs_step: u8, fs_time: Instant, relay_after_land: bool,
}

impl Crash {
    fn new() -> Self {
        Self {
            flying: false, emergency: false,
            stuck_timer: Instant::now(), gyro_timer: Instant::now(),
            throttle: 0, groundspeed: 0.0,
            failsafe: false, fs_step: 0, fs_time: Instant::now(), relay_after_land: false,
        }
    }
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut led = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    let boot = Input::new(peripherals.GPIO0, InputConfig::default().with_pull(Pull::Up));

    let uart_config = esp_hal::uart::Config::default().with_baudrate(115200);
    let mut fc = Uart::new(peripherals.UART0, uart_config).unwrap()
        .with_tx(peripherals.GPIO43)
        .with_rx(peripherals.GPIO44);

    let mut parser = MavParser::new();
    let mut crash = Crash::new();
    let mut armed = false;
    let mut last_armed = false;
    let mut hb_seq: u8 = 0;
    let mut last_hb = Instant::now();
    let mut rx = [0u8; 64];
    let mut n: u32 = 0;

    loop {
        n = n.wrapping_add(1);
        let now = Instant::now();

        if let Ok(len) = fc.read(&mut rx) {
            for &b in &rx[..len] {
                if let Some(pkt) = parser.parse(b) {
                    match pkt.msgid {
                        0 => {
                            armed = (pkt.payload[4] & 0x80) != 0;
                            if last_armed && !armed && !crash.emergency && crash.flying {
                                crash.emergency = true;
                                uart_write!(fc, &make_disarm());
                                if crash.relay_after_land { uart_write!(fc, &make_relay()); }
                            }
                            last_armed = armed;
                            if armed { crash.flying = true; }
                        }
                        74 => {
                            crash.throttle = (pkt.payload[8] as u16) | (pkt.payload[9] as u16) << 8;
                            crash.groundspeed = f32::from_le_bytes(
                                [pkt.payload[16], pkt.payload[17], pkt.payload[18], pkt.payload[19]]);
                            if !crash.emergency && crash.flying && armed {
                                if crash.groundspeed < 0.15 && crash.throttle > 45 {
                                    if (now - crash.stuck_timer) > Duration::from_millis(3000) {
                                        crash.emergency = true;
                                        crash.failsafe = true;
                                        crash.fs_step = 0;
                                        crash.fs_time = now;
                                        uart_write!(fc, &make_disarm());
                                    }
                                } else { crash.stuck_timer = now; }
                            }
                        }
                        27 => {
                            let xg = i16::from_le_bytes([pkt.payload[12], pkt.payload[13]]);
                            let yg = i16::from_le_bytes([pkt.payload[14], pkt.payload[15]]);
                            if !crash.emergency && crash.flying && armed {
                                if xg.abs() > 4500 || yg.abs() > 4500 {
                                    if (now - crash.gyro_timer) > Duration::from_millis(150) {
                                        crash.emergency = true;
                                    }
                                } else { crash.gyro_timer = now; }
                            }
                        }
                        _ => {}
                    }
                }
            }
        }

        if now - last_hb > Duration::from_millis(1000) {
            uart_write!(fc, &make_heartbeat(hb_seq));
            hb_seq = hb_seq.wrapping_add(1);
            last_hb = now;
        }

        if crash.failsafe {
            let t = now - crash.fs_time;
            if crash.fs_step == 0 {
                uart_write!(fc, &make_disarm());
                crash.fs_step = 1; crash.fs_time = now;
            } else if crash.fs_step == 1 && t > Duration::from_millis(25) {
                if crash.relay_after_land { uart_write!(fc, &make_relay()); }
                crash.fs_step = 2; crash.fs_time = now;
            } else if crash.fs_step == 2 && t > Duration::from_millis(50) {
                crash.failsafe = false;
            }
        }

        if boot.is_low() {
            led.set_high();
            let p = Instant::now();
            while p.elapsed() < Duration::from_millis(200) {}
            led.set_low();
        }

        if n % 500 == 0 { led.toggle(); }
        while now.elapsed() < Duration::from_millis(10) {}
    }
}
