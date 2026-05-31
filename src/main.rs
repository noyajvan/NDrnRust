//! ESP32-S3 DropCtrlV3 — no_std MAVLink Bridge
//!
//! Піни:
//!   - FC UART TX: GPIO 43
//!   - FC UART RX: GPIO 44
//!   - BOOT button: GPIO 0
//!   - Signal LED:  GPIO 15

#![no_std]
#![no_main]

use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use esp_hal::uart::Uart;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

const SYS_ID: u8 = 1;
const COMP_ID: u8 = 0;

// ─── MAVLink V2 парсер ─────────────────────────────────────────────────────
struct MavParser {
    buf: [u8; 280],
    pos: usize,
    framing: bool,
}

struct MavPacket {
    msgid: u32,
    payload: [u8; 255],
    len: usize,
}

impl MavParser {
    fn new() -> Self {
        Self { buf: [0; 280], pos: 0, framing: false }
    }

    fn parse(&mut self, byte: u8) -> Option<MavPacket> {
        if !self.framing {
            if byte == 0xFD { self.framing = true; self.pos = 0; }
            if self.pos < self.buf.len() { self.buf[self.pos] = byte; }
            self.pos += 1;
            return None;
        }
        if self.pos < self.buf.len() { self.buf[self.pos] = byte; }
        self.pos += 1;
        if self.pos >= 12 && self.buf[0] == 0xFD {
            let len = self.buf[1] as usize;
            if self.pos >= len + 12 {
                self.framing = false;
                let msgid = self.buf[5] as u32
                    | (self.buf[6] as u32) << 8
                    | (self.buf[7] as u32) << 16;
                let mut payload = [0u8; 255];
                for i in 0..len.min(255) { payload[i] = self.buf[10 + i]; }
                return Some(MavPacket { msgid, payload, len });
            }
        }
        None
    }
}

// ─── MAVLink builders ──────────────────────────────────────────────────────
fn make_heartbeat(seq: u8) -> [u8; 21] {
    let mut buf = [0u8; 21];
    buf[0] = 0xFD; buf[1] = 9; buf[4] = seq; buf[5] = 0;
    buf[8] = SYS_ID; buf[9] = COMP_ID;
    buf[10] = 9;  // MAV_TYPE_ONBOARD_CONTROLLER
    buf[18] = 3;  // mavlink_version
    buf
}

fn make_command_long(command: u16, param1: f32, param2: f32) -> [u8; 37] {
    let mut buf = [0u8; 37];
    buf[0] = 0xFD; buf[1] = 20; buf[5] = 76; buf[6] = 0; buf[7] = 0;
    buf[8] = SYS_ID; buf[9] = COMP_ID;
    buf[10] = 1; buf[11] = 1;
    buf[12] = command as u8; buf[13] = (command >> 8) as u8;
    let p1 = param1.to_le_bytes(); buf[15..19].copy_from_slice(&p1);
    let p2 = param2.to_le_bytes(); buf[19..23].copy_from_slice(&p2);
    buf
}

fn make_set_relay() -> [u8; 37] { make_command_long(189, 0.0, 1.0) }
fn make_disarm() -> [u8; 37] { make_command_long(400, 0.0, 21196.0) }

// ─── Відправка по UART ────────────────────────────────────────────────────
fn uart_write(uart: &mut impl esp_hal::uart::Write, data: &[u8]) {
    let _ = uart.write_bytes(data);
}

// ─── Crash detection ──────────────────────────────────────────────────────
struct CrashDetect {
    was_flying: bool,
    emergency: bool,
    stuck_timer: Instant,
    gyro_timer: Instant,
    throttle: u16,
    ground_speed: f32,
    failsafe_pending: bool,
    failsafe_step: u8,
    failsafe_time: Instant,
    relay_after_land: bool,
}

impl CrashDetect {
    fn new() -> Self {
        Self {
            was_flying: false, emergency: false,
            stuck_timer: Instant::now(), gyro_timer: Instant::now(),
            throttle: 0, ground_speed: 0.0,
            failsafe_pending: false, failsafe_step: 0,
            failsafe_time: Instant::now(), relay_after_land: false,
        }
    }
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut led = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    let boot_btn = Input::new(peripherals.GPIO0, InputConfig::default().with_pull(Pull::Up));

    let uart_config = esp_hal::uart::Config::default().with_baudrate(115200);
    let mut fc_uart = Uart::new(peripherals.UART0, uart_config)
        .unwrap()
        .with_tx(peripherals.GPIO43)
        .with_rx(peripherals.GPIO44);

    let mut parser = MavParser::new();
    let mut crash = CrashDetect::new();

    let mut hb_received = false;
    let mut is_armed = false;
    let mut last_armed = false;
    let mut hb_seq: u8 = 0;
    let mut last_hb_time = Instant::now();

    let mut rx_buf = [0u8; 64];

    let mut counter: u32 = 0;

    loop {
        counter = counter.wrapping_add(1);
        let now = Instant::now();

        // ─── Прийом UART ─────
        if let Ok(n) = fc_uart.read(&mut rx_buf) {
            for &byte in &rx_buf[..n] {
                if let Some(pkt) = parser.parse(byte) {
                    match pkt.msgid {
                        0 => { // HEARTBEAT
                            hb_received = true;
                            let base_mode = pkt.payload[4];
                            is_armed = (base_mode & 0x80) != 0;
                            if last_armed && !is_armed && !crash.emergency && crash.was_flying {
                                crash.emergency = true;
                                uart_write(&mut fc_uart, &make_disarm());
                                if crash.relay_after_land {
                                    uart_write(&mut fc_uart, &make_set_relay());
                                }
                            }
                            last_armed = is_armed;
                            if is_armed { crash.was_flying = true; }
                        }
                        74 => { // VFR_HUD
                            crash.throttle = (pkt.payload[8] as u16) | (pkt.payload[9] as u16) << 8;
                            crash.ground_speed = f32::from_le_bytes([
                                pkt.payload[16], pkt.payload[17], pkt.payload[18], pkt.payload[19]
                            ]);
                            if !crash.emergency && crash.was_flying && is_armed {
                                if crash.ground_speed < 0.15 && crash.throttle > 45 {
                                    if (now - crash.stuck_timer) > Duration::from_millis(3000) {
                                        crash.emergency = true;
                                        crash.failsafe_pending = true;
                                        crash.failsafe_step = 0;
                                        crash.failsafe_time = now;
                                        uart_write(&mut fc_uart, &make_disarm());
                                    }
                                } else {
                                    crash.stuck_timer = now;
                                }
                            }
                        }
                        27 => { // RAW_IMU
                            let xgyro = i16::from_le_bytes([pkt.payload[12], pkt.payload[13]]);
                            let ygyro = i16::from_le_bytes([pkt.payload[14], pkt.payload[15]]);
                            if !crash.emergency && crash.was_flying && is_armed {
                                if xgyro.abs() > 4500 || ygyro.abs() > 4500 {
                                    if (now - crash.gyro_timer) > Duration::from_millis(150) {
                                        crash.emergency = true;
                                    }
                                } else {
                                    crash.gyro_timer = now;
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
        }

        // ─── Heartbeat ─────
        if now - last_hb_time > Duration::from_millis(1000) {
            let hb = make_heartbeat(hb_seq);
            hb_seq = hb_seq.wrapping_add(1);
            uart_write(&mut fc_uart, &hb);
            last_hb_time = now;
        }

        // ─── Failsafe ─────
        if crash.failsafe_pending {
            let ft = now - crash.failsafe_time;
            if crash.failsafe_step == 0 {
                uart_write(&mut fc_uart, &make_disarm());
                crash.failsafe_step = 1;
                crash.failsafe_time = now;
            } else if crash.failsafe_step == 1 && ft > Duration::from_millis(25) {
                if crash.relay_after_land {
                    uart_write(&mut fc_uart, &make_set_relay());
                }
                crash.failsafe_step = 2;
                crash.failsafe_time = now;
            } else if crash.failsafe_step == 2 && ft > Duration::from_millis(50) {
                crash.failsafe_pending = false;
            }
        }

        // ─── BOOT button ─────
        if boot_btn.is_low() {
            led.set_high();
            let p = Instant::now();
            while p.elapsed() < Duration::from_millis(200) {}
            led.set_low();
        }

        // ─── LED blink ─────
        if counter % 500 == 0 {
            led.toggle();
        }

        // ─── Часовий крок ─────
        while now.elapsed() < Duration::from_millis(10) {}
    }
}
