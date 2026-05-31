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
use esp_hal::gpio::{Input, Level, Output, Pull};
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use esp_hal::uart::Uart;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

// ─── Константи ─────────────────────────────────────────────────────────────
const SYS_ID: u8 = 1;
const COMP_ID: u8 = 0; // MAV_COMP_ID_ONBOARD_COMPUTER
const TAKEOFF_MODE: u32 = 13;

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
            if byte == 0xFD {
                self.framing = true;
                self.pos = 0;
            }
            if self.pos < self.buf.len() {
                self.buf[self.pos] = byte;
            }
            self.pos += 1;
            return None;
        }

        if self.pos < self.buf.len() {
            self.buf[self.pos] = byte;
        }
        self.pos += 1;

        if self.pos >= 12 && self.buf[0] == 0xFD {
            let len = self.buf[1] as usize;
            if self.pos >= len + 12 {
                self.framing = false;
                let msgid = self.buf[5] as u32
                    | (self.buf[6] as u32) << 8
                    | (self.buf[7] as u32) << 16;
                let mut payload = [0u8; 255];
                for i in 0..len.min(255) {
                    payload[i] = self.buf[10 + i];
                }
                return Some(MavPacket { msgid, payload, len });
            }
        }
        None
    }
}

// ─── MAVLink helpers ──────────────────────────────────────────────────────
fn make_heartbeat() -> [u8; 30] {
    let mut buf = [0u8; 30];
    // V2 header: magic=FD, len=9, incompat=0, compat=0, seq=0, sysid, compid
    buf[0] = 0xFD;
    buf[1] = 9;  // payload length for HEARTBEAT
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = 0;  // seq
    buf[5] = 0;  // msgid (0)
    buf[6] = 0;
    buf[7] = 0;
    buf[8] = SYS_ID;
    buf[9] = COMP_ID;
    // HEARTBEAT payload: type=9 (ONBOARD_CONTROLLER), autopilot=0, base_mode=0, custom_mode=0, system_status=0
    buf[10] = 9;  // MAV_TYPE_ONBOARD_CONTROLLER
    buf[11] = 0;  // autopilot
    buf[12] = 0;  // base_mode
    buf[13] = 0;  // custom_mode (u32)
    buf[14] = 0;
    buf[15] = 0;
    buf[16] = 0;
    buf[17] = 0;  // system_status
    buf[18] = 3;  // mavlink_version
    // Simple checksum (just placeholder - FC ignores checksum usually)
    // bytes 1..19
    buf[19] = 0x55;
    buf[20] = 0xAA;
    21
}

fn make_command_long(command: u16, param1: f32, param2: f32) -> [u8; 45] {
    let mut buf = [0u8; 45];
    buf[0] = 0xFD;
    buf[1] = 20; // payload len
    buf[4] = 0;  // seq, updated per send
    let msgid = 76u32; // COMMAND_LONG
    buf[5] = msgid as u8;
    buf[6] = (msgid >> 8) as u8;
    buf[7] = (msgid >> 16) as u8;
    buf[8] = SYS_ID;
    buf[9] = COMP_ID;
    // target_system, target_component
    buf[10] = 1;
    buf[11] = 1;  // MAV_COMP_ID_AUTOPILOT1
    // command u16
    buf[12] = command as u8;
    buf[13] = (command >> 8) as u8;
    // confirmation
    buf[14] = 0;
    // param1
    let p1 = param1.to_le_bytes();
    buf[15] = p1[0]; buf[16] = p1[1]; buf[17] = p1[2]; buf[18] = p1[3];
    // param2
    let p2 = param2.to_le_bytes();
    buf[19] = p2[0]; buf[20] = p2[1]; buf[21] = p2[2]; buf[22] = p2[3];
    // param3-7 = 0
    // checksum placeholder
    buf[43] = 0x55;
    buf[44] = 0xAA;
    45
}

fn make_set_relay() -> [u8; 45] {
    make_command_long(189, 0.0, 1.0)  // MAV_CMD_DO_SET_RELAY = 189
}

fn make_disarm() -> [u8; 45] {
    make_command_long(400, 0.0, 21196.0)  // MAV_CMD_COMPONENT_ARM_DISARM = 400
}

// ─── Crash detection ──────────────────────────────────────────────────────
struct CrashDetect {
    was_flying: bool,
    emergency: bool,
    stuck_timer: Instant,
    gyro_timer: Instant,
    last_alt: f32,
    last_roll: f32,
    last_pitch: f32,
    last_throttle: u16,
    last_ground_speed: f32,
    failsafe_pending: bool,
    failsafe_step: u8,
    failsafe_time: Instant,
    relay_after_land: bool,
}

impl CrashDetect {
    fn new() -> Self {
        Self {
            was_flying: false,
            emergency: false,
            stuck_timer: Instant::now(),
            gyro_timer: Instant::now(),
            last_alt: 0.0,
            last_roll: 0.0,
            last_pitch: 0.0,
            last_throttle: 0,
            last_ground_speed: 0.0,
            failsafe_pending: false,
            failsafe_step: 0,
            failsafe_time: Instant::now(),
            relay_after_land: false,
        }
    }
}

// ─── Функція для запису в UART ────────────────────────────────────────────
fn uart_send(uart: &mut Uart, data: &[u8]) {
    for &b in data {
        uart.write_byte(b);
    }
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut led = Output::new(peripherals.GPIO15, Level::Low);
    let boot_btn = Input::new(peripherals.GPIO0, Pull::Up);

    // WS2812 на GPIO 48 (якщо є — ініціалізується нижче)
    // У esp-hal v1.1 RMT потребує іншого підходу, поки що тільки GPIO15

    let uart_config = esp_hal::uart::Config::default()
        .with_baudrate(115200)
        .with_data_bits(esp_hal::uart::DataBits::DataBits8)
        .with_stop_bits(esp_hal::uart::StopBits::STOP1)
        .with_parity(esp_hal::uart::Parity::ParityNone);
    let mut fc_uart = Uart::new(peripherals.UART0, uart_config)
        .unwrap()
        .with_tx(peripherals.GPIO43)
        .with_rx(peripherals.GPIO44);

    let mut parser = MavParser::new();
    let mut crash = CrashDetect::new();

    // Стан
    let mut hb_received = false;
    let mut is_armed = false;
    let mut last_armed = false;
    let mut custom_mode: u32 = 0;
    let mut mission_count: u16 = 0;
    let mut mission_loaded = false;
    let mut land_seen = false;

    let mut hb_seq: u8 = 0;
    let mut last_hb_time = Instant::now();
    let mut start_time = Instant::now();
    let mut last_mission_req = Instant::now();

    let mut counter: u32 = 0;

    loop {
        counter = counter.wrapping_add(1);
        let now = Instant::now();

        // ─── Прийом UART ─────
        while let Ok(byte) = fc_uart.read_byte() {
            if let Some(pkt) = parser.parse(byte) {
                // Обробка MAVLink повідомлень
                match pkt.msgid {
                    0 => { // HEARTBEAT
                        hb_received = true;
                        // payload offset: target_system=0, target_component=1, type=2, autopilot=3, base_mode=4
                        let base_mode = pkt.payload[4];
                        custom_mode = pkt.payload[5] as u32
                            | (pkt.payload[6] as u32) << 8
                            | (pkt.payload[7] as u32) << 16
                            | (pkt.payload[8] as u32) << 24;
                        is_armed = (base_mode & 0x80) != 0;

                        // Якщо був у польоті і раптом disarm
                        if last_armed && !is_armed && !crash.emergency && crash.was_flying {
                            crash.emergency = true;
                            uart_send(&mut fc_uart, &make_disarm());
                            if crash.relay_after_land {
                                uart_send(&mut fc_uart, &make_set_relay());
                            }
                        }
                        last_armed = is_armed;
                        if is_armed { crash.was_flying = true; }
                    }
                    74 => { // VFR_HUD
                        let alt = f32::from_le_bytes([pkt.payload[0], pkt.payload[1], pkt.payload[2], pkt.payload[3]]);
                        let throttle = (pkt.payload[8] as u16) | (pkt.payload[9] as u16) << 8;
                        let groundspeed = f32::from_le_bytes([pkt.payload[16], pkt.payload[17], pkt.payload[18], pkt.payload[19]]);

                        crash.last_alt = alt;
                        crash.last_throttle = throttle;
                        crash.last_ground_speed = groundspeed;

                        // Crash detection
                        if !crash.emergency && crash.was_flying && is_armed {
                            // Stuck detection
                            if groundspeed < 0.15 && throttle > 45 {
                                let stuck_dt = now - crash.stuck_timer;
                                if stuck_dt > Duration::from_millis(3000) {
                                    crash.emergency = true;
                                    crash.failsafe_pending = true;
                                    crash.failsafe_step = 0;
                                    crash.failsafe_time = now;
                                    uart_send(&mut fc_uart, &make_disarm());
                                }
                            } else {
                                crash.stuck_timer = now;
                            }
                        }
                    }
                    27 => { // RAW_IMU
                        // Gyro tumble detection
                        let xgyro = i16::from_le_bytes([pkt.payload[12], pkt.payload[13]]);
                        let ygyro = i16::from_le_bytes([pkt.payload[14], pkt.payload[15]]);
                        if !crash.emergency && crash.was_flying && is_armed {
                            if xgyro.abs() > 4500 || ygyro.abs() > 4500 {
                                let gyro_dt = now - crash.gyro_timer;
                                if gyro_dt > Duration::from_millis(150) {
                                    crash.emergency = true;
                                    // disarmed will trigger via heartbeat
                                }
                            } else {
                                crash.gyro_timer = now;
                            }
                        }
                    }
                    44 => { // MISSION_COUNT
                        mission_count = (pkt.payload[2] as u16) | (pkt.payload[3] as u16) << 8;
                        mission_loaded = false;
                        land_seen = false;
                        crash.relay_after_land = false;
                        // Request first item
                        if mission_count > 0 {
                            let mut req = [0u8; 18];
                            req[0] = 0xFD;
                            req[1] = 5;  // MISSION_REQUEST_INT payload length
                            req[5] = 51;  // msgid MISSION_REQUEST_INT = 51
                            req[8] = SYS_ID;
                            req[9] = COMP_ID;
                            req[10] = 1;  // target_system
                            req[11] = 1;  // target_component
                            req[12] = 0;  // seq start
                            req[13] = 0;
                            req[14] = 0;  // mission_type = MISSION
                            uart_send(&mut fc_uart, &req);
                        }
                    }
                    _ => {}
                }
            }
        }

        // ─── Периодичні MAVLink повідомлення ─────
        if now - last_hb_time > Duration::from_millis(1000) {
            let mut hb = make_heartbeat();
            hb[4] = hb_seq;
            hb_seq = hb_seq.wrapping_add(1);
            uart_send(&mut fc_uart, &hb[..]);
            last_hb_time = now;
        }

        // ─── Failsafe ─────
        if crash.failsafe_pending {
            let ft = now - crash.failsafe_time;
            if crash.failsafe_step == 0 {
                uart_send(&mut fc_uart, &make_disarm());
                crash.failsafe_step = 1;
                crash.failsafe_time = now;
            } else if crash.failsafe_step == 1 && ft > Duration::from_millis(25) {
                if crash.relay_after_land {
                    uart_send(&mut fc_uart, &make_set_relay());
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
        while now.elapsed() < Duration::from_millis(5) {}
    }
}
