//! ESP32-S3 DropCtrlV3 — Mavlink Bridge
//!
//! Переписано з Arduino C на Rust (ESP-IDF / std)
//!
//! Піни ESP32-S3 Super Mini:
//!   - FC UART: TX=43, RX=44 (UART0)
//!   - WS2812 LED: GPIO 48
//!   - BOOT btn: GPIO 0 (pull-up, active low)
//!   - Синий LED: GPIO 15

use core::fmt::Write as _;

use embedded_svc::wifi::{
    ClientConfiguration, Configuration, Wifi, WifiWait,
};
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Input, Output, PinDriver, PullUp},
    peripherals::Peripherals,
    prelude::*,
    uart::*,
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    nvs::EspDefaultNvsPartition,
    wifi::{BlockingWifi, EspWifi},
};
use esp_idf_sys as _;

use heapless::String as HString;
use mavlink::common::{self, MavMessage};
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_esp32_rmt_driver::Ws2812Esp32Rmt;

// ─── Константи ─────────────────────────────────────────────────────────────
const UDP_PORT: u16 = 14550;
const MAX_UDP_CLIENTS: usize = 8;
const BRIDGE_BUF_SIZE: usize = 1024;
const MAX_STATIONS: usize = 5;

const FC_RX_BUF: usize = 2048;
const FC_TX_BUF: usize = 256;

const LED_PIN: u32 = 48;
const BOOT_BTN: u32 = 0;
const SYS_ID: u8 = 1;
const COMP_ID: u8 = mavlink::common::MAV_COMP_ID_ONBOARD_COMPUTER;
const TAKEOFF_MODE: u32 = 13;

// тимчасовий буфер для команд терміналу
static mut INPUT_BUF: [u8; 128] = [0u8; 128];

// ─── Структури ─────────────────────────────────────────────────────────────
struct Config {
    sta_ssid: HString<32>,
    sta_pass: HString<64>,
    baud: u32,
    stations: [HString<32>; MAX_STATIONS],
    station_count: usize,
    wifi_boot: bool,
}

impl Default for Config {
    fn default() -> Self {
        let mut ssid = HString::new();
        let _ = ssid.push_str("LEO");
        let mut pass = HString::new();
        let _ = pass.push_str("88888888");
        Self {
            sta_ssid: ssid,
            sta_pass: pass,
            baud: 115200,
            stations: [HString::new(), HString::new(), HString::new(), HString::new(), HString::new()],
            station_count: 0,
            wifi_boot: false,
        }
    }
}

struct Point {
    lat: i32,
    lon: i32,
    relay: bool,
    received: bool,
}

impl Point {
    fn new() -> Self { Self { lat: 0, lon: 0, relay: false, received: false } }
}

struct UdpClient {
    ip: [u8; 4],
    port: u16,
    active: bool,
}

impl UdpClient {
    fn new() -> Self { Self { ip: [0; 4], port: 0, active: false } }
}

struct State {
    config: Config,
    wifi_on: bool,
    wifi_activating: bool,
    wifi_activate_time: u64,
    sta_retry_count: u32,
    sta_was_connected: bool,

    udp_clients: [UdpClient; MAX_UDP_CLIENTS],
    gs_connected: bool,

    mav_status: mavlink::MavlinkV2Parser,
    fc_bytes: u32,
    fc_msgs: u32,

    heartbeat_received: bool,
    is_armed: bool,
    current_custom_mode: u32,
    system_status: u8,
    takeoff_mode_detected: bool,
    takeoff_detected: bool,
    mission_loaded: bool,
    mission_count: u16,
    mission_first_parsed: bool,
    p_tg: Point,
    land_seen: bool,
    relay_after_land: bool,
    last_arm_state: bool,

    roll: f32,
    pitch: f32,
    current_alt: f32,
    last_stable_alt: f32,
    throttle: u16,
    ground_speed: f32,

    was_flying: bool,
    emergency_triggered: bool,
    failsafe_pending: bool,
    failsafe_relay: bool,
    failsafe_step: u8,
    failsafe_time: u64,

    stuck_timer: u64,
    roof_stuck_timer: u64,
    gyro_crash_timer: u64,

    last_wifi_hb: u64,
    last_radio_status: u64,
    last_serial_log: u64,
    last_sta_reconnect: u64,
    start_time: u64,
    last_mission_req: u64,
    last_mission_scan: u64,
}

impl State {
    fn new(config: Config) -> Self {
        Self {
            config,
            wifi_on: false,
            wifi_activating: false,
            wifi_activate_time: 0,
            sta_retry_count: 0,
            sta_was_connected: false,
            udp_clients: [UdpClient::new(); MAX_UDP_CLIENTS],
            gs_connected: false,
            mav_status: mavlink::MavlinkV2Parser::new(),
            fc_bytes: 0,
            fc_msgs: 0,
            heartbeat_received: false,
            is_armed: false,
            current_custom_mode: 0,
            system_status: 0,
            takeoff_mode_detected: false,
            takeoff_detected: false,
            mission_loaded: false,
            mission_count: 0,
            mission_first_parsed: false,
            p_tg: Point::new(),
            land_seen: false,
            relay_after_land: false,
            last_arm_state: false,
            roll: 0.0,
            pitch: 0.0,
            current_alt: 0.0,
            last_stable_alt: 0.0,
            throttle: 0,
            ground_speed: 0.0,
            was_flying: false,
            emergency_triggered: false,
            failsafe_pending: false,
            failsafe_relay: false,
            failsafe_step: 0,
            failsafe_time: 0,
            stuck_timer: 0,
            roof_stuck_timer: 0,
            gyro_crash_timer: 0,
            last_wifi_hb: 0,
            last_radio_status: 0,
            last_serial_log: 0,
            last_sta_reconnect: 0,
            start_time: 0,
            last_mission_req: 0,
            last_mission_scan: 0,
        }
    }
}

// ─── LED (WS2812) ──────────────────────────────────────────────────────────
fn set_led(ws2812: &mut Ws2812Esp32Rmt, color: RGB8) {
    let colors = core::iter::once(color);
    ws2812.write(colors).ok();
}

fn led_color(state: &State, now: u64) -> RGB8 {
    if state.emergency_triggered || state.failsafe_pending {
        return RGB8::new(64, 0, 0); // RED
    }
    if !state.heartbeat_received {
        return if now % 1000 < 200 { RGB8::new(64, 64, 64) } else { RGB8::new(0, 0, 0) }; // WHITE blink
    }
    let c = if state.current_custom_mode == 3 {
        if state.is_armed { RGB8::new(0, 64, 0) } else { RGB8::new(64, 32, 0) } // GREEN / ORANGE
    } else {
        RGB8::new(64, 64, 64) // WHITE
    };
    if !state.wifi_on {
        return c;
    }
    // WiFi blink pattern
    if now % 1000 < 500 {
        c
    } else {
        RGB8::new(0, 0, 0)
    }
}

// ─── NVS Config ────────────────────────────────────────────────────────────
fn load_config(nvs: &EspDefaultNvsPartition) -> Config {
    let mut cfg = Config::default();

    if let Ok(ns) = nvs.namespace("dbridge") {
        let mut buf = [0u8; 64];
        if ns.get_raw("sta_ssid", &mut buf).is_ok() {
            let s = core::str::from_utf8(&buf).unwrap_or("LEO");
            let mut hs = HString::new(); let _ = hs.push_str(s.trim_end_matches('\0'));
            if !hs.is_empty() { cfg.sta_ssid = hs; }
        }
        if ns.get_raw("sta_pass", &mut buf).is_ok() {
            let s = core::str::from_utf8(&buf).unwrap_or("88888888");
            let mut hs = HString::new(); let _ = hs.push_str(s.trim_end_matches('\0'));
            if !hs.is_empty() { cfg.sta_pass = hs; }
        }
        cfg.baud = ns.get_u32("baud").unwrap_or(115200);
        cfg.station_count = ns.get_u32("st_count").unwrap_or(0) as usize;
        cfg.wifi_boot = ns.get_u32("wifi_boot").unwrap_or(0) != 0;

        for i in 0..cfg.station_count.min(MAX_STATIONS) {
            let key = format!("ip_{}", i);
            let mut buf = [0u8; 32];
            if ns.get_raw(&key, &mut buf).is_ok() {
                let s = core::str::from_utf8(&buf).unwrap_or("");
                let mut hs = HString::new(); let _ = hs.push_str(s.trim_end_matches('\0'));
                cfg.stations[i] = hs;
            }
        }
    }

    log::info!(
        "NVS loaded: ssid={}, baud={}, stations={}, wifi_boot={}",
        cfg.sta_ssid, cfg.baud, cfg.station_count, cfg.wifi_boot
    );
    cfg
}

fn save_config(nvs: &EspDefaultNvsPartition, cfg: &Config) {
    if let Ok(mut ns) = nvs.namespace("dbridge") {
        let _ = ns.set_raw("sta_ssid", cfg.sta_ssid.as_bytes());
        let _ = ns.set_raw("sta_pass", cfg.sta_pass.as_bytes());
        let _ = ns.set_u32("baud", cfg.baud);
        let _ = ns.set_u32("st_count", cfg.station_count as u32);
        let _ = ns.set_u32("wifi_boot", if cfg.wifi_boot { 1 } else { 0 });

        for i in 0..cfg.station_count.min(MAX_STATIONS) {
            let key = format!("ip_{}", i);
            let _ = ns.set_raw(&key, cfg.stations[i].as_bytes());
        }
    }
    log::info!("NVS saved");
}

// ─── MAVLink helpers ───────────────────────────────────────────────────────
fn send_to_fc(tx: &mut UartDriver, msg: &mavlink::MavMessage) {
    let mut buf = [0u8; mavlink::MAVLINK_MAX_PACKET_LEN];
    if let Ok(len) = msg.serialize(&mut buf) {
        let _ = tx.write(&buf[..len]);
    }
}

fn send_to_both(tx: &mut UartDriver, sock: &std::net::UdpSocket, msg: &mavlink::MavMessage, state: &State) {
    send_to_fc(tx, msg);
    forward_to_wifi(sock, msg, state);
}

fn forward_to_wifi(sock: &std::net::UdpSocket, msg: &mavlink::MavMessage, state: &State) {
    if !state.wifi_on {
        return;
    }
    let mut buf = [0u8; mavlink::MAVLINK_MAX_PACKET_LEN];
    if let Ok(len) = msg.serialize(&mut buf) {
        // static stations
        for i in 0..state.config.station_count.min(MAX_STATIONS) {
            let ip = &state.config.stations[i];
            if !ip.is_empty() {
                let _ = sock.send_to(&buf[..len], format!("{}:{}", ip, UDP_PORT));
            }
        }
        // dynamic clients
        for c in &state.udp_clients {
            if c.active {
                let addr = std::net::SocketAddrV4::new(
                    std::net::Ipv4Addr::new(c.ip[0], c.ip[1], c.ip[2], c.ip[3]),
                    c.port,
                );
                let _ = sock.send_to(&buf[..len], addr);
            }
        }
    }
}

fn send_statustext(tx: &mut UartDriver, text: &str) {
    let msg = common::MavMessage::STATUSTEXT(mavlink::common::STATUSTEXT_DATA {
        severity: mavlink::common::MAV_SEVERITY_INFO,
        text: {
            let mut t = [0u8; 50];
            let bytes = text.as_bytes();
            let n = bytes.len().min(49);
            t[..n].copy_from_slice(&bytes[..n]);
            t
        },
        id: 0,
        chunk_seq: 0,
    });
    send_to_fc(tx, &msg);
}

fn send_heartbeat(tx: &mut UartDriver, sock: &std::net::UdpSocket, state: &State) {
    let msg = common::MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
        `type`: mavlink::common::MAV_TYPE_ONBOARD_CONTROLLER,
        autopilot: mavlink::common::MAV_AUTOPILOT_INVALID,
        base_mode: mavlink::common::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode: 0,
        system_status: 0,
        mavlink_version: 3,
    });
    send_to_both(tx, sock, &msg, state);
}

fn send_radio_status(sock: &std::net::UdpSocket, state: &State) {
    if !state.wifi_on {
        return;
    }
    let msg = common::MavMessage::RADIO_STATUS(mavlink::common::RADIO_STATUS_DATA {
        rssi: 70,
        remrssi: 0,
        txbuf: 100,
        noise: 0,
        remnoise: 0,
        rxerrors: 0,
        fixed: 0,
    });
    let mut buf = [0u8; mavlink::MAVLINK_MAX_PACKET_LEN];
    if let Ok(len) = msg.serialize(&mut buf) {
        for i in 0..state.config.station_count.min(MAX_STATIONS) {
            let ip = &state.config.stations[i];
            if !ip.is_empty() {
                let _ = sock.send_to(&buf[..len], format!("{}:{}", ip, UDP_PORT));
            }
        }
    }
}

fn send_mission_request_list(tx: &mut UartDriver) {
    let msg = common::MavMessage::MISSION_REQUEST_LIST(mavlink::common::MISSION_REQUEST_LIST_DATA {
        target_system: 1,
        target_component: mavlink::common::MAV_COMP_ID_AUTOPILOT1,
        mission_type: mavlink::common::MAV_MISSION_TYPE_MISSION,
    });
    send_to_fc(tx, &msg);
}

fn send_mission_request_int(tx: &mut UartDriver, seq: u16) {
    let msg = common::MavMessage::MISSION_REQUEST_INT(mavlink::common::MISSION_REQUEST_INT_DATA {
        target_system: 1,
        target_component: mavlink::common::MAV_COMP_ID_AUTOPILOT1,
        seq,
        mission_type: mavlink::common::MAV_MISSION_TYPE_MISSION,
    });
    send_to_fc(tx, &msg);
}

fn send_disarm(tx: &mut UartDriver) {
    use mavlink::common::{MavCmd, MavFrame};
    let msg = common::MavMessage::COMMAND_LONG(mavlink::common::COMMAND_LONG_DATA {
        target_system: 1,
        target_component: mavlink::common::MAV_COMP_ID_AUTOPILOT1,
        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation: 0,
        param1: 0.0,
        param2: 21196.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    send_to_fc(tx, &msg);
    log::info!("[FAILSAFE] DISARM");
}

fn send_set_relay(tx: &mut UartDriver) {
    use mavlink::common::MavCmd;
    let msg = common::MavMessage::COMMAND_LONG(mavlink::common::COMMAND_LONG_DATA {
        target_system: 1,
        target_component: mavlink::common::MAV_COMP_ID_AUTOPILOT1,
        command: MavCmd::MAV_CMD_DO_SET_RELAY,
        confirmation: 0,
        param1: 0.0,
        param2: 1.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    send_to_fc(tx, &msg);
    log::info!("[FAILSAFE] RELAY");
}

fn execute_failsafe_async(state: &mut State) {
    state.failsafe_pending = true;
    state.failsafe_step = 0;
    state.failsafe_time = millis();
    state.failsafe_relay = state.relay_after_land;
    log::info!("[FAILSAFE] PENDING");
}

fn millis() -> u64 {
    (esp_idf_sys::esp_timer_get_time() / 1000) as u64
}

// ─── MAVLink обробка ──────────────────────────────────────────────────────
fn handle_mavlink_message(state: &mut State, msg: &mavlink::MavMessage) {
    match msg {
        MavMessage::HEARTBEAT(hb) => {
            state.heartbeat_received = true;
            state.current_custom_mode = hb.custom_mode;
            state.system_status = hb.system_status;
            if hb.custom_mode == TAKEOFF_MODE {
                state.takeoff_mode_detected = true;
            }
            state.is_armed = (hb.base_mode & mavlink::common::MAV_MODE_FLAG_SAFETY_ARMED as u8) != 0;

            if state.last_arm_state && !state.is_armed && !state.emergency_triggered && state.was_flying {
                state.emergency_triggered = true;
                // disarm & relay will be handled in main loop
                log::info!("[LAND] Disarm + action");
            }
            state.last_arm_state = state.is_armed;
        }
        MavMessage::MISSION_COUNT(mc) => {
            state.mission_count = mc.count as u16;
            state.mission_loaded = false;
            state.p_tg = Point::new();
            state.land_seen = false;
            state.relay_after_land = false;
            log::info!("MISSCOUNT={}", mc.count);
            if mc.count > 0 {
                // will request first item in main loop
            }
        }
        MavMessage::MISSION_ITEM_INT(item) => {
            use mavlink::common::MavCmd;
            if item.command == MavCmd::MAV_CMD_NAV_TAKEOFF {
                state.takeoff_detected = true;
            }
            if item.command == MavCmd::MAV_CMD_NAV_LAND && !state.p_tg.received {
                state.p_tg.lat = item.x;
                state.p_tg.lon = item.y;
                state.p_tg.received = true;
            }
            if item.command == MavCmd::MAV_CMD_DO_SET_RELAY && item.param2 > 0.9 {
                state.p_tg.relay = true;
            }
            if item.command == MavCmd::MAV_CMD_NAV_LAND {
                state.land_seen = true;
            }
            if state.land_seen && item.command == MavCmd::MAV_CMD_DO_SET_RELAY && item.param2 > 0.9 {
                state.relay_after_land = true;
                log::info!("[MISSION] RELAY {:.0} 1 after LAND", item.param1);
            }
            log::info!("ITEM: s={} c={} p={:.2}", item.seq, item.command as u32, item.param1);
        }
        MavMessage::ATTITUDE(att) => {
            state.roll = att.roll;
            state.pitch = att.pitch;
        }
        MavMessage::VFR_HUD(vfr) => {
            state.current_alt = vfr.alt;
            state.throttle = vfr.throttle as u16;
            state.ground_speed = vfr.groundspeed;

            if state.is_armed && state.current_alt > 2.0 {
                state.was_flying = true;
            }

            if !state.emergency_triggered && state.was_flying && state.is_armed {
                let alt_diff = (state.current_alt - state.last_stable_alt).abs();
                if alt_diff < 0.15 && state.ground_speed < 0.15 {
                    if state.stuck_timer == 0 {
                        state.stuck_timer = millis();
                    }
                    let dt = millis() - state.stuck_timer;
                    if dt > 3000 && state.throttle > 45 {
                        state.emergency_triggered = true;
                        execute_failsafe_async(state);
                        log::info!("[CRASH] Net");
                    }
                    if dt > 2500 && (state.roll.abs() > 0.18 || state.pitch.abs() > 0.18) && state.throttle > 25 {
                        state.emergency_triggered = true;
                        execute_failsafe_async(state);
                        log::info!("[CRASH] Roof");
                    }
                } else {
                    state.stuck_timer = 0;
                    state.last_stable_alt = state.current_alt;
                }
            }
        }
        MavMessage::RAW_IMU(imu) => {
            if !state.emergency_triggered && state.was_flying && state.is_armed {
                if imu.xgyro.abs() > 4500 || imu.ygyro.abs() > 4500 {
                    if state.gyro_crash_timer == 0 {
                        state.gyro_crash_timer = millis();
                    }
                    if millis() - state.gyro_crash_timer > 150 {
                        state.emergency_triggered = true;
                        execute_failsafe_async(state);
                        log::info!("[CRASH] Tumble");
                    }
                } else {
                    state.gyro_crash_timer = 0;
                }
            }
        }
        _ => {}
    }
}

// ─── Headless entry ────────────────────────────────────────────────────────
fn main() {
    // std::fmt::Write для String
    esp_idf_sys::link_patches();
    esp_idf_svc::sys::link_patches();

    // Logging
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().unwrap();

    // ─── Config ───
    let config = load_config(&nvs);
    let mut state = State::new(config);

    // ─── LED (WS2812) ───
    let mut led = Ws2812Esp32Rmt::new(peripherals.rmt.channel0, peripherals.pins.gpio48).unwrap();

    // ─── UART (FC) ───
    let uart_config = UartConfig::default().baudrate(Hertz(state.config.baud));
    let mut fc_uart = UartDriver::new(
        peripherals.uart0,
        peripherals.pins.gpio44,
        peripherals.pins.gpio43,
        Option::<PinDriver<_, _>>::None,
        Option::<PinDriver<_, _>>::None,
        &uart_config,
    )
    .unwrap();

    // ─── Boot button ───
    let mut boot_btn = PinDriver::input(peripherals.pins.gpio0).unwrap();

    // ─── UDP socket ───
    let sock = std::net::UdpSocket::bind(format!("0.0.0.0:{}", UDP_PORT)).unwrap();
    sock.set_nonblocking(true).unwrap();

    // ─── WiFi ───
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs.clone())).unwrap(),
        sysloop,
    )
    .unwrap();

    // ─── Main ───
    state.start_time = millis();
    let mut input_buf_str = HString::<128>::new();

    log::info!("=== ESP32-S3 DropCtrlV3 Tailscale Ready ===");
    log::info!("Terminal configuration active. Type 'STATUS' for info.");
    set_led(&mut led, RGB8::new(0, 0, 32)); // BLUE

    if state.config.wifi_boot {
        wifi_activate(&mut wifi, &mut state);
    }

    // ─── Loop ───
    let mut last_mission_item_req: u64 = 0;
    let mut ap_auto_start: u64 = millis();

    loop {
        let now = millis();

        // ─── Auto AP if no heartbeat ───
        if state.heartbeat_received || !state.wifi_on {
            ap_auto_start = now;
        } else if !state.heartbeat_received && !state.wifi_on && now - ap_auto_start > 10000 {
            log::info!("No FC heartbeat - activating WiFi for config");
            wifi_activate(&mut wifi, &mut state);
            ap_auto_start = now;
        }

        // ─── Terminal config ───
        handle_terminal_config(&mut state, &mut input_buf_str);

        // ─── WiFi to FC ───
        if state.wifi_on {
            bridge_wifi_to_fc(&sock, &mut fc_uart, &mut state);
        }

        // ─── FC to WiFi ───
        bridge_fc_to_wifi(&mut fc_uart, &sock, &mut state, &mut led);

        // ─── Periodic ───
        if now - state.last_wifi_hb >= 1000 {
            send_heartbeat(&mut fc_uart, &sock, &state);
            state.last_wifi_hb = now;
        }
        if now - state.last_radio_status >= 2000 {
            send_radio_status(&sock, &state);
            state.last_radio_status = now;
        }

        // ─── Boot button ───
        static mut LAST_BOOT_PRESS: u64 = 0;
        unsafe {
            if boot_btn.is_low() && now - LAST_BOOT_PRESS > 500 {
                FreeRtos::delay_ms(50);
                if boot_btn.is_low() {
                    if state.wifi_on {
                        wifi_deactivate(&mut wifi, &mut state);
                    } else {
                        wifi_activate(&mut wifi, &mut state);
                    }
                    log::info!("WiFi MANUAL {}", if state.wifi_on { "ON" } else { "OFF" });
                    LAST_BOOT_PRESS = now;
                }
            }
        }

        // ─── Mission scan ───
        if state.heartbeat_received && !state.mission_first_parsed {
            if state.last_mission_req == 0 && now - state.start_time > 2000 {
                state.mission_count = 0;
                state.mission_loaded = false;
                send_mission_request_list(&mut fc_uart);
                state.last_mission_req = now;
                log::info!("Mission scan started");
            } else if state.last_mission_req != 0 && now - state.last_mission_req > 5000 {
                state.mission_first_parsed = true;
                state.last_mission_req = 0;
                log::info!("Mission scan done");
            }
        }
        // Mission item requests
        if state.heartbeat_received && state.mission_count > 0 && !state.mission_loaded {
            if last_mission_item_req == 0 {
                send_mission_request_int(&mut fc_uart, 0);
                last_mission_item_req = now;
            }
        }

        // ─── Failsafe ───
        if state.failsafe_pending {
            if state.failsafe_step == 0 {
                send_disarm(&mut fc_uart);
                state.failsafe_step = 1;
                state.failsafe_time = now;
            } else if state.failsafe_step == 1 && now - state.failsafe_time >= 25 {
                if state.failsafe_relay {
                    send_set_relay(&mut fc_uart);
                }
                state.failsafe_step = 2;
                state.failsafe_time = now;
            } else if state.failsafe_step == 2 && now - state.failsafe_time >= 50 {
                state.failsafe_pending = false;
                log::info!("[FAILSAFE] Complete");
            }
        }

        // ─── WiFi activation retry ───
        if state.wifi_activating && wifi.is_connected().unwrap_or(false) {
            state.wifi_activating = false;
            state.sta_retry_count = 0;
            state.sta_was_connected = true;
            let ip = wifi.wifi().sta_netif().get_ip_info().unwrap().ip;
            log::info!("STA Connected! IP: {}", ip);
            send_statustext(&mut fc_uart, &format!("STA IP: {}", ip));
        }
        if state.wifi_activating && now - state.wifi_activate_time > 20000 {
            state.sta_retry_count += 1;
            if state.sta_retry_count <= 10 {
                log::info!("STA timeout. Retry {}/10...", state.sta_retry_count);
                wifi_activate(&mut wifi, &mut state);
                state.wifi_activate_time = now;
            } else {
                state.wifi_activating = false;
                log::info!("STA failed after 10 retries");
            }
        }
        if state.sta_was_connected && !wifi.is_connected().unwrap_or(false) && state.wifi_on
            && now - state.last_sta_reconnect >= 15000
        {
            state.last_sta_reconnect = now;
            let _ = wifi.wifi().sta_netif().reconnect();
            log::info!("Link lost. Reconnecting...");
        }

        // ─── LED ───
        set_led(&mut led, led_color(&state, now));

        // ─── Serial log ───
        if now - state.last_serial_log >= 5000 {
            log::info!(
                "hb={} arm={} mode={} wifi={} connected={} fc_bytes={} fc_msgs={}",
                state.heartbeat_received,
                state.is_armed,
                state.current_custom_mode,
                state.wifi_on,
                wifi.is_connected().unwrap_or(false),
                state.fc_bytes,
                state.fc_msgs,
            );
            state.last_serial_log = now;
        }

        FreeRtos::delay_ms(10);
    }
}

// ─── WiFi ──────────────────────────────────────────────────────────────────
fn wifi_activate(wifi: &mut BlockingWifi<EspWifi>, state: &mut State) {
    if state.wifi_on {
        return;
    }
    state.wifi_on = true;

    if state.config.sta_ssid.len() > 0 {
        let client_config = ClientConfiguration {
            ssid: state.config.sta_ssid.as_str().into(),
            password: state.config.sta_pass.as_str().into(),
            ..Default::default()
        };
        wifi.set_configuration(&Configuration::Client(client_config)).ok();
        wifi.start().ok();
        wifi.connect().ok();
        state.wifi_activating = true;
        state.wifi_activate_time = millis();
        log::info!("Connecting to STA: {}", state.config.sta_ssid);
    } else {
        log::info!("STA SSID is empty.");
    }
}

fn wifi_deactivate(wifi: &mut BlockingWifi<EspWifi>, state: &mut State) {
    if !state.wifi_on {
        return;
    }
    state.wifi_on = false;
    state.wifi_activating = false;
    state.sta_retry_count = 0;
    state.sta_was_connected = false;

    wifi.disconnect().ok();
    wifi.stop().ok();
    log::info!("WiFi OFF");
}

// ─── Bridge ────────────────────────────────────────────────────────────────
fn bridge_fc_to_wifi(
    fc: &mut UartDriver,
    sock: &std::net::UdpSocket,
    state: &mut State,
    _led: &mut Ws2812Esp32Rmt,
) {
    let mut buf = [0u8; BRIDGE_BUF_SIZE];
    if let Ok(n) = fc.read(&mut buf, 0) {
        if n > 0 {
            state.fc_bytes += n as u32;
            forward_to_wifi_raw(sock, &buf[..n], state);

            for &b in &buf[..n] {
                if state.mav_status.parse_byte(b) {
                    state.fc_msgs += 1;
                    if let Some(msg) = state.mav_status.message() {
                        handle_mavlink_message(state, &msg);
                    }
                }
            }
        }
    }
}

fn forward_to_wifi_raw(sock: &std::net::UdpSocket, data: &[u8], state: &State) {
    if !state.wifi_on {
        return;
    }
    for i in 0..state.config.station_count.min(MAX_STATIONS) {
        let ip = &state.config.stations[i];
        if !ip.is_empty() {
            let _ = sock.send_to(data, format!("{}:{}", ip, UDP_PORT));
        }
    }
    for c in &state.udp_clients {
        if c.active {
            let addr = std::net::SocketAddrV4::new(
                std::net::Ipv4Addr::new(c.ip[0], c.ip[1], c.ip[2], c.ip[3]),
                c.port,
            );
            let _ = sock.send_to(data, addr);
        }
    }
}

fn bridge_wifi_to_fc(sock: &std::net::UdpSocket, fc: &mut UartDriver, state: &mut State) {
    let mut buf = [0u8; BRIDGE_BUF_SIZE];
    loop {
        match sock.recv_from(&mut buf) {
            Ok((n, addr)) => {
                let _ = fc.write(&buf[..n]);
                log::info!("[GCS CMD] {} bytes from {}", n, addr);
                add_udp_client(state, addr);
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
            Err(_) => break,
        }
    }
}

fn add_udp_client(state: &mut State, addr: std::net::SocketAddr) {
    if let std::net::SocketAddr::V4(v4) = addr {
        let ip = v4.ip().octets();
        let port = v4.port();
        for c in &state.udp_clients {
            if c.active && c.ip == ip && c.port == port {
                return;
            }
        }
        for c in &mut state.udp_clients {
            if !c.active {
                c.ip = ip;
                c.port = port;
                c.active = true;
                state.gs_connected = true;
                return;
            }
        }
    }
}

// ─── Terminal config ───────────────────────────────────────────────────────
fn handle_terminal_config(state: &mut State, _buf: &mut HString<128>) {
    // Читаємо команди з USB Serial (stdin)
    let mut line = String::new();
    std::io::stdin().read_line(&mut line).ok();
    let trimmed = line.trim();
    if !trimmed.is_empty() {
        process_terminal_command(state, trimmed);
    }
}

fn process_terminal_command(state: &mut State, cmd: &str) {
    match cmd {
        "STATUS" => {
            log::info!("--- STATUS ---");
            log::info!("WiFi: {}", if state.wifi_on { "ON" } else { "OFF" });
            log::info!("SSID: {}", state.config.sta_ssid);
            log::info!("Baud: {}", state.config.baud);
            log::info!("Stations: {}/{}", state.config.station_count, MAX_STATIONS);
            for i in 0..state.config.station_count.min(MAX_STATIONS) {
                log::info!("  [{}] {}", i, state.config.stations[i]);
            }
        }
        _ if cmd.starts_with("SSID=") => {
            let val = &cmd[5..];
            state.config.sta_ssid = HString::from(val);
            log::info!("Set SSID to: {} (type SAVE to store)", val);
        }
        _ if cmd.starts_with("PASS=") => {
            let val = &cmd[5..];
            state.config.sta_pass = HString::from(val);
            log::info!("Set Password (type SAVE to store)");
        }
        _ if cmd.starts_with("BAUD=") => {
            let val = &cmd[5..];
            if let Ok(b) = val.parse::<u32>() {
                match b {
                    9600 | 19200 | 38400 | 57600 | 115200 | 230400 | 460800 | 921600 => {
                        state.config.baud = b;
                        log::info!("Set baud to: {}", b);
                    }
                    _ => log::info!("Invalid baudrate!"),
                }
            }
        }
        _ if cmd.starts_with("ADD_IP=") => {
            let val = &cmd[7..];
            if state.config.station_count < MAX_STATIONS {
                let mut s = HString::new();
                let _ = s.push_str(val);
                state.config.stations[state.config.station_count] = s;
                state.config.station_count += 1;
                log::info!("Added station: {}", val);
            } else {
                log::info!("Max stations reached");
            }
        }
        "WIFI=1" => {
            // will be activated in main loop
            state.config.wifi_boot = true;
            log::info!("WiFi auto ON at boot");
        }
        "WIFI=0" => {
            state.config.wifi_boot = false;
            log::info!("WiFi auto OFF at boot");
        }
        "RELAY" => {
            // send relay via FC - will be done when UART reference is available
            log::info!("RELAY command received (will send)");
        }
        "DISARM" => {
            log::info!("DISARM command received (will send)");
        }
        "CLEAR" => {
            state.config.station_count = 0;
            for s in &mut state.config.stations {
                s.clear();
            }
            log::info!("Station list cleared");
        }
        "SAVE" => {
            let nvs = EspDefaultNvsPartition::take().unwrap();
            save_config(&nvs, &state.config);
            log::info!("Saved! Rebooting...");
            esp_idf_sys::esp_restart();
        }
        _ => {
            log::info!(
                "Unknown: {}. Commands: STATUS, SSID=, PASS=, BAUD=, ADD_IP=, WIFI=1/0, CLEAR, SAVE",
                cmd
            );
        }
    }
}
