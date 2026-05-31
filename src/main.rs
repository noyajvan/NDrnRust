//! ESP32-S3 DropCtrlV3 — no_std Mavlink Bridge
//!
//! Піни:
//!   - WS2812 LED:  GPIO 48
//!   - FC UART TX:   GPIO 43
//!   - FC UART RX:   GPIO 44
//!   - BOOT button:  GPIO 0 (pull-up, active low)

#![no_std]
#![no_main]

extern crate alloc;

use alloc::vec::Vec;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Input, Io, Level, Output, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    uart::{config::Config, TxRxPins, Uart},
};
use esp_println::println;

// ─── WS2812 / SK6812 ──────────────────────────────────────────────────────
mod rgb {
    use esp_hal::rmt::{PulseCode, RmtTxChannel, TxChannelConfig, TxChannelCreator};

    const T0H: u16 = 8; // 0.35 us @ 40 MHz
    const T0L: u16 = 16;
    const T1H: u16 = 16;
    const T1L: u16 = 8;

    pub struct Color(pub u8, pub u8, pub u8); // R, G, B

    pub struct Led;

    impl Led {
        pub fn new(channel: impl RmtTxChannel) -> Self {
            let rmt = esp_hal::rmt::Rmt::new(unsafe { esp_hal::peripherals::RMT::steal() }, 80.MHz()).unwrap();
            let _ = rmt.channel0.configure(
                unsafe { esp_hal::peripherals::GPIO::steal() },
                TxChannelConfig {
                    clk_divider: 1,
                    io_pin: 48,
                    ..Default::default()
                },
            );
            Self
        }

        pub fn set(&mut self, color: Color) {
            let colors = [color.1, color.0, color.2]; // GRB
            let mut pulses = [PulseCode::new(0, 0); 24];

            for (i, &byte) in colors.iter().enumerate() {
                for bit in 0..8 {
                    let idx = i * 8 + bit;
                    if byte & (1 << (7 - bit)) != 0 {
                        pulses[idx] = PulseCode::new(T1H, T1L);
                    } else {
                        pulses[idx] = PulseCode::new(T0H, T0L);
                    }
                }
            }
            // For now, just a placeholder — RMT needs proper channel setup
            core::hint::black_box(&pulses);
        }
    }
}

// ─── MAVLink (мінімальний парсер) ─────────────────────────────────────────
mod mavlink_mini {
    const MAGIC_V1: u8 = 0xFE;
    const MAGIC_V2: u8 = 0xFD;

    #[repr(u8)]
    pub enum MsgId {
        Heartbeat = 0,
        MissionCount = 44,
        MissionItemInt = 73,
        Attitude = 30,
        VfrHud = 74,
        RawImu = 27,
    }

    pub struct Parser {
        buf: [u8; 280],
        pos: usize,
        framing: bool,
        len: usize,
        incompat: u8,
        seq: u8,
        sysid: u8,
        compid: u8,
        msgid: u32,
        payload: [u8; 255],
    }

    impl Parser {
        pub fn new() -> Self {
            Self {
                buf: [0; 280],
                pos: 0,
                framing: false,
                len: 0,
                incompat: 0,
                seq: 0,
                sysid: 0,
                compid: 0,
                msgid: 0,
                payload: [0; 255],
            }
        }

        pub fn parse_byte(&mut self, byte: u8) -> bool {
            if !self.framing {
                if byte == MAGIC_V2 {
                    self.framing = true;
                    self.pos = 0;
                }
                self.buf[0] = byte;
                self.pos = 1;
                return false;
            }

            if self.pos < self.buf.len() {
                self.buf[self.pos] = byte;
            }
            self.pos += 1;

            if self.pos >= 10 {
                if self.buf[0] == MAGIC_V2 {
                    self.len = self.buf[1] as usize;
                    self.incompat = self.buf[2];
                    if self.pos >= self.len + 12 {
                        self.framing = false;
                        self.msgid = self.buf[5] as u32
                            | (self.buf[6] as u32) << 8
                            | (self.buf[7] as u32) << 16;
                        // payload starts at offset 10
                        let payload_start = 10;
                        for i in 0..self.len.min(255) {
                            self.payload[i] = self.buf[payload_start + i];
                        }
                        return true;
                    }
                }
            }
            false
        }

        pub fn msgid(&self) -> u32 {
            self.msgid
        }

        pub fn payload(&self) -> &[u8] {
            &self.payload[..self.len.min(255)]
        }
    }
}

// ─── Головна функція ──────────────────────────────────────────────────────
#[entry]
fn main() -> ! {
    // Ініціалізація
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    // Аллокатор
    init_heap();

    // Затримки
    let mut delay = Delay::new(&clocks);

    // GPIO
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // BOOT button
    let mut boot_btn = Input::new(io.pins.gpio0, Pull::Up);

    // Сигнальний LED (GPIO 15 — якщо є на платі)
    let mut signal_led = Output::new(io.pins.gpio15, Level::Low);

    // UART для зв'язку з польотним контролером
    let uart_config = Config::default().baudrate(115200.Hz());
    let (tx_pin, rx_pin) = (io.pins.gpio43, io.pins.gpio44);
    let mut fc_uart = Uart::new_with_config(
        peripherals.UART0,
        uart_config,
        TxRxPins::new(tx_pin, rx_pin),
        &clocks,
    );

    // USB Serial (JTAG)
    esp_println::init();

    // MAVLink парсер
    let mut parser = mavlink_mini::Parser::new();

    let mut counter: u32 = 0;

    println!("╔══════════════════════════════╗");
    println!("║ DropCtrlV3  (ESP32-S3 no_std)║");
    println!("╚══════════════════════════════╝");

    loop {
        counter = counter.wrapping_add(1);

        // ─── Читання UART (FC) ─────
        let mut buf = [0u8; 64];
        if let Ok(n) = fc_uart.read_bytes(&mut buf, 10.millis()) {
            if n > 0 {
                for &b in &buf[..n] {
                    if parser.parse_byte(b) {
                        let id = parser.msgid();
                        match id {
                            0 => {} // Heartbeat
                            74 => {} // VFR_HUD
                            _ => {}  // Інші
                        }
                    }
                }
                // Передаємо далі на USB Serial
                for &b in &buf[..n] {
                    print!("{:02x}", b);
                }
            }
        }

        // ─── Кнопка BOOT ──────────
        if boot_btn.is_low() {
            signal_led.set_high();
            println!(" BOOT!");
            delay.delay_millis(200);
            signal_led.set_low();
        }

        // ─── LED blink ────────────
        if counter % 100 == 0 {
            signal_led.toggle();
            println!("[{}] alive", counter);
        }

        delay.delay_millis(10);
    }
}

fn init_heap() {
    use esp_alloc::Heap;
    const HEAP_SIZE: usize = 64 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe {
        Heap::init(HEAP.as_mut_ptr() as usize, HEAP_SIZE);
    }
}
