//! ESP32-S3 DropCtrlV3 — no_std Mavlink Bridge
//!
//! Піни:
//!   - WS2812 LED:  GPIO 48
//!   - FC UART TX:   GPIO 43
//!   - FC UART RX:   GPIO 44
//!   - BOOT button:  GPIO 0

#![no_std]
#![no_main]

extern crate alloc;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Input, Io, Level, Output, Pull},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    uart::Uart,
};
use esp_println::println;

// ─── MAVLink парсер ───────────────────────────────────────────────────────
mod mavlink {
    const MAGIC_V2: u8 = 0xFD;

    pub struct Parser {
        buf: [u8; 280],
        pos: usize,
        framing: bool,
        len: usize,
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
            if self.pos >= 10 && self.buf[0] == MAGIC_V2 {
                self.len = self.buf[1] as usize;
                if self.pos >= self.len + 12 {
                    self.framing = false;
                    self.msgid = self.buf[5] as u32
                        | (self.buf[6] as u32) << 8
                        | (self.buf[7] as u32) << 16;
                    for i in 0..self.len.min(255) {
                        self.payload[i] = self.buf[10 + i];
                    }
                    return true;
                }
            }
            false
        }

        pub fn msgid(&self) -> u32 { self.msgid }
    }
}

// ─── Головна функція ──────────────────────────────────────────────────────
#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    init_heap();

    let delay = Delay::new(&clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let boot_btn = Input::new(io.pins.gpio0, Pull::Up);
    let mut led = Output::new(io.pins.gpio15, Level::Low);

    // UART для FC (GPIO 43 TX, GPIO 44 RX)
    let mut fc_uart = Uart::new(
        peripherals.UART0,
        esp_hal::uart::Config::default(),
        (io.pins.gpio43, io.pins.gpio44),
        &clocks,
    )
    .unwrap();

    esp_println::init();

    let mut parser = mavlink::Parser::new();
    let mut counter: u32 = 0;

    println!("╔══════════════════════════════╗");
    println!("║ DropCtrlV3  (ESP32-S3 no_std)║");
    println!("╚══════════════════════════════╝");

    loop {
        counter = counter.wrapping_add(1);

        // Читання UART
        let mut buf = [0u8; 64];
        if let Ok(n) = fc_uart.read(&mut buf) {
            if n > 0 {
                for &b in &buf[..n] {
                    if parser.parse_byte(b) {
                        match parser.msgid() {
                            0 => {} // Heartbeat
                            74 => {} // VFR_HUD
                            _ => {}
                        }
                    }
                }
            }
        }

        // Кнопка
        if boot_btn.is_low() {
            led.set_high();
            println!(" BOOT!");
            // затримка через delay
            core::hint::spin_loop();
            led.set_low();
        }

        if counter % 1000 == 0 {
            led.toggle();
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
