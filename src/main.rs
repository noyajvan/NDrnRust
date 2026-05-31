//! ESP32-S3 DropCtrlV3 — no_std Mavlink Bridge
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

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // LED на GPIO 15
    let mut led = Output::new(peripherals.GPIO15, Level::Low);

    // Кнопка BOOT (GPIO 0, підтяжка вгору)
    let boot_btn = Input::new(peripherals.GPIO0, Pull::Up);

    // UART для зв'язку з польотним контролером (FC)
    let uart_config = esp_hal::uart::Config::default()
        .with_baudrate(115200)
        .with_data_bits(esp_hal::uart::DataBits::DataBits8)
        .with_stop_bits(esp_hal::uart::StopBits::STOP1)
        .with_parity(esp_hal::uart::Parity::ParityNone);
    let mut fc_uart = Uart::new(peripherals.UART0, uart_config)
        .unwrap()
        .with_tx(peripherals.GPIO43)
        .with_rx(peripherals.GPIO44);

    // Буфер для прийому
    let mut rx_buf = [0u8; 64];

    // Мінімальний парсер MAVLink V2
    let mut mav_pos: usize = 0;
    let mut mav_framing = false;
    let mut mav_buf = [0u8; 280];

    let mut counter: u32 = 0;

    loop {
        counter = counter.wrapping_add(1);

        // Прийом даних з FC
        if let Ok(byte) = fc_uart.read_byte() {
            // Парсинг MAVLink V2
            if !mav_framing {
                if byte == 0xFD {
                    mav_framing = true;
                    mav_pos = 0;
                }
                mav_buf[mav_pos] = byte;
                mav_pos = 1;
            } else {
                if mav_pos < mav_buf.len() {
                    mav_buf[mav_pos] = byte;
                }
                mav_pos += 1;
                // Повне повідомлення: заголовок (10) + payload (len) + checksum (2)
                if mav_pos >= 12 && mav_buf[0] == 0xFD {
                    let payload_len = mav_buf[1] as usize;
                    if mav_pos >= payload_len + 12 {
                        let msgid = mav_buf[5] as u32
                            | (mav_buf[6] as u32) << 8
                            | (mav_buf[7] as u32) << 16;
                        match msgid {
                            0 => {} // Heartbeat
                            74 => {} // VFR_HUD
                            _ => {}
                        }
                        mav_framing = false;
                    }
                }
            }
        }

        // Кнопка BOOT
        if boot_btn.is_low() {
            led.set_high();
            let press_start = Instant::now();
            while press_start.elapsed() < Duration::from_millis(200) {}
            led.set_low();
        }

        // LED мигалка кожні 2.5с
        if counter % 250 == 0 {
            led.toggle();
        }

        // Затримка ~10ms
        let loop_start = Instant::now();
        while loop_start.elapsed() < Duration::from_millis(10) {}
    }
}
