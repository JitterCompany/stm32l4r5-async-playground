#![allow(unused)]
//! This is the hardware Abstraction Layer (HAL) or the Board Support Package (BSP) for Frogwatch.
//! This crate should hide most of the implementation details of the MCU and the PCB.
//! It exposes high level APIs to control the device.
//!
//! Drivers are written interrupt based.
//! Interrupt handlers should be placed in the application and should call the correct driver.
//! This means the application is not fully hardware agnostic.

mod gpio;
pub mod mono;
mod rcc;
mod timer;
// mod uart;
pub mod uartasync;

/// Expose RAL for rtic to use as a device.
/// Note: RTIC will take _all_ peripherals.
pub use stm32ral as rtic_device;

pub use self::uartasync::UART;

use self::gpio::{PinMode, PinN, Port};
// use self::uart::*;

pub use self::gpio::Pin;
pub use self::timer::Timer;

// Inspired by https://gist.github.com/peter9477/1c8e6496a99df173ed8aaee6aad9e9a9

/// The Board struct exposes all initialized drivers.
/// All members are public and are intended to be moved out all at the same time.
pub struct Board {
    pub clocks: rcc::Clocks,
    pub red_led: Pin,
    pub blue_led: Pin,
    pub green_led: Pin,
    pub timer: Timer<1_000_000>,
    pub uart: UART,
}

/// This is the entry function for the hardware crate.
/// The application needs to call this in order to gain access to board functionality.
///
/// # Example
///
/// ```rust
/// let (_, mut hw) = hardware::board_init(cx.device);
/// let mut led = hw.led;
/// led.toggle();
/// ```
pub fn board_init(device: stm32ral::Peripherals) -> Board {
    let rcc = rcc::Rcc::new(device.RCC);
    let clocks = rcc.setup();
    let sysclk = clocks.sysclk().to_MHz();
    log::info!("Sysclk: {sysclk:?} MHz");

    let red_led = Pin::new(Port::B, PinN::P14, PinMode::Output);
    let green_led = Pin::new(Port::C, PinN::P7, PinMode::Output);
    let blue_led = Pin::new(Port::B, PinN::P7, PinMode::Output);

    let modem_tx = Pin::new(Port::D, PinN::P5, PinMode::Alt(7));
    let modem_rx = Pin::new(Port::D, PinN::P6, PinMode::Alt(7));

    let timer = timer::Timer::new(device.TIM2, &clocks);

    // Init monotonic timer required by rtic
    mono::Mono::start(device.TIM5, &clocks);
    // let _clock_out = Pin::new(Port::A, PinN::P8, PinMode::Alt(0));

    let uart = UART::new(device.USART2, modem_tx, modem_rx,  &clocks);

    Board {
        clocks,
        red_led,
        green_led,
        blue_led,
        timer,
        uart,
    }
}
