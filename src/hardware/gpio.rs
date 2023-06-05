//! GPIO abstraction for configuring and controling mcu pins.
//!
//! Note that this module currently still allows creating multiple instances of the same pin.

use cortex_m::interrupt::free;
use paste::paste;
use stm32ral::{gpio, modify_reg, read_reg, write_reg};

use super::rcc;

macro_rules! set_field {
    ($inst:expr, $pin:expr, $reg:ident, $field:ident, $val:expr) => {
        paste! {
            match $pin {
                PinN::P0 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 0>]: $val),
                PinN::P1 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 1>]: $val),
                PinN::P2 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 2>]: $val),
                PinN::P3 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 3>]: $val),
                PinN::P4 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 4>]: $val),
                PinN::P5 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 5>]: $val),
                PinN::P6 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 6>]: $val),
                PinN::P7 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 7>]: $val),
                PinN::P8 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 8>]: $val),
                PinN::P9 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 9>]: $val),
                PinN::P10 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 10>]: $val),
                PinN::P11 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 11>]: $val),
                PinN::P12 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 12>]: $val),
                PinN::P13 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 13>]: $val),
                PinN::P14 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 14>]: $val),
                PinN::P15 => modify_reg!(stm32ral::gpio, $inst, $reg, [<$field 15>]: $val)
            }
        }
    };
}

/// Values for `GPIOx_MODER`. Sets pin to input, output, and other functionality.
#[derive(Clone)]
#[repr(u8)]
pub enum PinMode {
    /// An input pin; read by firmware; set by something connected to the pin.
    Input,
    /// An input pin; set by firmware; read by something connected to the pin.
    Output,
    /// An alternate function, as defined in the MCU's user manual. Used for various
    /// onboard peripherals like buses, timers etc.
    Alt(u8),
    /// For use with the onboard ADC and DAC. Prevent parasitic power loss on the pin
    // if using it for one of these functionalities.
    Analog,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_IDR` and `GPIOx_ODR`.
pub enum PinState {
    High = 1,
    Low = 0,
}

impl PinMode {
    /// We use this function to find the value bits due to being unable to repr(u8) with
    /// the wrapped `AltFn` value.
    fn val(&self) -> u8 {
        match self {
            Self::Input => 0b00,
            Self::Output => 0b01,
            Self::Alt(_) => 0b10,
            Self::Analog => 0b11,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Port {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum PinN {
    P0,
    P1,
    P2,
    P3,
    P4,
    P5,
    P6,
    P7,
    P8,
    P9,
    P10,
    P11,
    P12,
    P13,
    P14,
    P15,
}

/// Represents a single GPIO pin. Allows configuration, and reading/setting state.
pub struct Pin {
    /// The GPIO Port letter. Eg A, B, C.
    pub port: Port,
    /// The pin number: 0 - 15.
    pub pin: PinN,
}

impl Pin {
    /// Create a new pin, with a specific mode. Enables the RCC peripheral clock to the port,
    /// if not already enabled. Example: `let pa1 = Pin::new(Port::A, PinN::P1, PinMode::Output);` Leaves settings
    /// other than mode and alternate function (if applicable) at their hardware defaults.
    pub fn new(port: Port, pin: PinN, mode: PinMode) -> Self {
        free(|_| {
            // Turn on GPIO peripheral if not already enabled
            match port {
                Port::A => {
                    if !rcc::gpioa_is_enabled() {
                        rcc::enable_rst_gpioa()
                    }
                }
                Port::B => {
                    if !rcc::gpiob_is_enabled() {
                        rcc::enable_rst_gpiob()
                    }
                }
                Port::C => {
                    if !rcc::gpioc_is_enabled() {
                        rcc::enable_rst_gpioc()
                    }
                }
                Port::D => {
                    if !rcc::gpiod_is_enabled() {
                        rcc::enable_rst_gpiod()
                    }
                }
                Port::E => {
                    if !rcc::gpioe_is_enabled() {
                        rcc::enable_rst_gpioe()
                    }
                }
                Port::F => {
                    if !rcc::gpiof_is_enabled() {
                        rcc::enable_rst_gpiof()
                    }
                }
                Port::G => {
                    if !rcc::gpiog_is_enabled() {
                        rcc::enable_rst_gpiog()
                    }
                }
            }
        });

        let mut new_pin = Self { port, pin };

        new_pin.mode(mode);

        new_pin
    }

    /// Set a pin state (ie set high or low output voltage level). See also `set_high()` and
    /// `set_low()`. Sets the `BSRR` register. Atomic.
    pub fn set_state(&mut self, value: PinState) {
        let offset = match value {
            PinState::Low => 16,
            PinState::High => 0,
        };

        write_reg!(
            stm32ral::gpio,
            unsafe { instance(self.port) },
            BSRR,
            1 << ((self.pin as u32) + offset)
        );
    }

    /// Read the input data register. Eg determine if the pin is high or low. See also `is_high()`
    /// and `is_low()`. Reads from the `IDR` register.
    pub fn get_state(&self) -> PinState {
        let port = read_reg!(stm32ral::gpio, unsafe { instance(self.port) }, IDR);

        if (port & (1 << self.pin as u8)) > 0 {
            PinState::High
        } else {
            PinState::Low
        }
    }

    /// Check if the pin's input voltage is high. Reads from the `IDR` register.
    pub fn is_high(&self) -> bool {
        match self.get_state() {
            PinState::High => true,
            PinState::Low => false,
        }
    }

    /// Check if the pin's input voltage is low. Reads from the `IDR` register.
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Set the pin's output voltage to high. Sets the `BSRR` register. Atomic.
    pub fn set_high(&mut self) {
        self.set_state(PinState::High);
    }

    /// Set the pin's output voltage to low. Sets the `BSRR` register. Atomic.
    pub fn set_low(&mut self) {
        self.set_state(PinState::Low);
    }

    /// Toggle output voltage between low and high.
    /// Note: reads current pin state before changing it.
    pub fn toggle(&mut self) {
        if self.is_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }

    pub fn mode(&mut self, value: PinMode) {
        set_field!(
            unsafe { instance(self.port) },
            self.pin,
            MODER,
            MODER,
            value.val() as u32
        );

        if let PinMode::Alt(alt) = value {
            self.alt_fn(alt as u32);
        }
    }

    /// Set up a pin's alternate function. We set this up initially using `mode()`.
    fn alt_fn(&mut self, value: u32) {
        assert!(value <= 15, "Alt function must be 0 to 15.");

        let instance = unsafe { instance(self.port) };
        match self.pin {
            PinN::P0 => modify_reg!(stm32ral::gpio, instance, AFRL, AFRL0: value),
            PinN::P1 => modify_reg!(stm32ral::gpio, instance, AFRL, AFRL1: value),
            PinN::P2 => modify_reg!(stm32ral::gpio, instance, AFRL, AFRL2: value),
            PinN::P3 => modify_reg!(stm32ral::gpio, instance, AFRL, AFRL3: value),
            PinN::P4 => modify_reg!(stm32ral::gpio, instance, AFRL, AFRL4: value),
            PinN::P5 => modify_reg!(stm32ral::gpio, instance, AFRL, AFRL5: value),
            PinN::P6 => modify_reg!(stm32ral::gpio, instance, AFRL, AFRL6: value),
            PinN::P7 => modify_reg!(stm32ral::gpio, instance, AFRL, AFRL7: value),
            PinN::P8 => modify_reg!(stm32ral::gpio, instance, AFRH, AFRH8: value),
            PinN::P9 => modify_reg!(stm32ral::gpio, instance, AFRH, AFRH9: value),
            PinN::P10 => modify_reg!(stm32ral::gpio, instance, AFRH, AFRH10: value),
            PinN::P11 => modify_reg!(stm32ral::gpio, instance, AFRH, AFRH11: value),
            PinN::P12 => modify_reg!(stm32ral::gpio, instance, AFRH, AFRH12: value),
            PinN::P13 => modify_reg!(stm32ral::gpio, instance, AFRH, AFRH13: value),
            PinN::P14 => modify_reg!(stm32ral::gpio, instance, AFRH, AFRH14: value),
            PinN::P15 => modify_reg!(stm32ral::gpio, instance, AFRH, AFRH15: value),
        }
    }
}

unsafe fn instance(port: Port) -> gpio::Instance {
    match port {
        Port::A => gpio::GPIOA::steal(),
        Port::B => gpio::GPIOB::steal(),
        Port::C => gpio::GPIOC::steal(),
        Port::D => gpio::GPIOD::steal(),
        Port::E => gpio::GPIOE::steal(),
        Port::F => gpio::GPIOF::steal(),
        Port::G => gpio::GPIOG::steal(),
    }
}
