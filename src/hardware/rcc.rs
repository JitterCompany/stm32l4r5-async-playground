//! Peripheral reset and clock configuration module
//!

use cortex_m::interrupt::free;
// use stm32ral::rcc::CFGR::MCOSEL;
use stm32ral::{flash, rcc};
use stm32ral::{modify_reg, read_reg, reset_reg};

use fugit::{Hertz, RateExtU32};

pub struct Rcc {
    rcc: rcc::Instance,
}

/// Read only struct for obtaining clock rates.
pub struct Clocks {
    sysclk: u32,
}

impl Clocks {
    pub fn sysclk(&self) -> Hertz<u32> {
        self.sysclk.Hz()
    }

    pub fn hclk(&self) -> Hertz<u32> {
        let rcc = unsafe { &*rcc::RCC };
        let hpre = read_reg!(rcc, rcc, CFGR, HPRE);
        match hpre {
            0b1000 => self.sysclk() / 2,
            0b1001 => self.sysclk() / 4,
            0b1010 => self.sysclk() / 8,
            0b1011 => self.sysclk() / 16,
            0b1100 => self.sysclk() / 64,
            0b1101 => self.sysclk() / 128,
            0b1110 => self.sysclk() / 256,
            0b1111 => self.sysclk() / 512,
            _ => self.sysclk(),
        }
    }

    pub fn pclk1(&self) -> Hertz<u32> {
        let hclk = self.hclk();

        let rcc = unsafe { &*rcc::RCC };
        let ppre = read_reg!(rcc, rcc, CFGR, PPRE1);
        match ppre {
            0b100 => hclk / 2,
            0b101 => hclk / 4,
            0b110 => hclk / 8,
            0b111 => hclk / 16,
            _ => hclk,
        }
    }

    pub fn pclk2(&self) -> Hertz<u32> {
        let hclk = self.hclk();

        let rcc = unsafe { &*rcc::RCC };
        let ppre = read_reg!(rcc, rcc, CFGR, PPRE2);
        match ppre {
            0b100 => hclk / 2,
            0b101 => hclk / 4,
            0b110 => hclk / 8,
            0b111 => hclk / 16,
            _ => hclk,
        }
    }
}

impl Rcc {
    pub fn new(rcc: rcc::Instance) -> Self {
        Rcc { rcc }
    }

    pub fn setup(&self) -> Clocks {
        // At power up we're running from the MSI at 4 MHz
        // Wait for it to be ready
        while read_reg!(rcc, self.rcc, CR, MSIRDY != 1) {}

        // start setting up PLL

        // Disable PLL
        modify_reg!(rcc, self.rcc, CR, PLLON: 0);
        // Wait until it RDY bit is cleared
        while read_reg!(rcc, self.rcc, CR, PLLRDY != 0) {}

        // We want to run at 80MHz
        // VOS is in Range1 by default, so up to 80MHz we can leave it as is.
        // PLLCLK = input / M x N / R
        // PLLQ (PLL48M1CLK) = input / M x N / Q
        // PLLP (PLLSAI3CLK) = input / M x N / P
        // M = 1
        // N = 40
        // R = 2
        let pllm = 0u32; // divider of 1
        let plln = 40u32; // multiplier of 40
        let pllr = 0u32; // divider of 2
        let pllq = 0u32; // divider of 2
        let pllp = 0u32; // divider of 7
        let pllpdiv = 0u32;
        let sysclk = 80_000_000;

        // Configure PLL from MSI
        modify_reg!(
            rcc,
            self.rcc,
            PLLCFGR,
            PLLSRC: 0b01, // MSI
            PLLM: pllm,
            PLLR: pllr,
            PLLN: plln,
            PLLP: pllp,
            PLLQ: pllq,
            PLLPDIV: pllpdiv
        );

        // Enable PLL
        modify_reg!(rcc, self.rcc, CR, PLLON: 1);
        // Wait until it RDY bit if cleared
        while read_reg!(rcc, self.rcc, CR, PLLRDY == 1) {}

        // Enable PLL Output
        modify_reg!(rcc, self.rcc, PLLCFGR, PLLREN: 1);

        // Disable all peripherals clocks
        reset_reg!(rcc, self.rcc, RCC, AHB1ENR);
        reset_reg!(rcc, self.rcc, RCC, AHB2ENR);
        reset_reg!(rcc, self.rcc, RCC, AHB3ENR);

        reset_reg!(rcc, self.rcc, RCC, APB1ENR1);
        reset_reg!(rcc, self.rcc, RCC, APB1ENR2);
        reset_reg!(rcc, self.rcc, RCC, APB2ENR);

        // Adjust flash wait states. For 80 MHz we need 3 WS (4 CPU cycles)
        unsafe { modify_reg!(flash, flash::FLASH, ACR, LATENCY: 0b11) }

        // Swap system clock to PLL
        modify_reg!(rcc, self.rcc, CFGR, SW: 0b11u32);

        // Wait for system clock to be PLL
        while read_reg!(rcc, self.rcc, CFGR, SWS != 0b11) {}

        // // CLOCK OUT
        // // Divide clock out by 8
        // modify_reg!(rcc, self.rcc, CFGR, MCOPRE: 0b011);

        // // SYSCLK to clock-out, enable
        // modify_reg!(rcc, self.rcc, CFGR, MCOSEL: 0b0001);

        Clocks { sysclk }
    }
}

/// Enables the peripheral clock for the USART1 peripheral
/// And resets the peripheral
pub fn enable_rst_usart2() {
    free(|_| {
        unsafe {
            // Enable peripheral clock
            modify_reg!(rcc, RCC, APB1ENR1, USART2EN: 1);
            // Reset peripheral
            modify_reg!(rcc, RCC, APB1RSTR1, USART2RST: 1);
            modify_reg!(rcc, RCC, APB1RSTR1, USART2RST: 0);
        };
    });
}

/// Enables the peripheral clock for the TIMER2 peripheral
/// And resets the peripheral
pub fn enable_rst_timer2() {
    free(|_| {
        unsafe {
            // Enable peripheral clock
            modify_reg!(rcc, RCC, APB1ENR1, TIM2EN: 1);
            // Reset peripheral
            modify_reg!(rcc, RCC, APB1RSTR1, TIM2RST: 1);
            modify_reg!(rcc, RCC, APB1RSTR1, TIM2RST: 0);
        };
    });
}

/// Enables the peripheral clock for the TIMER5 peripheral
/// And resets the peripheral
pub fn enable_rst_timer5() {
    free(|_| {
        unsafe {
            // Enable peripheral clock
            modify_reg!(rcc, RCC, APB1ENR1, TIM5EN: 1);
            // Reset peripheral
            modify_reg!(rcc, RCC, APB1RSTR1, TIM5RST: 1);
            modify_reg!(rcc, RCC, APB1RSTR1, TIM5RST: 0);
        };
    });
}

use paste::paste;

macro_rules! enable_rst_gpio {
    ($port: tt) => {
        paste! {
            /// Enables and resets the peripheral.
            pub fn [<enable_rst_ $port:lower>] () {
                free(|_| {
                    unsafe {
                        modify_reg!(rcc, RCC, AHB2ENR, [<$port EN>]: 1);
                        modify_reg!(rcc, RCC, AHB2RSTR, [<$port RST>]: 1);
                        modify_reg!(rcc, RCC, AHB2RSTR, [<$port RST>]: 0);
                    }
                })
            }

            /// Returns true if the peripheral is enabled.
            pub fn [<$port:lower _is_enabled>] () -> bool {
                unsafe { read_reg!(stm32ral::rcc, RCC, AHB2ENR, [<$port EN>] == 1) }
            }

        }
    };
}

enable_rst_gpio!(GPIOA);
enable_rst_gpio!(GPIOB);
enable_rst_gpio!(GPIOC);
enable_rst_gpio!(GPIOD);
enable_rst_gpio!(GPIOE);
enable_rst_gpio!(GPIOF);
enable_rst_gpio!(GPIOG);
