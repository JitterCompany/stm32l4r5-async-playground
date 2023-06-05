//! Interrupt driven UART driver
//!
//! This driver is implemented by splitting itself in two part, the interface part and the interrupt part.
//!
//! Note: UARTInterrupt should run in higher priority context than UARTInterface
//!
use core::ops::Deref;

use embedded_hal_nb::nb;
use embedded_hal_nb::serial::{Error as HALErrorTrait, ErrorKind, ErrorType, Read, Write};

use heapless::spsc::{Consumer, Producer, Queue};
use stm32ral::{modify_reg, reset_reg, write_reg};
use stm32ral::{read_reg, usart};

use super::{
    gpio::Pin,
    rcc::{self, Clocks},
};

#[derive(Debug)]
pub enum Error {
    SomeError,
}

impl HALErrorTrait for Error {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

static mut Q_TX: Queue<u8, 1024> = Queue::new();
static mut Q_RX: Queue<u8, 1024> = Queue::new();

pub struct UARTInterface {
    tx_producer: Producer<'static, u8, 1024>,
    rx_consumer: Consumer<'static, u8, 1024>,
}

pub struct UARTInterrupt {
    uart: usart::Instance,
    rx_producer: Producer<'static, u8, 1024>,
    tx_consumer: Consumer<'static, u8, 1024>,
    _tx: Pin,
    _rx: Pin,
}

pub fn create_modem_uart(
    uart: usart::Instance,
    tx: Pin,
    rx: Pin,
    clocks: &Clocks,
) -> (UARTInterface, UARTInterrupt) {
    if uart.deref() as *const _ != usart::USART2 {
        panic!("UART must be USART2");
    }
    // Step 1: enable and reset peripheral in RCC
    rcc::enable_rst_usart2();

    // Reset control registers
    // Maybe this is not strictly necessary
    reset_reg!(usart, uart, USART2, CR1);
    reset_reg!(usart, uart, USART2, CR2);
    reset_reg!(usart, uart, USART2, CR3);

    // Configure baudrate
    // USART1 gets its clock PLK2 via the APB2_PRESCALER
    // USART2-5 gets their clock PLK1 via the APB1_PRESCALER
    // For oversampling 16 (default)
    // baud = usart_ker_clk / usart_div
    // BRR = usart_div
    let baudrate = 115200u32;
    let usart_div = (clocks.pclk1() / baudrate).raw();

    modify_reg!(usart, uart, BRR, BRR: usart_div);

    // Configure CR1
    modify_reg!(usart, uart, CR1,
        FIFOEN: 1,
        TE: Enabled,
        RE: Enabled,
        RTOIE: 1,
        TXFEIE: 0 // Fifo empty interrupt
    );

    modify_reg!(usart, uart, CR2, RTOEN: Enabled);

    modify_reg!(usart, uart, CR3,
        RXFTIE: 1,
        RXFTCFG: 0b011 // 3/4 depth = 6 bytes
    );

    // Set receiver timeout to 10 bits
    modify_reg!(usart, uart, RTOR, RTO: 10);

    // Clear all interrupts
    write_reg!(usart, uart, ICR, 0x00123BFF);

    // Enable peripheral
    modify_reg!(usart, uart, CR1, UE: Enabled);

    // Enable interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(stm32ral::Interrupt::USART2);
    }

    let (tx_producer, tx_consumer) = unsafe { Q_TX.split() };
    let (rx_producer, rx_consumer) = unsafe { Q_RX.split() };

    (
        UARTInterface {
            tx_producer,
            rx_consumer,
        },
        UARTInterrupt {
            uart,
            rx_producer,
            tx_consumer,
            _tx: tx,
            _rx: rx,
        },
    )
}

pub struct Tx {
    tx_producer: Producer<'static, u8, 1024>,
}

pub struct Rx {
    rx_consumer: Consumer<'static, u8, 1024>,
}

impl UARTInterface {
    pub fn split(self) -> (Tx, Rx) {
        (
            Tx {
                tx_producer: self.tx_producer,
            },
            Rx {
                rx_consumer: self.rx_consumer,
            },
        )
    }
}

impl ErrorType for UARTInterface {
    type Error = Error;
}

impl ErrorType for Tx {
    type Error = Error;
}

impl ErrorType for Rx {
    type Error = Error;
}

impl Write<u8> for Tx {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        unsafe {
            modify_reg!(usart, USART2, CR1, TXFEIE: 1);
        }
        self.tx_producer
            .enqueue(word)
            .map_err(|_| nb::Error::WouldBlock)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // Wait blocking until all items from the queue are processed.
        while self.tx_producer.len() > 0 {}
        Ok(())
    }
}

impl core::fmt::Write for Tx {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let _ = s.bytes().map(|c| nb::block!(self.write(c))).last();
        Ok(())
    }
}

impl Read<u8> for Rx {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if let Some(byte) = self.rx_consumer.dequeue() {
            Ok(byte)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl UARTInterrupt {
    pub fn isr(&mut self) {
        // Check interrupt status
        // TX fifo empty
        if read_reg!(usart, self.uart, ISR, TXFE == 1) {
            // clear interrupt
            write_reg!(usart, self.uart, ICR, 1 << 5); // Missing field TXFECF is bit 5
        }

        // While fifo is not full, try to write new bytes.
        while read_reg!(usart, self.uart, ISR, TXE == 1) {
            // Missing TXFNF field (bit 7), use TXE
            // read queue and move data to FIFO
            if let Some(byte) = self.tx_consumer.dequeue() {
                // write byte to TDR register
                write_reg!(usart, self.uart, TDR, byte as u32);
            } else {
                // Turn off interrupt since we've sent everything
                modify_reg!(usart, self.uart, CR1, TXFEIE: 0);
                break;
            }
        }

        if read_reg!(usart, self.uart, ISR, RTOF == 1) {
            // Clear timeout
            write_reg!(usart, self.uart, ICR, RTOCF: 1);
        }

        if read_reg!(usart, self.uart, ISR, RXFT == 1) {
            // Flag is cleared by reading bytes from the FIFO
        }

        while read_reg!(usart, self.uart, ISR, RXNE == 1) {
            let byte = read_reg!(usart, self.uart, RDR) as u8;

            if let Err(_err) = self.rx_producer.enqueue(byte) {
                log::error!("Modem uart overrun");
                break;
            }
        }
    }
}

struct UARTAsync {}
