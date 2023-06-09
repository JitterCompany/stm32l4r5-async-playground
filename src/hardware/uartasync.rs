use core::future::poll_fn;
use core::task::Poll;
use core::{num, ops::Deref};

use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal_async::serial::{Error as HALErrorTrait, ErrorKind, ErrorType, Write};
use embedded_io::{
    asynch::{Read as AsyncRead, Write as AsyncWrite},
    Io,
};
use heapless::spsc::{Consumer, Queue};
use stm32ral::{modify_reg, read_reg, reset_reg, usart, write_reg};

use super::{
    gpio::Pin,
    rcc::{self, Clocks},
};

pub struct UART {
    uart: usart::Instance,
    _tx: Pin,
    _rx: Pin,
    rx_buffer_consumer: Consumer<'static, u8, 16>,
}

// static RX_BUFFER: heapless::spsc::Queue<u8, 32> = heapless::spsc::Queue::new();
static mut RX_BUFFER: Queue<u8, 16> = Queue::new();

const INIT: AtomicWaker = AtomicWaker::new();
static TX_WAKER: AtomicWaker = AtomicWaker::new();
static RX_WAKER: AtomicWaker = AtomicWaker::new();

const UART_FIFO_SIZE: usize = 8;

impl UART {
    pub fn new(uart: usart::Instance, tx: Pin, rx: Pin, clocks: &Clocks) -> Self {
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
            TCIE: Enabled,
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

        let (_p, rx_buffer_consumer) = unsafe { RX_BUFFER.split() };

        UART {
            uart,
            _tx: tx,
            _rx: rx,
            rx_buffer_consumer,
        }
    }

    fn write_byte(&mut self, byte: u8) {
        write_reg!(usart, self.uart, TDR, byte as u32);
    }

    fn fifo_is_empty(&self) -> bool {
        read_reg!(usart, self.uart, ISR, TXFE == 1)
    }

    async fn write(&mut self, words: &[u8]) -> Result<usize, Error> {
        for chunk in words.chunks(UART_FIFO_SIZE as usize) {
            for &byte in chunk {
                self.write_byte(byte)
            }
            UartFuture::new(Event::TxFiFoEmpty).await;
        }
        Ok(words.len())
    }

    async fn flush(&mut self) -> Result<(), Error> {
        if !self.fifo_is_empty() {
            UartFuture::new(Event::TxDone).await;
        }
        Ok(())
    }

    pub unsafe fn isr() {
        // Check interrupt status

        // TX fifo empty and TX Complete
        if read_reg!(usart, USART2, ISR, TXFE == 1) || read_reg!(usart, USART2, ISR, TC == 1) {
            // Clear interrupts

            write_reg!(usart, USART2, ICR, 1 << 5); // Missing field TXFECF is bit 5

            write_reg!(usart, USART2, ICR, TCCF: 1);

            TX_WAKER.wake();
        }

        if read_reg!(usart, USART2, ISR, RTOF == 1) || read_reg!(usart, USART2, ISR, RXFT == 1) {
            // Clear timeout
            write_reg!(usart, USART2, ICR, RTOCF: 1);

            // RXFT is cleared by reading bytes from the FIFO
            while read_reg!(usart, USART2, ISR, RXNE == 1) {
                let byte = read_reg!(usart, USART2, RDR) as u8;

                if let Err(_err) = RX_BUFFER.enqueue(byte) {
                    log::error!("Modem uart overrun");
                    break;
                }
            }

            RX_WAKER.wake();
        }
    }

    /// Read at least 1 and maximum `buf.len()` bytes
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        poll_fn(|cx| {
            let num_bytes = self.rx_buffer_consumer.len().min(buf.len());

            if num_bytes > 0 {
                let mut index = 0usize;
                while let Some(byte) = self.rx_buffer_consumer.dequeue() {
                    buf[index] = byte;
                    index += 1;
                    if index >= buf.len() {
                        break;
                    }
                }
                Poll::Ready(Ok(index))
            } else {
                RX_WAKER.register(cx.waker());
                Poll::Pending
            }
        })
        .await
    }
}

impl Io for UART {
    type Error = self::Error;
}

impl AsyncWrite for UART {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush().await
    }
}

impl AsyncRead for UART {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read(buf).await
    }
}

pub(crate) enum Event {
    TxDone,
    TxFiFoEmpty,
}

pub(crate) struct UartFuture {
    event: Event,
}

impl UartFuture {
    pub fn new(event: Event) -> Self {
        match event {
            Event::TxDone => {
                // enable tx done interrupt here?
            }
            Event::TxFiFoEmpty => {
                // Enable fixo empty interrupt here?
            }
        }

        Self { event }
    }

    fn event_bit_is_clear(&self) -> bool {
        match self.event {
            Event::TxDone => unsafe { read_reg!(usart, USART2, ISR, TC == 0) },
            Event::TxFiFoEmpty => unsafe { read_reg!(usart, USART2, ISR, TXFE == 0) },
        }
    }
}

impl core::future::Future for UartFuture {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        TX_WAKER.register(cx.waker());
        if self.event_bit_is_clear() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

#[derive(Debug)]
pub enum Error {
    SomeError,
}

impl HALErrorTrait for Error {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for UART {
    type Error = Error;
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}
