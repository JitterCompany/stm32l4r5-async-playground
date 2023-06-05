#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]

mod hardware;
mod rtt_logger;

use panic_rtt_target as _;
use rtic::app;
use embedded_hal_async::serial::{Write};
use fugit::ExtU64;

use hardware::{mono::Mono, uartasync};
#[app(device=hardware::rtic_device, peripherals = true, dispatchers = [SPI1, SPI2])]
mod app {

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        red_led: hardware::Pin,
        green_led: hardware::Pin,
        blue_led: hardware::Pin,
        uart: hardware::UART
    }

    const SYSCLK_HZ: u32 = 80_000_000;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        rtt_logger::init();

        log::info!("Hello");

        let _cortex = cx.core;

        let hw = hardware::board_init(cx.device);

        log::info!("Board Init Done");

        let clocks = hw.clocks;
        if clocks.sysclk().raw() != SYSCLK_HZ {
            panic!("Unexpected clock rate");
        }

        blink::spawn().ok();

        (
            Shared {},
            Local {
                uart: hw.uart,
                blue_led: hw.blue_led,
                green_led: hw.green_led,
                red_led: hw.red_led,
            },
        )
    }

    #[idle(local=[green_led], shared=[])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            core::hint::spin_loop(); // Todo: Do we need this?
        }
    }

    #[task(local = [blue_led, uart], priority=1)]
    async fn blink(cx: blink::Context) {
        let blueled = cx.local.blue_led;
        let uart = cx.local.uart;

        let mut num = 0usize;

        loop {
            uart.write("BLINK\n".as_bytes()).await.unwrap();
            log::info!("BLINK {}", num);
            num+=1;
            Mono::delay(2u64.millis()).await;
            blueled.toggle();
        }
    }

    #[task(binds=USART2, priority=3, local = [red_led])]
    fn uart_isr(cx: uart_isr::Context) {
        log::info!("uart isr");

        unsafe {
            uartasync::UART::isr();
        }
        cx.local.red_led.toggle();

    }

    #[task(binds=TIM5, priority=7)]
    fn tim5(_cx: tim5::Context) {
        unsafe {
            Mono::interrupt_handler();
        }
    }
}
