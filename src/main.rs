#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod hardware;
mod rtt_logger;

use panic_rtt_target as _;
use rtic::app;

use hardware::mono::Mono;

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
                blue_led: hw.blue_led,
                green_led: hw.green_led,
                red_led: hw.red_led,
            },
        )
    }

    #[idle(local=[green_led, red_led], shared=[])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            core::hint::spin_loop(); // Todo: Do we need this?
        }
    }

    #[task(local = [blue_led], priority=1)]
    async fn blink(cx: blink::Context) {
        cx.local.blue_led.toggle();
        log::info!("BLINK");
    }

    // #[task(binds=USART2, priority=3, local = [uart_interrupt, red_led])]
    // fn uart_isr(cx: uart_isr::Context) {
    //     cx.local.uart_interrupt.isr();
    //     cx.local.red_led.toggle();
    // }

    #[task(binds=TIM5, priority=7)]
    fn tim5(_cx: tim5::Context) {
        unsafe {
            Mono::interrupt_handler();
        }
    }
}
