use super::rcc::{self, Clocks};
use cortex_m::interrupt;
use stm32ral::{modify_reg, read_reg, tim2, write_reg};

pub struct Timer<const TIMER_HZ: u32> {
    timer: tim2::Instance,
    end_time: Option<fugit::TimerInstantU32<TIMER_HZ>>,
}

impl<const TIMER_HZ: u32> Timer<TIMER_HZ> {
    pub fn new(timer: tim2::Instance, clocks: &Clocks) -> Self {
        rcc::enable_rst_timer2();

        let clock_speed = clocks.pclk1();

        // Configure timer rate
        // PSC and ARR range: 0 to 65535
        // (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period
        let ratio = clock_speed.raw() / TIMER_HZ;
        let psc = ratio - 1;

        // Panic if there is a remainder or clock division to large
        if ((clock_speed.raw() % TIMER_HZ) != 0) || psc >= (1 << 16) {
            panic!(
                "Tim2: Cannot make required rate ({TIMER_HZ:?}) from clock {:?}",
                clock_speed.raw()
            );
        }

        write_reg!(tim2, timer, PSC, psc);

        write_reg!(tim2, timer, CNT, 0);

        // enable timer
        modify_reg!(tim2, timer, CR1, CEN: Enabled);

        Self {
            timer,
            end_time: None,
        }
    }

    pub fn ticks(&self) -> u64 {
        static mut OVERFLOWS: u32 = 0;
        static mut OLD_CNT: u32 = 0;

        interrupt::free(|_| {
            // Safety: These static mut variables are accessed in an interrupt free section.
            let (overflows, last_count) = unsafe { (&mut OVERFLOWS, &mut OLD_CNT) };

            let count = read_reg!(tim2, self.timer, CNT);

            if count <= *last_count {
                *overflows += 1;
            }

            let ticks = (*overflows as u64) << 32 | (count as u64);
            *last_count = count;

            ticks
        })
    }
}

impl<const TIMER_HZ: u32> fugit_timer::Timer<TIMER_HZ> for Timer<TIMER_HZ> {
    type Error = core::convert::Infallible;
    fn now(&mut self) -> fugit::TimerInstantU32<TIMER_HZ> {
        fugit::TimerInstantU32::from_ticks(self.ticks() as u32)
    }

    fn start(&mut self, duration: fugit::TimerDurationU32<TIMER_HZ>) -> Result<(), Self::Error> {
        let now = self.now();
        let end = now + duration;
        self.end_time.replace(end);
        Ok(())
    }

    fn cancel(&mut self) -> Result<(), Self::Error> {
        self.end_time.take();
        Ok(())
    }

    fn wait(&mut self) -> nb::Result<(), Self::Error> {
        let now = self.now();
        match self.end_time {
            Some(end) if end <= now => Ok(()),
            _ => Err(nb::Error::WouldBlock),
        }
    }
}
