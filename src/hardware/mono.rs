//! Monotonic timer implementation for use with async rtic v2.
//!

use core::sync::atomic::{AtomicU32, Ordering};

use rtic_time::{Monotonic, TimerQueue};
use stm32ral::{modify_reg, read_reg, tim5, write_reg};

use super::rcc::Clocks;

const TIMER_HZ: u32 = 1_000_000;

// Global state for keeping track of time and scheduled events.
static TIMER_OVERFLOWS: AtomicU32 = AtomicU32::new(0);
static NEXT_COMPARE_LO: AtomicU32 = AtomicU32::new(0);
static NEXT_COMPARE_HI: AtomicU32 = AtomicU32::new(0);

static TIMER_QUEUE: TimerQueue<Mono> = TimerQueue::new();

/// Monotonic timer. This takes pseudo ownership of TIMER5.
/// Make sure to call `interrupt_handler()` from the TIMER5 interrupt handler.
/// This implementation assumes to run as the highest interrupt priority
/// to ensure integrity of 64 bits timestamps.
pub struct Mono;

impl Mono {
    pub fn start(tim5: tim5::Instance, clocks: &Clocks) {
        super::rcc::enable_rst_timer5();

        let clock_speed = clocks.pclk1();

        modify_reg!(tim5, tim5, CR1, CEN: Disabled);

        // Configure timer rate
        // PSC and ARR range: 0 to 65535
        // (PSC+1)*(ARR+1) = TIMclk/Updatefrequency = TIMclk * period
        let ratio = clock_speed.raw() / TIMER_HZ;
        let psc = ratio - 1;

        write_reg!(tim5, tim5, PSC, psc);

        // Enable update interrupt for keeping track of overflows.
        modify_reg!(tim5, tim5, DIER, UIE: 1, CC1IE: 1);

        // Set compare register to 0
        write_reg!(tim5, tim5, CCR1, 0u32);

        // Trigger update event to commit PSC
        write_reg!(tim5, tim5, EGR, UG: 1);

        // enable timer
        modify_reg!(tim5, tim5, CR1, CEN: Enabled);

        // Wait for interrupt triggered by UG
        while read_reg!(tim5, tim5, SR, UIF == 0) {}
        // Clear interrupt flag and initialize counter back to 0
        // because `set_match_value` depends on this via `duration_since_epoch`
        write_reg!(tim5, tim5, SR, UIF: 0);
        write_reg!(tim5, tim5, CNT, 0);

        TIMER_QUEUE.initialize(Mono {});

        unsafe {
            cortex_m::peripheral::NVIC::unmask(stm32ral::interrupt::TIM5);
        }

        log::info!("timer enabled");
    }

    /// Call this from the TIM5 interrupt handler
    pub unsafe fn interrupt_handler() {
        TIMER_QUEUE.on_monotonic_interrupt();
    }

    /// Delay for some duration of time.
    #[inline]
    pub async fn delay(duration: <Self as Monotonic>::Duration) {
        // Forward the TIMER_QUEUE API so that we can use it from rtic tasks.
        TIMER_QUEUE.delay(duration).await;
    }

    fn set_match_value(instant: <Mono as Monotonic>::Instant) {
        let now = Self::now();
        let max = u32::MAX as u64;

        let ticks = match instant.checked_duration_since(now) {
            Some(until_instant) if until_instant.ticks() <= max => {
                // Will overflow maximum once before
                // Reset next compare value
                NEXT_COMPARE_LO.store(0, Ordering::SeqCst);
                NEXT_COMPARE_HI.store(0, Ordering::SeqCst);

                // Return ticks
                instant.duration_since_epoch().ticks() & max
            }
            Some(_) => {
                let ticks = instant.duration_since_epoch().ticks() & max;
                let ticks_lo = (ticks & max) as u32;
                let ticks_hi = ((ticks >> 32) & max) as u32;
                // Will overflow, we have to save the instant to apply it later
                NEXT_COMPARE_LO.store(ticks_lo, Ordering::SeqCst);
                NEXT_COMPARE_HI.store(ticks_hi, Ordering::SeqCst);
                // Use 0 for current compare so we do not interrupt twice
                0
            }
            None => {
                // In the past
                0
            }
        };

        unsafe {
            write_reg!(tim5, TIM5, CCR1, ticks as u32);
        };
    }
}

impl Monotonic for Mono {
    type Instant = fugit::TimerInstantU64<TIMER_HZ>;
    type Duration = fugit::TimerDurationU64<TIMER_HZ>;

    const ZERO: Self::Instant = Self::Instant::from_ticks(0);

    fn now() -> Self::Instant {
        let mut overflows0 = TIMER_OVERFLOWS.load(Ordering::SeqCst) as u64;
        let now = loop {
            let count = unsafe { read_reg!(tim5, TIM5, CNT) };
            let overflows1 = TIMER_OVERFLOWS.load(Ordering::SeqCst) as u64;
            // Make sure we didn't have an additional overflow in between.
            if overflows0 == overflows1 {
                break (overflows1 << 32 | (count as u64));
            }
            overflows0 = overflows1;
        };

        Self::Instant::from_ticks(now)
    }

    fn set_compare(instant: Self::Instant) {
        Self::set_match_value(instant);
    }

    fn clear_compare_flag() {
        unsafe { modify_reg!(tim5, TIM5, SR, CC1IF: 0) }
    }

    fn pend_interrupt() {
        cortex_m::peripheral::NVIC::pend(stm32ral::interrupt::TIM5);
    }

    fn enable_timer() {
        unsafe { modify_reg!(tim5, TIM5, DIER, CC1IE: 1) }
    }

    fn disable_timer() {
        unsafe { modify_reg!(tim5, TIM5, DIER, CC1IE: 0) }
    }

    fn on_interrupt() {
        // Clear update flag if it was set
        unsafe {
            if read_reg!(tim5, TIM5, SR, UIF == 1) {
                modify_reg!(tim5, TIM5, SR, UIF: 0);
                TIMER_OVERFLOWS.fetch_add(1, Ordering::SeqCst);
                let compare = load_next_compare();
                let instant = Self::Instant::from_ticks(compare);
                Self::set_match_value(instant);
            }
        }
    }
}

/// Safely read 64 bits next_compare value
fn load_next_compare() -> u64 {
    let mut compare_hi0 = NEXT_COMPARE_HI.load(Ordering::SeqCst);
    loop {
        let compare_lo = NEXT_COMPARE_LO.load(Ordering::SeqCst);
        let compare_hi1 = NEXT_COMPARE_HI.load(Ordering::SeqCst);
        if compare_hi0 == compare_hi1 {
            return (compare_hi1 as u64) << 32 | (compare_lo as u64);
        }
        compare_hi0 = compare_hi1;
    }
}
