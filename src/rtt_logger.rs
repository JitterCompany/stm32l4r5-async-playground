use core::fmt::Write;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use log::{LevelFilter, Metadata, Record};
use rtt_target::{rprintln, rtt_init, set_print_channel, UpChannel};

pub struct Logger {
    pub trace_channel: Mutex<RefCell<Option<UpChannel>>>,
}

static LOGGER: Logger = Logger {
    trace_channel: Mutex::new(RefCell::new(None)),
};

pub fn init() {
    let channels = rtt_init! {
        up: {
            0: {
                size: 1024
                mode: NoBlockSkip
                name: "Print Output"
            }
            1: {
                size: 128
                mode: NoBlockSkip
                name: "Trace output"
            }
        }
        down: {
            0: {
                size: 80
                mode: NoBlockSkip
                name: "Command input"
            }
        }
    };

    let output = channels.up.0;
    let trace_channel = channels.up.1;

    cortex_m::interrupt::free(|cs| {
        LOGGER.trace_channel.borrow(cs).replace(Some(trace_channel));
    });

    set_print_channel(output);

    log::set_logger(&LOGGER)
        .map(|()| log::set_max_level(LevelFilter::Trace))
        .unwrap();
}

impl log::Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        match record.level() {
            log::Level::Trace => {
                cortex_m::interrupt::free(|cs| {
                    if let Some(tx) = self.trace_channel.borrow(cs).borrow_mut().as_mut() {
                        writeln!(tx, "{}", record.args()).ok();
                    }
                });
            }
            _ => rprintln!("{} - {}", record.level(), record.args()),
        }
    }

    fn flush(&self) {}
}
