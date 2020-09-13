#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rtt_target::{rprintln, rtt_init_print};

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();

    loop {
        rprintln!("Hello, world!");

        panic!("I panicked");
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
