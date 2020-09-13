#![no_std]
#![no_main]

use core::panic::PanicInfo;
use embedded_hal::digital::v2::OutputPin;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::stm32::{I2C1, SPI1, SPI2};
use stm32f4xx_hal::{
    delay::Delay,
    gpio,
    i2c::{self, I2c},
    prelude::*,
    spi::{self, Mode, Phase, Polarity, Spi},
    stm32::{self, TIM2},
    time::Hertz,
    timer::{Event, Timer},
};

type StatusLED = gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>;

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        timer: Timer<TIM2>,
        status: StatusLED,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();

        let dp = cx.device;
        // let core = cx.core;

        let rcc = dp.RCC.constrain();

        let clocks = rcc.cfgr.use_hse(25u32.mhz()).sysclk(100u32.mhz()).freeze();

        // let mut gpiob = dp.GPIOB.split();
        // let mut gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        // let mut delay = Delay::new(core.SYST, clocks);

        let mut status = gpioc.pc13.into_push_pull_output();

        let mut timer = Timer::tim2(dp.TIM2, 5.hz(), clocks);

        timer.listen(Event::TimeOut);

        status.set_low().unwrap();

        rprintln!("Init complete");

        // Init the static resources to use them later through RTIC
        init::LateResources { timer, status }
    }

    // FIXME: RTT doesn't work in sleep, and default idle() uses WFI which puts device to sleep.
    // Once fixed, this empty idle can be removed for efficiency. Tracked here
    // https://github.com/probe-rs/probe-rs/issues/300
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            core::sync::atomic::spin_loop_hint();
        }
    }

    #[task(binds = TIM2, resources = [timer, status])]
    fn update(cx: update::Context) {
        let update::Resources { timer, status, .. } = cx.resources;

        rprintln!("Update");

        status.toggle().unwrap();

        timer.clear_interrupt(Event::TimeOut);
    }

    extern "C" {
        fn EXTI0();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
