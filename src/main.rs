#![no_std]
#![no_main]

use core::fmt::Write;
use core::panic::PanicInfo;
use embedded_hal::digital::v2::OutputPin;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::stm32::{I2C1, SPI1, SPI2};
use stm32f4xx_hal::{
    delay::Delay,
    gpio::{self, gpioa, gpiob, Alternate, AF7},
    i2c::{self, I2c},
    prelude::*,
    serial::{self, config::Config, Serial},
    spi::{self, Mode, Phase, Polarity, Spi},
    stm32::{self, TIM2, USART1},
    time::Hertz,
    timer::{Event, Timer},
};

type StatusLED = gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>;

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        timer: Timer<TIM2>,
        status: StatusLED,
        tx: serial::Tx<USART1>,
        rx: serial::Rx<USART1>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();

        let dp = cx.device;
        // let core = cx.core;

        let rcc = dp.RCC.constrain();

        let clocks = rcc.cfgr.use_hse(25u32.mhz()).sysclk(100u32.mhz()).freeze();

        let mut gpiob = dp.GPIOB.split();
        // let mut gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        // let mut delay = Delay::new(core.SYST, clocks);

        let mut status = gpioc.pc13.into_push_pull_output();

        let serial = {
            // USART1
            let tx = gpiob.pb6.into_alternate_af7();
            let rx = gpiob.pb7.into_alternate_af7();

            let mut serial = Serial::usart1(
                dp.USART1,
                (tx, rx),
                Config::default().baudrate(115200.bps()),
                clocks,
            )
            .unwrap();

            // Listen for returned chars
            serial.listen(serial::Event::Rxne);

            serial
        };

        let (tx, rx) = serial.split();

        let mut timer = Timer::tim2(dp.TIM2, 1.hz(), clocks);

        timer.listen(Event::TimeOut);

        status.set_low().unwrap();

        rprintln!("Init complete");

        // Init the static resources to use them later through RTIC
        init::LateResources {
            timer,
            status,
            tx,
            rx,
        }
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

    #[task(binds = TIM2, resources = [timer, status, tx])]
    fn update(cx: update::Context) {
        let update::Resources {
            timer, status, tx, ..
        } = cx.resources;

        rprintln!("Update");

        tx.write_str("AT\r\n").unwrap();

        // status.toggle().unwrap();

        timer.clear_interrupt(Event::TimeOut);
    }

    #[task(binds = USART1, resources = [status, rx])]
    fn serial_recv(cx: serial_recv::Context) {
        let serial_recv::Resources { status, rx, .. } = cx.resources;

        let byte = rx.read().unwrap();

        rprintln!("Received {}", byte);

        status.toggle().unwrap();
    }
};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
