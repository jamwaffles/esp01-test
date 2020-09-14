#![no_std]
#![no_main]

mod commands;

use atat::{prelude::*, ClientBuilder, ComQueue, Queues, ResQueue, UrcQueue};
use core::fmt::Write;
use core::panic::PanicInfo;
use embedded_hal::digital::v2::OutputPin;
use heapless::{consts, spsc::Queue};
use panic_probe as _;
use rtic::app;
use rtic::cyccnt::{Instant, U32Ext};
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

#[app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        // timer: Timer<TIM2>,
        status: StatusLED,
        // tx: serial::Tx<USART1>,
        rx: serial::Rx<USART1>,
        ingress: atat::IngressManager<consts::U256, atat::NoopUrcMatcher>,
        client: atat::Client<serial::Tx<USART1>, Timer<TIM2>>,
    }

    #[init(spawn = [at_loop], schedule = [initialise])]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut RES_QUEUE: ResQueue<consts::U256, consts::U5> = Queue(heapless::i::Queue::u8());
        static mut URC_QUEUE: UrcQueue<consts::U256, consts::U10> = Queue(heapless::i::Queue::u8());
        static mut COM_QUEUE: ComQueue<consts::U3> = Queue(heapless::i::Queue::u8());

        rtt_init_print!();

        let dp = ctx.device;
        let mut core = ctx.core;

        core.DWT.enable_cycle_counter();

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

        // timer.listen(Event::TimeOut);

        let queues = Queues {
            res_queue: RES_QUEUE.split(),
            urc_queue: URC_QUEUE.split(),
            com_queue: COM_QUEUE.split(),
        };

        let (mut client, ingress) =
            ClientBuilder::new(tx, timer, atat::Config::new(atat::Mode::Timeout)).build(queues);

        status.set_low().unwrap();

        // ctx.spawn.at_loop().unwrap();
        ctx.spawn.at_loop().unwrap();
        ctx.schedule
            .initialise(ctx.start + 100_000_000.cycles())
            .unwrap();

        rprintln!("Init complete");

        // Init the static resources to use them later through RTIC
        init::LateResources {
            // timer,
            status,
            // tx,
            rx,
            client,
            ingress,
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

    // #[task(binds = TIM2, resources = [timer, status, tx])]
    // fn update(ctx: update::Context) {
    //     let update::Resources {
    //         timer, status, tx, ..
    //     } = ctx.resources;

    //     rprintln!("Update");

    //     // tx.write_str("AT\r\n").unwrap();

    //     // status.toggle().unwrap();

    //     timer.clear_interrupt(Event::TimeOut);
    // }

    #[task(resources = [client])]
    fn initialise(mut ctx: initialise::Context) {
        rprintln!("Time to init");
        let initialise::Resources { mut client, .. } = ctx.resources;

        let response = client.send(&commands::At);
        rprintln!("{:?}", response);
    }

    // #[task(spawn = [at_loop], resources = [ingress])]
    #[task(schedule = [at_loop], resources = [ingress], priority = 2)]
    fn at_loop(mut ctx: at_loop::Context) {
        // rprintln!("Digest");
        let at_loop::Resources { mut ingress, .. } = ctx.resources;

        ingress.lock(|at| at.digest());

        // ctx.spawn.at_loop().unwrap();

        ctx.schedule
            .at_loop(ctx.scheduled + 10_000.cycles())
            .unwrap();
    }

    #[task(binds = USART1, priority = 3, resources = [status, rx, ingress])]
    fn serial_recv(mut ctx: serial_recv::Context) {
        let serial_recv::Resources {
            status,
            rx,
            ingress,
            ..
        } = ctx.resources;

        if let Ok(d) = nb::block!(rx.read()) {
            rprintln!("Received {:?}", d);

            ingress.write(&[d]);

            status.toggle().unwrap();
        }
    }

    // Extra interrupts for software tasks
    extern "C" {
        fn EXTI0();
        fn USART2();
    }
};
