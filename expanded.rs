#![feature(prelude_import)]
#![no_std]
#![no_main]
#[prelude_import]
use core::prelude::v1::*;
#[macro_use]
extern crate core;
#[macro_use]
extern crate compiler_builtins;
mod commands {
    use atat::{
        atat_derive::{AtatCmd, AtatResp, AtatUrc},
        AtatCmd, AtatResp, AtatUrc,
    };
    use heapless::String;
    pub struct EmptyResponse;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for EmptyResponse {
        #[inline]
        fn clone(&self) -> EmptyResponse {
            match *self {
                EmptyResponse => EmptyResponse,
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for EmptyResponse {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                EmptyResponse => {
                    let mut debug_trait_builder = f.debug_tuple("EmptyResponse");
                    debug_trait_builder.finish()
                }
            }
        }
    }
    #[automatically_derived]
    impl atat::AtatResp for EmptyResponse {}
    #[automatically_derived]
    impl<'de> serde::Deserialize<'de> for EmptyResponse {
        #[inline]
        fn deserialize<D>(deserializer: D) -> serde::export::Result<Self, D::Error>
        where
            D: serde::Deserializer<'de>,
        {
            #[allow(non_camel_case_types)]
            enum EmptyResponseField {
                ignore,
            }
            struct EmptyResponseFieldVisitor;
            impl<'de> serde::de::Visitor<'de> for EmptyResponseFieldVisitor {
                type Value = EmptyResponseField;
                fn expecting(
                    &self,
                    formatter: &mut serde::export::Formatter,
                ) -> serde::export::fmt::Result {
                    serde::export::Formatter::write_str(formatter, "field identifier")
                }
                fn visit_u64<E>(self, value: u64) -> serde::export::Result<Self::Value, E>
                where
                    E: serde::de::Error,
                {
                    match value {
                        _ => serde::export::Err(::serde::de::Error::invalid_value(
                            serde::de::Unexpected::Unsigned(value),
                            &"field index 0 <= i < 0",
                        )),
                    }
                }
                fn visit_u128<E>(self, value: u128) -> serde::export::Result<Self::Value, E>
                where
                    E: serde::de::Error,
                {
                    match value {
                        _ => serde::export::Err(::serde::de::Error::invalid_value(
                            serde::de::Unexpected::Other("u128"),
                            &"field index 0 <= i < 0",
                        )),
                    }
                }
                fn visit_str<E>(self, value: &str) -> serde::export::Result<Self::Value, E>
                where
                    E: serde::de::Error,
                {
                    match value {
                        _ => serde::export::Ok(EmptyResponseField::ignore),
                    }
                }
                fn visit_bytes<E>(self, value: &[u8]) -> serde::export::Result<Self::Value, E>
                where
                    E: serde::de::Error,
                {
                    match value {
                        _ => serde::export::Ok(EmptyResponseField::ignore),
                    }
                }
            }
            impl<'de> serde::Deserialize<'de> for EmptyResponseField {
                #[inline]
                fn deserialize<D>(deserializer: D) -> serde::export::Result<Self, D::Error>
                where
                    D: serde::Deserializer<'de>,
                {
                    serde::Deserializer::deserialize_identifier(
                        deserializer,
                        EmptyResponseFieldVisitor,
                    )
                }
            }
            struct EmptyResponseVisitor<'de> {
                marker: serde::export::PhantomData<EmptyResponse>,
                lifetime: serde::export::PhantomData<&'de ()>,
            }
            impl<'de> serde::de::Visitor<'de> for EmptyResponseVisitor<'de> {
                type Value = EmptyResponse;
                fn expecting(
                    &self,
                    formatter: &mut serde::export::Formatter,
                ) -> serde::export::fmt::Result {
                    serde::export::Formatter::write_str(formatter, "struct EmptyResponse")
                }
                #[inline]
                fn visit_seq<A>(self, mut seq: A) -> serde::export::Result<Self::Value, A::Error>
                where
                    A: serde::de::SeqAccess<'de>,
                {
                    serde::export::Ok(EmptyResponse {})
                }
                #[inline]
                fn visit_map<A>(self, mut map: A) -> serde::export::Result<Self::Value, A::Error>
                where
                    A: serde::de::MapAccess<'de>,
                {
                    while let serde::export::Some(key) =
                        match serde::de::MapAccess::next_key::<EmptyResponseField>(&mut map) {
                            serde::export::Ok(val) => val,
                            serde::export::Err(err) => {
                                return serde::export::Err(err);
                            }
                        }
                    {
                        match key {
                            _ => {
                                let _ = match serde::de::MapAccess::next_value::<
                                    serde::de::IgnoredAny,
                                >(&mut map)
                                {
                                    serde::export::Ok(val) => val,
                                    serde::export::Err(err) => {
                                        return serde::export::Err(err);
                                    }
                                };
                            }
                        }
                    }
                    serde::export::Ok(EmptyResponse {})
                }
            }
            const FIELDS: &'static [&'static str] = &[];
            serde::Deserializer::deserialize_struct(
                deserializer,
                "EmptyResponse",
                FIELDS,
                EmptyResponseVisitor {
                    marker: serde::export::PhantomData::<EmptyResponse>,
                    lifetime: serde::export::PhantomData,
                },
            )
        }
    }
    #[at_cmd("", EmptyResponse, timeout_ms = 1000)]
    pub struct At;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for At {
        #[inline]
        fn clone(&self) -> At {
            match *self {
                At => At,
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for At {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                At => {
                    let mut debug_trait_builder = f.debug_tuple("At");
                    debug_trait_builder.finish()
                }
            }
        }
    }
    #[automatically_derived]
    impl atat::AtatLen for At {
        type Len = ::heapless::consts::U0;
    }
    #[automatically_derived]
    impl atat::AtatCmd for At {
        type Response = EmptyResponse;
        type CommandLen =
            <<Self as atat::AtatLen>::Len as core::ops::Add<::heapless::consts::U5>>::Output;
        #[inline]
        fn as_bytes(&self) -> ::heapless::Vec<u8, Self::CommandLen> {
            let s: ::heapless::String<::heapless::consts::U0> = ::heapless::String::from("");
            match serde_at::to_vec(
                self,
                s,
                serde_at::SerializeOptions {
                    value_sep: true,
                    cmd_prefix: "AT",
                    termination: "\r\n",
                },
            ) {
                Ok(s) => s,
                Err(_) => ::core::panicking::panic("Failed to serialize command"),
            }
        }
        #[inline]
        fn parse(&self, resp: &[u8]) -> core::result::Result<EmptyResponse, atat::Error> {
            serde_at::from_slice::<EmptyResponse>(resp).map_err(|e| atat::Error::ParseString)
        }
        fn max_timeout_ms(&self) -> u32 {
            1000u32
        }
    }
    #[automatically_derived]
    impl serde::Serialize for At {
        #[inline]
        fn serialize<S>(&self, serializer: S) -> serde::export::Result<S::Ok, S::Error>
        where
            S: serde::Serializer,
        {
            let mut serde_state =
                match serde::Serializer::serialize_struct(serializer, "At", 0usize) {
                    serde::export::Ok(val) => val,
                    serde::export::Err(err) => {
                        return serde::export::Err(err);
                    }
                };
            serde::ser::SerializeStruct::end(serde_state)
        }
    }
    #[at_cmd("+RST", EmptyResponse, timeout_ms = 1000)]
    pub struct Reset;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Reset {
        #[inline]
        fn clone(&self) -> Reset {
            match *self {
                Reset => Reset,
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Reset {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                Reset => {
                    let mut debug_trait_builder = f.debug_tuple("Reset");
                    debug_trait_builder.finish()
                }
            }
        }
    }
    #[automatically_derived]
    impl atat::AtatLen for Reset {
        type Len = ::heapless::consts::U0;
    }
    #[automatically_derived]
    impl atat::AtatCmd for Reset {
        type Response = EmptyResponse;
        type CommandLen =
            <<Self as atat::AtatLen>::Len as core::ops::Add<::heapless::consts::U9>>::Output;
        #[inline]
        fn as_bytes(&self) -> ::heapless::Vec<u8, Self::CommandLen> {
            let s: ::heapless::String<::heapless::consts::U4> = ::heapless::String::from("+RST");
            match serde_at::to_vec(
                self,
                s,
                serde_at::SerializeOptions {
                    value_sep: true,
                    cmd_prefix: "AT",
                    termination: "\r\n",
                },
            ) {
                Ok(s) => s,
                Err(_) => ::core::panicking::panic("Failed to serialize command"),
            }
        }
        #[inline]
        fn parse(&self, resp: &[u8]) -> core::result::Result<EmptyResponse, atat::Error> {
            serde_at::from_slice::<EmptyResponse>(resp).map_err(|e| atat::Error::ParseString)
        }
        fn max_timeout_ms(&self) -> u32 {
            1000u32
        }
    }
    #[automatically_derived]
    impl serde::Serialize for Reset {
        #[inline]
        fn serialize<S>(&self, serializer: S) -> serde::export::Result<S::Ok, S::Error>
        where
            S: serde::Serializer,
        {
            let mut serde_state =
                match serde::Serializer::serialize_struct(serializer, "Reset", 0usize) {
                    serde::export::Ok(val) => val,
                    serde::export::Err(err) => {
                        return serde::export::Err(err);
                    }
                };
            serde::ser::SerializeStruct::end(serde_state)
        }
    }
    #[at_cmd("E0", EmptyResponse, timeout_ms = 1000)]
    pub struct DisableEcho;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for DisableEcho {
        #[inline]
        fn clone(&self) -> DisableEcho {
            match *self {
                DisableEcho => DisableEcho,
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for DisableEcho {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                DisableEcho => {
                    let mut debug_trait_builder = f.debug_tuple("DisableEcho");
                    debug_trait_builder.finish()
                }
            }
        }
    }
    #[automatically_derived]
    impl atat::AtatLen for DisableEcho {
        type Len = ::heapless::consts::U0;
    }
    #[automatically_derived]
    impl atat::AtatCmd for DisableEcho {
        type Response = EmptyResponse;
        type CommandLen =
            <<Self as atat::AtatLen>::Len as core::ops::Add<::heapless::consts::U7>>::Output;
        #[inline]
        fn as_bytes(&self) -> ::heapless::Vec<u8, Self::CommandLen> {
            let s: ::heapless::String<::heapless::consts::U2> = ::heapless::String::from("E0");
            match serde_at::to_vec(
                self,
                s,
                serde_at::SerializeOptions {
                    value_sep: true,
                    cmd_prefix: "AT",
                    termination: "\r\n",
                },
            ) {
                Ok(s) => s,
                Err(_) => ::core::panicking::panic("Failed to serialize command"),
            }
        }
        #[inline]
        fn parse(&self, resp: &[u8]) -> core::result::Result<EmptyResponse, atat::Error> {
            serde_at::from_slice::<EmptyResponse>(resp).map_err(|e| atat::Error::ParseString)
        }
        fn max_timeout_ms(&self) -> u32 {
            1000u32
        }
    }
    #[automatically_derived]
    impl serde::Serialize for DisableEcho {
        #[inline]
        fn serialize<S>(&self, serializer: S) -> serde::export::Result<S::Ok, S::Error>
        where
            S: serde::Serializer,
        {
            let mut serde_state =
                match serde::Serializer::serialize_struct(serializer, "DisableEcho", 0usize) {
                    serde::export::Ok(val) => val,
                    serde::export::Err(err) => {
                        return serde::export::Err(err);
                    }
                };
            serde::ser::SerializeStruct::end(serde_state)
        }
    }
    #[at_cmd("E1", EmptyResponse, timeout_ms = 1000)]
    pub struct EnableEcho;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for EnableEcho {
        #[inline]
        fn clone(&self) -> EnableEcho {
            match *self {
                EnableEcho => EnableEcho,
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for EnableEcho {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                EnableEcho => {
                    let mut debug_trait_builder = f.debug_tuple("EnableEcho");
                    debug_trait_builder.finish()
                }
            }
        }
    }
    #[automatically_derived]
    impl atat::AtatLen for EnableEcho {
        type Len = ::heapless::consts::U0;
    }
    #[automatically_derived]
    impl atat::AtatCmd for EnableEcho {
        type Response = EmptyResponse;
        type CommandLen =
            <<Self as atat::AtatLen>::Len as core::ops::Add<::heapless::consts::U7>>::Output;
        #[inline]
        fn as_bytes(&self) -> ::heapless::Vec<u8, Self::CommandLen> {
            let s: ::heapless::String<::heapless::consts::U2> = ::heapless::String::from("E1");
            match serde_at::to_vec(
                self,
                s,
                serde_at::SerializeOptions {
                    value_sep: true,
                    cmd_prefix: "AT",
                    termination: "\r\n",
                },
            ) {
                Ok(s) => s,
                Err(_) => ::core::panicking::panic("Failed to serialize command"),
            }
        }
        #[inline]
        fn parse(&self, resp: &[u8]) -> core::result::Result<EmptyResponse, atat::Error> {
            serde_at::from_slice::<EmptyResponse>(resp).map_err(|e| atat::Error::ParseString)
        }
        fn max_timeout_ms(&self) -> u32 {
            1000u32
        }
    }
    #[automatically_derived]
    impl serde::Serialize for EnableEcho {
        #[inline]
        fn serialize<S>(&self, serializer: S) -> serde::export::Result<S::Ok, S::Error>
        where
            S: serde::Serializer,
        {
            let mut serde_state =
                match serde::Serializer::serialize_struct(serializer, "EnableEcho", 0usize) {
                    serde::export::Ok(val) => val,
                    serde::export::Err(err) => {
                        return serde::export::Err(err);
                    }
                };
            serde::ser::SerializeStruct::end(serde_state)
        }
    }
}
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
#[allow(non_snake_case)]
fn init(
    init::Locals {
        RES_QUEUE,
        URC_QUEUE,
        COM_QUEUE,
        ..
    }: init::Locals,
    ctx: init::Context,
) -> init::LateResources {
    let channels = {
        use core::mem::MaybeUninit;
        use core::ptr;
        use ::rtt_target::UpChannel;
        use ::rtt_target::DownChannel;
        use ::rtt_target::rtt::*;
        #[repr(C)]
        pub struct RttControlBlock {
            header: RttHeader,
            up_channels: [RttChannel; (1 + 0)],
            down_channels: [RttChannel; (0)],
        }
        #[used]
        #[no_mangle]
        #[export_name = "_SEGGER_RTT"]
        pub static mut CONTROL_BLOCK: MaybeUninit<RttControlBlock> = MaybeUninit::uninit();
        unsafe {
            ptr::write_bytes(CONTROL_BLOCK.as_mut_ptr(), 0, 1);
            let cb = &mut *CONTROL_BLOCK.as_mut_ptr();
            let mut name: *const u8 = core::ptr::null();
            name = "Terminal\u{0}".as_bytes().as_ptr();
            let mut mode = ::rtt_target::ChannelMode::NoBlockSkip;
            mode = ::rtt_target::ChannelMode::NoBlockSkip;
            cb.up_channels[0].init(name, mode, {
                static mut _RTT_CHANNEL_BUFFER: MaybeUninit<[u8; 1024]> = MaybeUninit::uninit();
                _RTT_CHANNEL_BUFFER.as_mut_ptr()
            });
            cb.header.init(cb.up_channels.len(), cb.down_channels.len());
            pub struct Channels {
                up: (UpChannel,),
            }
            Channels {
                up: (UpChannel::new(&mut cb.up_channels[0] as *mut _),),
            }
        }
    };
    ::rtt_target::set_print_channel(channels.up.0);
    let dp = ctx.device;
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25u32.mhz()).sysclk(100u32.mhz()).freeze();
    let mut gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let mut status = gpioc.pc13.into_push_pull_output();
    let serial = {
        let tx = gpiob.pb6.into_alternate_af7();
        let rx = gpiob.pb7.into_alternate_af7();
        let mut serial = Serial::usart1(
            dp.USART1,
            (tx, rx),
            Config::default().baudrate(115200.bps()),
            clocks,
        )
        .unwrap();
        serial.listen(serial::Event::Rxne);
        serial
    };
    let (tx, rx) = serial.split();
    let mut timer = Timer::tim2(dp.TIM2, 1000.hz(), clocks);
    let queues = Queues {
        res_queue: RES_QUEUE.split(),
        urc_queue: URC_QUEUE.split(),
        com_queue: COM_QUEUE.split(),
    };
    let (mut client, ingress) =
        ClientBuilder::new(tx, timer, atat::Config::new(atat::Mode::Timeout)).build(queues);
    status.set_low().unwrap();
    ctx.spawn.at_loop().unwrap();
    ctx.spawn.initialise().unwrap();
    ::rtt_target::print_impl::write_str(0, "Init complete\n");
    init::LateResources {
        status,
        rx,
        client,
        ingress,
    }
}
#[allow(non_snake_case)]
fn idle(_: idle::Context) -> ! {
    use rtic::Mutex as _;
    loop {
        core::sync::atomic::spin_loop_hint();
    }
}
#[allow(non_snake_case)]
fn serial_recv(mut ctx: serial_recv::Context) {
    use rtic::Mutex as _;
    let serial_recv::Resources {
        status,
        rx,
        ingress,
        ..
    } = ctx.resources;
    if let Ok(d) = loop {
        #[allow(unreachable_patterns)]
        match rx.read() {
            Err(::nb::Error::Other(e)) =>
            {
                #[allow(unreachable_code)]
                break Err(e)
            }
            Err(::nb::Error::WouldBlock) => {}
            Ok(x) => break Ok(x),
        }
    } {
        ::rtt_target::print_impl::write_fmt(
            0,
            ::core::fmt::Arguments::new_v1(
                &["Received ", "\n"],
                &match (&d,) {
                    (arg0,) => [::core::fmt::ArgumentV1::new(
                        arg0,
                        ::core::fmt::Display::fmt,
                    )],
                },
            ),
        );
        ingress.write(&[d]);
        status.toggle().unwrap();
    }
}
#[allow(non_snake_case)]
fn initialise(mut ctx: initialise::Context) {
    use rtic::Mutex as _;
    let initialise::Resources { mut client, .. } = ctx.resources;
    let response = client.send(&commands::At).expect("AT failed");
    let response = client.send(&commands::Reset).expect("Reset failed");
    ::rtt_target::print_impl::write_fmt(
        0,
        ::core::fmt::Arguments::new_v1(
            &["", "\n"],
            &match (&response,) {
                (arg0,) => [::core::fmt::ArgumentV1::new(arg0, ::core::fmt::Debug::fmt)],
            },
        ),
    );
}
#[allow(non_snake_case)]
fn at_loop(mut ctx: at_loop::Context) {
    use rtic::Mutex as _;
    let at_loop::Resources { mut ingress, .. } = ctx.resources;
    ingress.lock(|at| at.digest());
    ::rtt_target::print_impl::write_str(0, "Digest\n");
    ctx.spawn.at_loop().unwrap();
}
/// Resources initialized at runtime
#[allow(non_snake_case)]
pub struct initLateResources {
    pub client: atat::Client<serial::Tx<USART1>, Timer<TIM2>>,
    pub ingress: atat::IngressManager<consts::U256, atat::NoopUrcMatcher>,
    pub rx: serial::Rx<USART1>,
    pub status: StatusLED,
}
#[allow(non_snake_case)]
#[doc(hidden)]
pub struct initLocals {
    RES_QUEUE: &'static mut ResQueue<consts::U256, consts::U5>,
    URC_QUEUE: &'static mut UrcQueue<consts::U256, consts::U10>,
    COM_QUEUE: &'static mut ComQueue<consts::U3>,
}
impl initLocals {
    #[inline(always)]
    unsafe fn new() -> Self {
        static mut RES_QUEUE: ResQueue<consts::U256, consts::U5> = Queue(heapless::i::Queue::u8());
        static mut URC_QUEUE: UrcQueue<consts::U256, consts::U10> = Queue(heapless::i::Queue::u8());
        static mut COM_QUEUE: ComQueue<consts::U3> = Queue(heapless::i::Queue::u8());
        initLocals {
            RES_QUEUE: &mut RES_QUEUE,
            URC_QUEUE: &mut URC_QUEUE,
            COM_QUEUE: &mut COM_QUEUE,
        }
    }
}
#[allow(non_snake_case)]
///Initialization function
pub mod init {
    #[doc(inline)]
    pub use super::initLocals as Locals;
    ///Tasks that can be `spawn`-ed from this context
    pub struct Spawn {
        _not_send: core::marker::PhantomData<*mut ()>,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Spawn {
        #[inline]
        fn clone(&self) -> Spawn {
            {
                let _: ::core::clone::AssertParamIsClone<core::marker::PhantomData<*mut ()>>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Spawn {}
    #[doc(inline)]
    pub use super::initLateResources as LateResources;
    /// Execution context
    pub struct Context {
        /// Core (Cortex-M) peripherals
        pub core: rtic::export::Peripherals,
        /// Device peripherals
        pub device: stm32f4xx_hal::stm32::Peripherals,
        ///Tasks that can be `spawn`-ed from this context
        pub spawn: Spawn,
    }
    impl Context {
        #[inline(always)]
        pub unsafe fn new(core: rtic::export::Peripherals) -> Self {
            Context {
                device: stm32f4xx_hal::stm32::Peripherals::steal(),
                core,
                spawn: Spawn {
                    _not_send: core::marker::PhantomData,
                },
            }
        }
    }
}
#[allow(non_snake_case)]
///Idle loop
pub mod idle {
    /// Execution context
    pub struct Context {}
    impl Context {
        #[inline(always)]
        pub unsafe fn new(priority: &rtic::export::Priority) -> Self {
            Context {}
        }
    }
}
mod resources {
    use rtic::export::Priority;
    #[allow(non_camel_case_types)]
    pub struct ingress<'a> {
        priority: &'a Priority,
    }
    impl<'a> ingress<'a> {
        #[inline(always)]
        pub unsafe fn new(priority: &'a Priority) -> Self {
            ingress { priority }
        }
        #[inline(always)]
        pub unsafe fn priority(&self) -> &Priority {
            self.priority
        }
    }
}
#[allow(non_snake_case)]
///Resources `serial_recv` has access to
pub struct serial_recvResources<'a> {
    pub status: &'a mut StatusLED,
    pub rx: &'a mut serial::Rx<USART1>,
    pub ingress: &'a mut atat::IngressManager<consts::U256, atat::NoopUrcMatcher>,
}
#[allow(non_snake_case)]
///Hardware task
pub mod serial_recv {
    #[doc(inline)]
    pub use super::serial_recvResources as Resources;
    /// Execution context
    pub struct Context<'a> {
        /// Resources this task has access to
        pub resources: Resources<'a>,
    }
    impl<'a> Context<'a> {
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            Context {
                resources: Resources::new(priority),
            }
        }
    }
}
#[allow(non_snake_case)]
///Resources `initialise` has access to
pub struct initialiseResources<'a> {
    pub client: &'a mut atat::Client<serial::Tx<USART1>, Timer<TIM2>>,
}
#[allow(non_snake_case)]
///Software task
pub mod initialise {
    #[doc(inline)]
    pub use super::initialiseResources as Resources;
    /// Execution context
    pub struct Context<'a> {
        /// Resources this task has access to
        pub resources: Resources<'a>,
    }
    impl<'a> Context<'a> {
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            Context {
                resources: Resources::new(priority),
            }
        }
    }
}
#[allow(non_snake_case)]
///Resources `at_loop` has access to
pub struct at_loopResources<'a> {
    pub ingress: resources::ingress<'a>,
}
#[allow(non_snake_case)]
///Software task
pub mod at_loop {
    #[doc(inline)]
    pub use super::at_loopResources as Resources;
    /// Tasks that can be spawned from this context
    pub struct Spawn<'a> {
        priority: &'a rtic::export::Priority,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl<'a> ::core::clone::Clone for Spawn<'a> {
        #[inline]
        fn clone(&self) -> Spawn<'a> {
            {
                let _: ::core::clone::AssertParamIsClone<&'a rtic::export::Priority>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl<'a> ::core::marker::Copy for Spawn<'a> {}
    impl<'a> Spawn<'a> {
        #[doc(hidden)]
        #[inline(always)]
        pub unsafe fn priority(&self) -> &rtic::export::Priority {
            self.priority
        }
    }
    /// Execution context
    pub struct Context<'a> {
        /// Resources this task has access to
        pub resources: Resources<'a>,
        ///Tasks that can be `spawn`-ed from this context
        pub spawn: Spawn<'a>,
    }
    impl<'a> Context<'a> {
        #[inline(always)]
        pub unsafe fn new(priority: &'a rtic::export::Priority) -> Self {
            Context {
                resources: Resources::new(priority),
                spawn: Spawn { priority },
            }
        }
    }
}
/// Implementation details
const APP: () = { # [ doc = r" Always include the device crate which contains the vector table" ] use stm32f4xx_hal :: stm32 as _ ; # [ allow ( non_upper_case_globals ) ] # [ link_section = ".uninit.rtic0" ] static mut status : core :: mem :: MaybeUninit < StatusLED > = core :: mem :: MaybeUninit :: uninit ( ) ; # [ allow ( non_upper_case_globals ) ] # [ link_section = ".uninit.rtic1" ] static mut rx : core :: mem :: MaybeUninit < serial :: Rx < USART1 > > = core :: mem :: MaybeUninit :: uninit ( ) ; # [ allow ( non_upper_case_globals ) ] # [ link_section = ".uninit.rtic2" ] static mut ingress : core :: mem :: MaybeUninit < atat :: IngressManager < consts :: U256 , atat :: NoopUrcMatcher > > = core :: mem :: MaybeUninit :: uninit ( ) ; impl < 'a > rtic :: Mutex for resources :: ingress < 'a > { type T = atat :: IngressManager < consts :: U256 , atat :: NoopUrcMatcher > ; # [ inline ( always ) ] fn lock < R > ( & mut self , f : impl FnOnce ( & mut atat :: IngressManager < consts :: U256 , atat :: NoopUrcMatcher > ) -> R ) -> R { # [ doc = r" Priority ceiling" ] const CEILING : u8 = 2u8 ; unsafe { rtic :: export :: lock ( ingress . as_mut_ptr ( ) , self . priority ( ) , CEILING , stm32f4xx_hal :: stm32 :: NVIC_PRIO_BITS , f ) } } } # [ allow ( non_upper_case_globals ) ] # [ link_section = ".uninit.rtic3" ] static mut client : core :: mem :: MaybeUninit < atat :: Client < serial :: Tx < USART1 > , Timer < TIM2 > > > = core :: mem :: MaybeUninit :: uninit ( ) ; # [ allow ( non_snake_case ) ] # [ no_mangle ] unsafe fn USART1 ( ) { const PRIORITY : u8 = 2u8 ; rtic :: export :: run ( PRIORITY , | | { crate :: serial_recv ( serial_recv :: Context :: new ( & rtic :: export :: Priority :: new ( PRIORITY ) ) ) } ) ; } impl < 'a > serial_recvResources < 'a > { # [ inline ( always ) ] unsafe fn new ( priority : & 'a rtic :: export :: Priority ) -> Self { serial_recvResources { status : & mut * status . as_mut_ptr ( ) , rx : & mut * rx . as_mut_ptr ( ) , ingress : & mut * ingress . as_mut_ptr ( ) , } } } # [ doc = r" Queue version of a free-list that keeps track of empty slots in" ] # [ doc = r" the following buffers" ] static mut initialise_S0_FQ : rtic :: export :: SCFQ < rtic :: export :: consts :: U1 > = rtic :: export :: Queue ( unsafe { rtic :: export :: iQueue :: u8_sc ( ) } ) ; # [ link_section = ".uninit.rtic4" ] # [ doc = r" Buffer that holds the inputs of a task" ] static mut initialise_S0_INPUTS : [ core :: mem :: MaybeUninit < ( ) > ; 1 ] = [ core :: mem :: MaybeUninit :: uninit ( ) ] ; impl < 'a > initialiseResources < 'a > { # [ inline ( always ) ] unsafe fn new ( priority : & 'a rtic :: export :: Priority ) -> Self { initialiseResources { client : & mut * client . as_mut_ptr ( ) , } } } # [ doc = r" Queue version of a free-list that keeps track of empty slots in" ] # [ doc = r" the following buffers" ] static mut at_loop_S0_FQ : rtic :: export :: SCFQ < rtic :: export :: consts :: U1 > = rtic :: export :: Queue ( unsafe { rtic :: export :: iQueue :: u8_sc ( ) } ) ; struct at_loop_S0_FQ < 'a > { priority : & 'a rtic :: export :: Priority , } impl < 'a > rtic :: Mutex for at_loop_S0_FQ < 'a > { type T = rtic :: export :: SCFQ < rtic :: export :: consts :: U1 > ; # [ inline ( always ) ] fn lock < R > ( & mut self , f : impl FnOnce ( & mut rtic :: export :: SCFQ < rtic :: export :: consts :: U1 > ) -> R ) -> R { # [ doc = r" Priority ceiling" ] const CEILING : u8 = 1u8 ; unsafe { rtic :: export :: lock ( & mut at_loop_S0_FQ , self . priority , CEILING , stm32f4xx_hal :: stm32 :: NVIC_PRIO_BITS , f ) } } } # [ link_section = ".uninit.rtic5" ] # [ doc = r" Buffer that holds the inputs of a task" ] static mut at_loop_S0_INPUTS : [ core :: mem :: MaybeUninit < ( ) > ; 1 ] = [ core :: mem :: MaybeUninit :: uninit ( ) ] ; impl < 'a > at_loopResources < 'a > { # [ inline ( always ) ] unsafe fn new ( priority : & 'a rtic :: export :: Priority ) -> Self { at_loopResources { ingress : resources :: ingress :: new ( priority ) , } } } # [ allow ( non_camel_case_types ) ] # [ doc = "Software tasks spawned from core #0 to be dispatched at priority level 1 by core #0" ] enum R0_P1_S0_T { at_loop , initialise , } # [ automatically_derived ] # [ allow ( unused_qualifications ) ] # [ allow ( non_camel_case_types ) ] impl :: core :: clone :: Clone for R0_P1_S0_T { # [ inline ] fn clone ( & self ) -> R0_P1_S0_T { { * self } } } # [ automatically_derived ] # [ allow ( unused_qualifications ) ] # [ allow ( non_camel_case_types ) ] impl :: core :: marker :: Copy for R0_P1_S0_T { } # [ doc = "Queue of tasks sent by core #0 ready to be dispatched by core #0 at priority level 1" ] static mut R0_P1_S0_RQ : rtic :: export :: SCRQ < R0_P1_S0_T , rtic :: export :: consts :: U2 > = rtic :: export :: Queue ( unsafe { rtic :: export :: iQueue :: u8_sc ( ) } ) ; struct R0_P1_S0_RQ < 'a > { priority : & 'a rtic :: export :: Priority , } impl < 'a > rtic :: Mutex for R0_P1_S0_RQ < 'a > { type T = rtic :: export :: SCRQ < R0_P1_S0_T , rtic :: export :: consts :: U2 > ; # [ inline ( always ) ] fn lock < R > ( & mut self , f : impl FnOnce ( & mut rtic :: export :: SCRQ < R0_P1_S0_T , rtic :: export :: consts :: U2 > ) -> R ) -> R { # [ doc = r" Priority ceiling" ] const CEILING : u8 = 1u8 ; unsafe { rtic :: export :: lock ( & mut R0_P1_S0_RQ , self . priority , CEILING , stm32f4xx_hal :: stm32 :: NVIC_PRIO_BITS , f ) } } } # [ allow ( non_snake_case ) ] # [ doc = "Interrupt handler used by core #0 to dispatch tasks at priority 1" ] # [ no_mangle ] unsafe fn EXTI0 ( ) { # [ doc = r" The priority of this interrupt handler" ] const PRIORITY : u8 = 1u8 ; rtic :: export :: run ( PRIORITY , | | { while let Some ( ( task , index ) ) = R0_P1_S0_RQ . split ( ) . 1 . dequeue ( ) { match task { R0_P1_S0_T :: at_loop => { let ( ) = at_loop_S0_INPUTS . get_unchecked ( usize :: from ( index ) ) . as_ptr ( ) . read ( ) ; at_loop_S0_FQ . split ( ) . 0 . enqueue_unchecked ( index ) ; let priority = & rtic :: export :: Priority :: new ( PRIORITY ) ; crate :: at_loop ( at_loop :: Context :: new ( priority ) ) } R0_P1_S0_T :: initialise => { let ( ) = initialise_S0_INPUTS . get_unchecked ( usize :: from ( index ) ) . as_ptr ( ) . read ( ) ; initialise_S0_FQ . split ( ) . 0 . enqueue_unchecked ( index ) ; let priority = & rtic :: export :: Priority :: new ( PRIORITY ) ; crate :: initialise ( initialise :: Context :: new ( priority ) ) } } } } ) ; } impl init :: Spawn < > { fn at_loop ( & self ) -> Result < ( ) , ( ) > { unsafe { use rtic :: Mutex as _ ; let input = ( ) ; if let Some ( index ) = at_loop_S0_FQ . dequeue ( ) { at_loop_S0_INPUTS . get_unchecked_mut ( usize :: from ( index ) ) . as_mut_ptr ( ) . write ( input ) ; R0_P1_S0_RQ . enqueue_unchecked ( ( R0_P1_S0_T :: at_loop , index ) ) ; rtic :: pend ( stm32f4xx_hal :: stm32 :: Interrupt :: EXTI0 ) ; Ok ( ( ) ) } else { Err ( input ) } } } fn initialise ( & self ) -> Result < ( ) , ( ) > { unsafe { use rtic :: Mutex as _ ; let input = ( ) ; if let Some ( index ) = initialise_S0_FQ . dequeue ( ) { initialise_S0_INPUTS . get_unchecked_mut ( usize :: from ( index ) ) . as_mut_ptr ( ) . write ( input ) ; R0_P1_S0_RQ . enqueue_unchecked ( ( R0_P1_S0_T :: initialise , index ) ) ; rtic :: pend ( stm32f4xx_hal :: stm32 :: Interrupt :: EXTI0 ) ; Ok ( ( ) ) } else { Err ( input ) } } } } unsafe fn spawn_at_loop_S0 ( priority : & rtic :: export :: Priority ) -> Result < ( ) , ( ) > { unsafe { use rtic :: Mutex as _ ; let input = ( ) ; if let Some ( index ) = ( at_loop_S0_FQ { priority , } . lock ( | fq | fq . split ( ) . 1 . dequeue ( ) ) ) { at_loop_S0_INPUTS . get_unchecked_mut ( usize :: from ( index ) ) . as_mut_ptr ( ) . write ( input ) ; ( R0_P1_S0_RQ { priority , } . lock ( | rq | { rq . split ( ) . 0 . enqueue_unchecked ( ( R0_P1_S0_T :: at_loop , index ) ) } ) ) ; rtic :: pend ( stm32f4xx_hal :: stm32 :: Interrupt :: EXTI0 ) ; Ok ( ( ) ) } else { Err ( input ) } } } impl < 'a > at_loop :: Spawn < 'a > { # [ inline ( always ) ] fn at_loop ( & self ) -> Result < ( ) , ( ) > { unsafe { spawn_at_loop_S0 ( self . priority ( ) ) } } } # [ no_mangle ] unsafe extern "C" fn main ( ) -> ! { let _TODO : ( ) = ( ) ; rtic :: export :: assert_send :: < StatusLED > ( ) ; rtic :: export :: assert_send :: < serial :: Rx < USART1 > > ( ) ; rtic :: export :: assert_send :: < atat :: IngressManager < consts :: U256 , atat :: NoopUrcMatcher > > ( ) ; rtic :: export :: assert_send :: < atat :: Client < serial :: Tx < USART1 > , Timer < TIM2 > > > ( ) ; rtic :: export :: interrupt :: disable ( ) ; ( 0 .. 1u8 ) . for_each ( | i | at_loop_S0_FQ . enqueue_unchecked ( i ) ) ; ( 0 .. 1u8 ) . for_each ( | i | initialise_S0_FQ . enqueue_unchecked ( i ) ) ; let mut core : rtic :: export :: Peripherals = rtic :: export :: Peripherals :: steal ( ) . into ( ) ; let _ = [ ( ) ; ( ( 1 << stm32f4xx_hal :: stm32 :: NVIC_PRIO_BITS ) - 1u8 as usize ) ] ; core . NVIC . set_priority ( stm32f4xx_hal :: stm32 :: Interrupt :: EXTI0 , rtic :: export :: logical2hw ( 1u8 , stm32f4xx_hal :: stm32 :: NVIC_PRIO_BITS ) ) ; rtic :: export :: NVIC :: unmask ( stm32f4xx_hal :: stm32 :: Interrupt :: EXTI0 ) ; let _ = [ ( ) ; ( ( 1 << stm32f4xx_hal :: stm32 :: NVIC_PRIO_BITS ) - 2u8 as usize ) ] ; core . NVIC . set_priority ( stm32f4xx_hal :: stm32 :: Interrupt :: USART1 , rtic :: export :: logical2hw ( 2u8 , stm32f4xx_hal :: stm32 :: NVIC_PRIO_BITS ) ) ; rtic :: export :: NVIC :: unmask ( stm32f4xx_hal :: stm32 :: Interrupt :: USART1 ) ; let late = crate :: init ( init :: Locals :: new ( ) , init :: Context :: new ( core . into ( ) ) ) ; client . as_mut_ptr ( ) . write ( late . client ) ; ingress . as_mut_ptr ( ) . write ( late . ingress ) ; rx . as_mut_ptr ( ) . write ( late . rx ) ; status . as_mut_ptr ( ) . write ( late . status ) ; rtic :: export :: interrupt :: enable ( ) ; crate :: idle ( idle :: Context :: new ( & rtic :: export :: Priority :: new ( 0 ) ) ) } };
