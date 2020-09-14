use atat::{
    atat_derive::{AtatCmd, AtatResp, AtatUrc},
    AtatCmd, AtatResp, AtatUrc, Error,
};
use core::fmt::Write;
use heapless::{String, Vec};
use rtt_target::rprintln;

#[derive(Clone, Debug, AtatResp)]
pub struct EmptyResponse;

// #[derive(Clone, Debug)]
#[derive(Clone, Debug, AtatCmd)]
#[at_cmd("", EmptyResponse, timeout_ms = 5000)]
pub struct At;

// impl AtatCmd for At {
//     type CommandLen = heapless::consts::U4;
//     type Response = EmptyResponse;

//     fn as_bytes(&self) -> Vec<u8, Self::CommandLen> {
//         let mut buf: Vec<u8, Self::CommandLen> = Vec::new();
//         write!(buf, "AT\r\n");
//         buf
//     }

//     fn parse(&self, resp: &[u8]) -> Result<Self::Response, Error> {
//         rprintln!("Resp {:?}", resp);
//         Ok(EmptyResponse)
//     }

//     fn max_timeout_ms(&self) -> u32 {
//         2000u32
//     }
// }

#[derive(Clone, Debug, AtatCmd)]
#[at_cmd("+RST", EmptyResponse, timeout_ms = 1000)]
pub struct Reset;

#[derive(Clone, Debug, AtatCmd)]
#[at_cmd("+GMR", EmptyResponse, timeout_ms = 1000)]
pub struct FirmwareVersion;

#[derive(Clone, Debug, AtatCmd)]
#[at_cmd("E0", EmptyResponse, timeout_ms = 1000)]
pub struct DisableEcho;

#[derive(Clone, Debug, AtatCmd)]
#[at_cmd("E1", EmptyResponse, timeout_ms = 1000)]
pub struct EnableEcho;
