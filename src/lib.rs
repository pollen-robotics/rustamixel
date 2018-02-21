#![feature(never_type)]

extern crate embedded_hal as hal;
use hal::serial::{Read, Write};

extern crate crc16;
#[macro_use(block)]
extern crate nb;

mod error;
#[macro_use]
pub mod motors;
mod protocol;

pub fn with_protocol_v2<RX, TX>(rx: RX, tx: TX) -> protocol::V2<RX, TX>
where
    RX: Read<u8, Error = !>,
    TX: Write<u8, Error = !>,
{
    protocol::V2::new(rx, tx)
}
