//! Rust implementation of the Robotis Dynamixel motor protocol.
//!
//! ## Example
//!
//! ```ignore
//! extern crate dynamixel;
//!
//! use dynamixel::motors::XL_320;
//!
//! fn main() {
//!     let mut c = dynamixel::with_protocol_v2(my_serial);
//!
//!     loop {
//!         let pos = c.read_data(ID1, XL_320::PresentPosition).unwrap();
//!         c.write_data(ID2, XL_320::GoalPosition, pos).ok();
//!     }
//! }
//! ```
#![deny(missing_docs)]
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
pub use protocol::ControllerV2;

/// Create a controller for the Dynamixel protocol V2 using a serial RX/TX
pub fn with_protocol_v2<RX, TX>(rx: RX, tx: TX) -> ControllerV2<RX, TX>
where
    RX: Read<u8, Error = !>,
    TX: Write<u8, Error = !>,
{
    ControllerV2::new(rx, tx)
}
