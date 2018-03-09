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
#![deny(unsafe_code)]
#![deny(missing_docs)]
#![feature(never_type)]
#![feature(const_fn)]
#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(not(feature = "std"), feature(alloc))]

#[cfg(not(feature = "std"))]
#[macro_use(format, vec)]
extern crate alloc;

extern crate embedded_hal as hal;
use hal::serial::{Read, Write};
use hal::time::Time;

extern crate crc16;
#[macro_use(block)]
extern crate nb;

mod error;
#[macro_use]
pub mod motors;
#[macro_use]
mod protocol;
pub use protocol::ControllerV2;

/// Create a controller for the Dynamixel protocol V2 using a serial RX/TX
pub fn with_protocol_v2<RX, TX, CLOCK>(rx: RX, tx: TX, clock: CLOCK) -> ControllerV2<RX, TX, CLOCK>
where
    RX: Read<u8, Error = !>,
    TX: Write<u8, Error = !>,
    CLOCK: Time,
{
    ControllerV2::new(rx, tx, clock)
}
