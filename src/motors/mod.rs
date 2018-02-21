//! Motors abstraction module. Defined all supported Motors.
//!
//! The supported motors are currently
//!    * the `XL_320`
//!
//! Adding support for a new type of motor should only require to add a new submodule with the specific registers.
//!
//! [Contributions are welcomed!](https://github.com/pollen-robotics/rustamixel)

/// Register trait shared by all dynamixel motor registers.
pub trait Register {
    /// Address of the register
    fn address(&self) -> u16;
    /// Length of the register (usually 1 or 2 for the common registers)
    fn length(&self) -> u16;
}

macro_rules! register {
    ($($reg:ident : $addr:expr, $len:expr,)+) => {
        $(
            #[allow(missing_docs)]
            pub struct $reg;
            impl super::Register for $reg {
                fn address(&self) -> u16 { $addr }
                fn length(&self) -> u16 { $len }
            }
        )+
    }
}

#[allow(non_snake_case)]
pub mod XL_320;

macro_rules! pack {
    ($l:expr, $h:expr) => (u16::from($h) << 8 | u16::from($l))
}
macro_rules! unpack {
    ($b:expr) => {
        ($b as u8, ($b >> 8) as u8)
    }
}
