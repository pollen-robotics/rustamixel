pub trait Register {
    fn address(&self) -> u16;
    fn length(&self) -> u16;
}

macro_rules! register {
    ($($reg:ident : $addr:expr, $len:expr,)+) => {
        $(
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
