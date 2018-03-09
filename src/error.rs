#[cfg(not(feature = "std"))]
use alloc::String;

#[derive(Debug, PartialEq)]
pub enum ErrorType {
    Parsing,
    UnsupportedRegister,
    StatusError(u8),
    InvalidChecksum,
    Timeout,
}

#[derive(Debug)]
pub struct DynamixelError {
    pub error: ErrorType,
}
impl DynamixelError {
    pub fn parsing_error() -> DynamixelError {
        DynamixelError {
            error: ErrorType::Parsing,
        }
    }
    pub fn unsupported_register() -> DynamixelError {
        DynamixelError {
            error: ErrorType::UnsupportedRegister,
        }
    }
    pub fn status_error_code(e: u8) -> DynamixelError {
        DynamixelError {
            error: ErrorType::StatusError(e),
        }
    }
    pub fn invalid_checksum() -> DynamixelError {
        DynamixelError {
            error: ErrorType::InvalidChecksum,
        }
    }
    pub fn timeout() -> DynamixelError {
        DynamixelError {
            error: ErrorType::Timeout,
        }
    }
    pub fn description(&self) -> String {
        format!("Dynxamiel Error: {:?}", self.error)
    }
}
