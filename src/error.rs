#[derive(Debug)]
enum ErrorType {
    Parsing,
    UnsupportedRegister,
    StatusError(u8),
    InvalidChecksum,
}

#[derive(Debug)]
pub struct DynamixelError {
    error: ErrorType,
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
    pub fn description(&self) -> String {
        format!("Dynxamiel Error: {:?}", self.error)
    }
}
