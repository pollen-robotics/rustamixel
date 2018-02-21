#[derive(Debug)]
enum ErrorType {
    Parsing,
    UnsupportedRegister,
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
    pub fn description(&self) -> String {
        format!("Dynxamiel Error: {:?}", self.error)
    }
}
