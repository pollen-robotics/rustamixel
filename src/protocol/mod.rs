macro_rules! busy_wait {
    ($e:expr, $clock:expr, $timeout:expr) => {{
        let t0 = $clock.now();
        loop {
            if let Ok(b) = $e {
                break Ok(b);
            }

            if ($clock.now() - t0) > $timeout {
                break Err(DynamixelError::timeout());
            }
        }
    }};
}

mod v2;
pub use self::v2::ControllerV2;
