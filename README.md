# Dynamixel Rust library

This crate provides a Rust implementation of the Robotis Dynamixel motor protocol. It is still a work in progress.

Its goal is to support:

* both protocol v1 and v2
* all types of Robotis motor
* work with no_std environment

## Example

```
extern crate dynamixel;

use dynamixel::motors::XL_320;

fn main() {
    let mut c = dynamixel::with_protocol_v2(my_serial);

    loop {
        let pos = c.read_data(ID1, XL_320::PresentPosition).unwrap();
        c.write_data(ID2, XL_320::GoalPosition, pos).ok();
    }
}
```
