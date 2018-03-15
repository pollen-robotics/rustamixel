//! Definition of the `XL_320` registers

register![
    ID: 0x03, 1,
    TorqueEnable: 0x18, 1,
    PresentPosition: 0x25, 2,
    GoalPosition: 0x1E, 2,
    MovingSpeed: 0x20, 2,
    TorqueLimit: 0x23, 2,
];
