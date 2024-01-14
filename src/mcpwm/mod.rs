//! Motor Control Pulse Width Modulator peripheral
//!
//! Interface to the [Motor Control Pulse Width Modulator peripheral (MCPWM)
//! peripheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html)
//!
//! ```
//!   --------------------------------------------------------------------------
//!  |                             MCPWM Group N                                |
//!  |                       -------------------------                          |
//!  |                                                                          |
//!  |  ---------            -------------------------------------------------  |
//!  | | Timer 0 |-------*  |              OPERATOR  0                        | |
//!  |  ---------        |  |            ---------------                      | |
//!  |  ---------        |  |                                  -------------  | |
//!  | | Timer 1 |----*  |  |        *----------------------->|             | | |
//!  |  ---------     |  |  |        |                *------>| GENERATOR 0 |-|-|--> To Output pin
//!  |  ---------     |  *--|-|\     |                | *---->|             | | |
//!  | | Timer 2 |-*  |  |  | |  \   *->Comparator 0>-* |      -------------  | |
//!  |  ---------  |  *-----|-|   |>-|                | |                     | |
//!  |             |  |  |  | |  /   *->Comparator 1>-|-*      -------------  | |
//!  |             *--------|-|/     |                *-|---->|             | |-|--> To Output pin
//!  |             |  |  |  |        |                  *---->| GENERATOR 1 | | |
//!  |             |  |  |  |        *----------------------->|             | | |
//!  |             |  |  |  |                                  -------------  | |
//!  |             |  |  |   -------------------------------------------------  |
//!  |             |  |  |                                                      |
//!  |             |  |  |   -------------------------------------------------  |
//!  |             |  |  *--|              OPERATOR  1                        | |
//!  |             |  |  |  |            ---------------                      |-|--> To Output pin
//!  |             |  *-----|                                                 | |
//!  |             |  |  |  |                  ...                            |-|--> To Output pin
//!  |             *--------|                                                 | |
//!  |             |  |  |   -------------------------------------------------  |
//!  |             |  |  |                                                      |
//!  |             |  |  |   -------------------------------------------------  |
//!  |             |  |  *--|              OPERATOR  2                        | |
//!  |             |  |  |  |            ---------------                      |-|--> To Output pin
//!  |             |  *-----|                                                 | |
//!  |             |  |  |  |                  ...                            |-|--> To Output pin
//!  |             *--------|                                                 | |
//!  |                       -------------------------------------------------  |
//!  |                                                                          |
//!  |                                                                          |
//!   -------------------------------------------------------------------------
//! ```

mod operator;
mod timer;

use core::ffi;

pub use self::timer::{TimerConfig, TimerDriver, TIMER};

pub struct MCPWM<G: Group> {
    pub timer0: TIMER<0, G>,
    pub timer1: TIMER<1, G>,
    pub timer2: TIMER<2, G>,
}

impl<G: Group> MCPWM<G> {
    pub unsafe fn new() -> Self {
        Self {
            timer0: TIMER::new(),
            timer1: TIMER::new(),
            timer2: TIMER::new(),
        }
    }
}

#[derive(Default)]
pub struct Group0;

#[cfg(not(esp32c6))]
#[derive(Default)]
pub struct Group1;

pub type Duty = u16;

// This was called `Unit` in IDF < 5.0
pub trait Group: Default {
    const ID: ffi::c_int;
}

impl Group for Group0 {
    const ID: ffi::c_int = 0;
}

#[cfg(not(esp32c6))]
impl Group for Group1 {
    const ID: ffi::c_int = 1;
}

pub struct McpwmPeripheral {
    pub mcpwm0: MCPWM<Group0>,
    pub mcpwm1: MCPWM<Group1>,
}

impl McpwmPeripheral {
    // Can only call this function once!!
    pub fn take() -> Self {
        unsafe {
            return Self {
                mcpwm0: MCPWM::<Group0>::new(),
                mcpwm1: MCPWM::<Group1>::new(),
            };
        }
    }
}
