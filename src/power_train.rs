/*
*
* REQ-PWT-000: Shall initialize the four channels of pwm for all the four mototrs using the least
* amount of times as possible. I think i can use two, one for each side.
*
* REQ-PWT-001: Find a way to self test the motors. The EM40A has a sink current pin, i can use this
* and the provided nominal amperage loaded/non-loaded values to determine if the mototrs are
* behaving as expected.
*
* REQ-PWT-002: Find out what is the best frequency and duty cycle values for each mode.
*
* REQ-PWT-003: Shall provide a small api to abstract the duty cycle values to a percentage.
*
* REQ-PWT-004: Create a fail safe mode, im still not entirelly sure how to do it.
*
* */

use esp_idf_svc::sys::EspError;

pub fn initialize() -> Result<(), EspError> {
    return Ok(());
}
