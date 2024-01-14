use core::panic;
use esp_idf_svc::hal::delay::FreeRtos;

mod imu;
mod mcpwm;
mod power_train;
mod telemetry;

use mcpwm::{McpwmPeripheral, TimerConfig, TimerDriver};

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly.
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let mcpwm_peripheral = McpwmPeripheral::take();
    let timer_config = TimerConfig::default().period_ticks(8_000);
    let timer = TimerDriver::new(mcpwm_peripheral.mcpwm0.timer0, timer_config).unwrap();

    let period = timer.get_period_ticks();

    println!("period: {}", period);

    // let mut imu_inst = match imu::Imu::init() {
    //     Ok(imu) => imu,
    //     Err(e) => {
    //         panic!("Imu initialization failed -> {}", e.to_string());
    //     }
    // };
    //
    // let imu_jn = match imu_inst.run() {
    //     Ok(jn) => jn,
    //     Err(e) => {
    //         panic!("Imu run failed -> {}", e.to_string());
    //     }
    // };
    //
    // let _ = telemetry::initialize();
    //
    // imu_jn.join().unwrap();
    //
    loop {
        FreeRtos::delay_ms(500);
    }
}
