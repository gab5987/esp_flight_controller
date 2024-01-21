use esp_idf_svc::hal::{delay::FreeRtos, rom::crc};

mod imu;
mod mcpwm;
mod power_train;
mod telemetry;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly.
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

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
    let _ = telemetry::initialize();
    //
    // imu_jn.join().unwrap();
    //
    loop {
        FreeRtos::delay_ms(500);
    }
}
