use esp_idf_svc::{sys::abort, hal::delay::FreeRtos};

mod imu;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. 
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let mut imu_inst: imu::Imu = imu::Imu::new().unwrap();

    if let Err(e) = imu_inst.run() {
        log::error!("Imu failed to Initialize -> {}", e.to_string());
        unsafe { abort(); };
    }

    loop 
    {
        FreeRtos::delay_ms(500);
    }
}

