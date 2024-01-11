use esp_idf_svc::{sys::abort, hal::delay::FreeRtos};

mod imu;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. 
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    if let Err(e) = imu::initialize() {
        log::error!("Imu failed to Initialize -> {}", e.to_string());
        unsafe { abort(); };
    }

    let _ = imu::run().unwrap().join();

    loop 
    {
        FreeRtos::delay_ms(500);
    }
}

