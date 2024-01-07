use esp_idf_svc::sys::abort;

mod imu;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. 
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let ret: i32 = imu::initialize();
    
    if ret != 0 {
        log::error!("Imu failed to Initialize");
        unsafe { abort(); };
    }
}

