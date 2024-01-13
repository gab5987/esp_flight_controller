/*
 * REQ-TEL-000: Shall initialize both the espnow and wifi modules, providing a
 * small footprint, full-duplex comunication between the controller and the
 * device.
 *
 * REQ-TEL-001: Shall use the espnow to receive controller info and actuate into
 * the flight controller, updating the device's state machine.
 *
 * REQ-TEL-002: Shall provide a telemetry interface, sending curent data such as
 * speed, altitude, rotation, currenttime and remaining operational time.
 *
 * REQ-TEL-003: Provide data corruption and error handling into the transmited
 * data. CRCs might just enought.
 * */

use esp_idf_svc::{espnow::EspNow, sys::EspError};

pub enum MessageType {
    PosReq = 0x00,
    PosUpdate = 0x01,
}

struct DataExchange<'a> {
    msg_type: MessageType,
    crc: u16,
    flags: u8,
    data_len: u8,
    payload: &'a [u8],
}

pub struct Telemetry {
    esp_now: EspNow<'static>,
}

impl Telemetry {
    pub fn new() -> Result<Self, EspError> {
        let esp_now = EspNow::take()?;
        log::info!("Espnow initialized -> version: {}", esp_now.get_version()?);
        return Ok(Self { esp_now });
    }

    fn recv_cb(&self, mac_addr: &[u8], data: &[u8]) {}

    pub fn setup(&self) -> Result<(), EspError> {
        self.esp_now
            .register_recv_cb(|mac, d| self.recv_cb(mac, d))?;

        return Ok(());
    }
}

pub fn initialize() -> Result<Telemetry, EspError> {
    let tel = Telemetry::new()?;
    tel.setup()?;
    return Ok(tel);
}
