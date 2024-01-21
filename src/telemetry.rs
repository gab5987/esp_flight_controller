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

use esp_idf_svc::{
    espnow::{EspNow, PeerInfo},
    eventloop::EspSystemEventLoop,
    hal::peripherals::Peripherals,
    nvs::EspDefaultNvsPartition,
    sys::{esp, EspError, *},
    wifi::{AuthMethod, BlockingWifi, ClientConfiguration, Configuration, EspWifi},
};

const SSID: &str = "gabriel_2.4";
const PASSWORD: &str = "b1e3l245";

// pub enum MessageType {
//     PosReq = 0x00,
//     PosUpdate = 0x01,
// }
//
// struct DataExchange<'a> {
//     msg_type: MessageType,
//     crc: u16,
//     flags: u8,
//     data_len: u8,
//     payload: &'a [u8],
// }
//
// pub struct Telemetry {
//     esp_now: EspNow<'static>,
// }
//
// impl Telemetry {
//     pub fn new() -> Result<Self, EspError> {
//         let peripherals = Peripherals::take()?;
//         let sys_loop = EspSystemEventLoop::take()?;
//         let nvs = EspDefaultNvsPartition::take()?;
//
//         let mut wifi = BlockingWifi::wrap(
//             EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
//             sys_loop,
//         )?;
//
//         let wifi_configuration: Configuration =
//             Configuration::Client::ClientConfiguration::default();
//
//         wifi.set_configuration(&wifi_configuration)?;
//
//         wifi.start()?;
//         log::info!("Wifi started");
//
//         // wifi.connect()?;
//         // log::info!("Wifi connected");
//
//         wifi.wait_netif_up()?;
//         log::info!("Wifi netif up");
//
//         let esp_now = EspNow::take()?;
//         log::info!("Espnow initialized -> version: {}", esp_now.get_version()?);
//         return Ok(Self { esp_now });
//     }
//
//     fn recv_cb(&self, mac_addr: &[u8], data: &[u8]) {}
//
//     pub fn fetch_peer(&self, from_head: bool) -> Result<PeerInfo, EspError> {
//         let mut peer_info = PeerInfo::default();
//         esp!(unsafe { esp_now_fetch_peer(from_head, &mut peer_info as *mut esp_now_peer_info_t) })?;
//         Ok(peer_info)
//     }
//
//     fn print_peer_info(&mut self, num: u8) -> Result<(), EspError> {
//         return match self.fetch_peer(num == 0) {
//             Ok(peer_info) => {
//                 log::info!("peer info -> {:?}", peer_info.peer_addr);
//                 return Ok(());
//             }
//             Err(e) => {
//                 log::error!("error -> {}", e.to_string());
//                 return Err(e);
//             }
//         };
//     }
//
//     pub fn setup(&mut self) -> Result<(), EspError> {
//         self.esp_now
//             .register_recv_cb(|mac, d| self.recv_cb(mac, d))?;
//
//         let mut peer_info = PeerInfo::default();
//         peer_info.peer_addr = [0x44, 0x44, 0x44, 0x44, 0x44, 0x44];
//         self.esp_now.add_peer(peer_info)?;
//         peer_info.peer_addr = [0x56, 0x45, 0x25, 0x57, 0x32, 0x66];
//         self.esp_now.add_peer(peer_info)?;
//         peer_info.peer_addr = [0x66, 0x66, 0x66, 0x66, 0x66, 0x66];
//         self.esp_now.add_peer(peer_info)?;
//
//         self.print_peer_info(0)?;
//         self.print_peer_info(1)?;
//         self.print_peer_info(2)?;
//
//         return Ok(());
//     }
// }

//     fn print_peer_info(&mut self, num: u8) -> Result<(), EspError> {
//         return match self.fetch_peer(num == 0) {
//             Ok(peer_info) => {
//                 log::info!("peer info -> {:?}", peer_info.peer_addr);
//                 return Ok(());
//             }
//             Err(e) => {
//                 log::error!("error -> {}", e.to_string());
//                 return Err(e);
//             }
//         };
//     }

pub fn fetch_peer(from_head: bool) -> Result<PeerInfo, EspError> {
    let mut peer_info = PeerInfo::default();
    esp!(unsafe { esp_now_fetch_peer(from_head, &mut peer_info as *mut esp_now_peer_info_t) })?;
    Ok(peer_info)
}

pub fn initialize() -> Result<(), EspError> {
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
    )?;

    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: PASSWORD.try_into().unwrap(),
        channel: None,
    });

    wifi.set_configuration(&wifi_configuration)?;

    wifi.start()?;
    log::info!("Wifi started");

    wifi.connect()?;
    log::info!("Wifi connected");

    wifi.wait_netif_up()?;
    log::info!("Wifi netif up");

    let esp_now = EspNow::take()?;
    log::info!("Espnow initialized -> version: {}", esp_now.get_version()?);

    let mut peer_info = PeerInfo::default();
    peer_info.peer_addr = [0x44, 0x44, 0x44, 0x44, 0x44, 0x44];
    esp_now.add_peer(peer_info)?;
    peer_info.peer_addr = [0x56, 0x45, 0x25, 0x57, 0x32, 0x66];
    esp_now.add_peer(peer_info)?;
    peer_info.peer_addr = [0x66, 0x66, 0x66, 0x66, 0x66, 0x66];
    esp_now.add_peer(peer_info)?;

    match fetch_peer(true) {
        Ok(peer_info) => {
            log::info!("peer info -> {:?}", peer_info.peer_addr);
        }
        Err(e) => {
            log::error!("error -> {}", e.to_string());
        }
    };

    match fetch_peer(false) {
        Ok(peer_info) => {
            log::info!("peer info -> {:?}", peer_info.peer_addr);
        }
        Err(e) => {
            log::error!("error -> {}", e.to_string());
        }
    };

    match fetch_peer(false) {
        Ok(peer_info) => {
            log::info!("peer info -> {:?}", peer_info.peer_addr);
        }
        Err(e) => {
            log::error!("error -> {}", e.to_string());
        }
    };

    return Ok(());

    // return match Telemetry::new() {
    //     Ok(mut tel) => {
    //         tel.setup()?;
    //         return Ok(tel);
    //     }
    //     Err(e) => {
    //         log::error!("telemetry error -> {}", e.to_string());
    //         return Err(e);
    //     }
    // };
}
