// Don't show the console window when run from an executable.
// Note: This surpresses all console out including panics and shader errors,
// so only uncomment for releases.
// #![windows_subsystem = "windows"]

#![allow(unused)] // For the copy+paste code in `types`.
#![allow(clippy::derive_partial_eq_without_eq)] // For the copy+paste code in `types`.

use std::{
    convert::TryInto,
    f32::consts::TAU,
    io::{self, Read},
    thread,
    time::{self, Duration, Instant},
};

use serialport::{self, SerialPort, SerialPortType};

use lin_alg2::f32::Quaternion;
use types::*;

mod render;
mod types;
mod ui;

const FC_SERIAL_NUMBER: &str = "AN";

const BAUD: u32 = 115_200;

const TIMEOUT_MILIS: u64 = 10;

// At this interval, in seconds, request new data from the FC.
// todo: Do you want some (or all?) of the read data to be pushed at a regular
// todo interval on request from this program, or pushed at an interval from the FC
// todo without explicitly requesting here?
// todo: Also: multiple intervals for different sorts of data, eg update
// todo attitude at a higher rate than other things.
const READ_INTERVAL: f32 = 0.1;
const READ_INTERVAL_MS: u128 = (READ_INTERVAL * 1_000.) as u128;

/// Data passed by the flight controller
pub struct State {
    pub attitude: Quaternion,
    pub attitude_commanded: Quaternion,
    pub controls: ChannelData,
    pub link_stats: LinkStats,
    pub waypoints: [Option<Location>; MAX_WAYPOINTS],
    pub altitude_baro: f32,
    pub altitude_agl: Option<f32>,
    pub batt_v: f32,
    pub current: f32,
    pub lat: Option<f32>,
    pub lon: Option<f32>,
    // todo: aftleft etc, or 1-4?
    pub rpm1: Option<f32>,
    pub rpm2: Option<f32>,
    pub rpm3: Option<f32>,
    pub rpm4: Option<f32>,
    pub autopilot_status: AutopilotStatus,
    pub last_attitude_update: Instant,
    pub last_controls_update: Instant,
    pub last_link_stats_update: Instant,
    pub system_status: SystemStatus,
    pub aircraft_type: AircraftType,
    /// Note directly pulled from the FC; usd for sequencing read
    /// todo: Consider diff update rate (and last query times) for
    /// todo different types of data.
    pub last_fc_query: Instant,
    interface: SerialInterface,
}

impl Default for State {
    fn default() -> Self {
        // todo: Really?
        let waypoints = [
            None, None, None, None, None, None, None, None, None, None, None, None, None, None,
            None, None, None, None, None, None, None, None, None, None, None, None, None, None,
            None, None,
        ];

        Self {
            attitude: Quaternion::new_identity(),
            attitude_commanded: Quaternion::new_identity(),
            controls: Default::default(),
            link_stats: Default::default(),
            waypoints,
            altitude_baro: 0.,
            altitude_agl: None,
            batt_v: 0.,
            current: 0.,
            lat: None,
            lon: None,
            rpm1: None,
            rpm2: None,
            rpm3: None,
            rpm4: None,
            autopilot_status: Default::default(),
            last_attitude_update: Instant::now(),
            last_controls_update: Instant::now(),
            last_link_stats_update: Instant::now(),
            system_status: Default::default(),
            aircraft_type: AircraftType::Quadcopter,
            last_fc_query: Instant::now(),
            interface: SerialInterface::new(),
        }
    }
}

impl State {
    /// Read parameters, such as attitude and altitutde
    // pub fn read_params(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_params(&mut self) -> Result<(), io::Error> {
        // todo: DRY with port. Trouble passing it as a param due to box<dyn
        let port = match self.interface.serial_port.as_mut() {
            Some(p) => p,
            None => {
                return Err(io::Error::new(
                    io::ErrorKind::NotConnected,
                    "Flight controller not connected",
                ))
            }
        };

        let crc_tx = calc_crc(
            &CRC_LUT,
            &[MsgType::ReqParams as u8],
            MsgType::ReqParams.payload_size() as u8 + 1,
        );
        let xmit_buf = &[MsgType::ReqParams as u8, crc_tx];

        // Write the buffer requesting params from the FC.
        port.write_all(xmit_buf)?;

        // Read the params passed by the FC in response.
        let mut rx_buf = [0; PARAMS_SIZE + 2];
        port.read_exact(&mut rx_buf)?;

        // The order (or equivalently indices) of params here must match the FC firmware. Use it
        // as a reference.
        let mut i = 1;

        let attitude_data: [u8; QUATERNION_SIZE] =
            rx_buf[i..QUATERNION_SIZE + i].try_into().unwrap();

        self.attitude = quat_from_buf(&attitude_data);
        i += QUATERNION_SIZE;

        let attitude_commanded_data: [u8; QUATERNION_SIZE] =
            rx_buf[i..QUATERNION_SIZE + i].try_into().unwrap();

        self.attitude_commanded = quat_from_buf(&attitude_commanded_data);
        i += QUATERNION_SIZE;

        self.altitude_baro = f32::from_be_bytes(rx_buf[i..F32_SIZE + i].try_into().unwrap());
        i += F32_SIZE;

        self.altitude_agl = match rx_buf[i] {
            0 => None,
            _ => Some(f32::from_be_bytes(
                rx_buf[i + 1..F32_SIZE + i + 1].try_into().unwrap(),
            )),
        };
        i += F32_SIZE + 1;

        self.batt_v = f32::from_be_bytes(rx_buf[i..F32_SIZE + i].try_into().unwrap());
        i += F32_SIZE;

        self.current = f32::from_be_bytes(rx_buf[i..F32_SIZE + i].try_into().unwrap());
        i += F32_SIZE;

        // todo: CRC check on other items, and impl it here.
        // todo: Probably return a suitable error type here.
        let payload_size = MsgType::ReqParams.payload_size();
        let crc_rx_expected = calc_crc(
            &CRC_LUT,
            &rx_buf[..payload_size + 1],
            payload_size as u8 + 1,
        );
        let crc_read = rx_buf[PARAMS_SIZE] + 1;
        if crc_read != crc_rx_expected {
            // todo: Do something else here? Eg don't update self and resend.
            println!("Incorrect CRC from reading params!");
        }

        Ok(())
    }

    /// Read system status, and autopilot status
    // pub fn read_controls(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_sys_ap_status(&mut self) -> Result<(), io::Error> {
        // todo: DRY with port. Trouble passing it as a param due to box<dyn
        let port = match self.interface.serial_port.as_mut() {
            Some(p) => p,
            None => {
                return Err(io::Error::new(
                    io::ErrorKind::NotConnected,
                    "Flight controller not connected",
                ))
            }
        };

        let crc_tx = calc_crc(
            &CRC_LUT,
            &[MsgType::ReqSysApStatus as u8],
            MsgType::ReqSysApStatus.payload_size() as u8 + 1,
        );
        let xmit_buf = &[MsgType::ReqSysApStatus as u8, crc_tx];

        port.write_all(xmit_buf)?;

        // todo: Just sys status for now; do AP too.
        let mut rx_buf = [0; SYS_AP_STATUS_SIZE + 2];
        port.read_exact(&mut rx_buf)?;

        let sys_status: [u8; SYS_AP_STATUS_SIZE] =
            rx_buf[1..SYS_AP_STATUS_SIZE + 1].try_into().unwrap();
        self.system_status = sys_status.into();

        Ok(())
    }

    /// Read control channel data.
    // pub fn read_controls(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_controls(&mut self) -> Result<(), io::Error> {
        // todo: DRY with port. Trouble passing it as a param due to box<dyn
        let port = match self.interface.serial_port.as_mut() {
            Some(p) => p,
            None => {
                return Err(io::Error::new(
                    io::ErrorKind::NotConnected,
                    "Flight controller not connected",
                ))
            }
        };

        let crc_tx = calc_crc(
            &CRC_LUT,
            &[MsgType::ReqControls as u8],
            MsgType::ReqControls.payload_size() as u8 + 1,
        );
        let xmit_buf = &[MsgType::ReqControls as u8, crc_tx];

        port.write_all(xmit_buf)?;

        // let mut rx_buf = [0; CONTROLS_SIZE + 2]; // todo: Bogus leading 1?
        let mut rx_buf = [0; CONTROLS_SIZE + 2];
        port.read_exact(&mut rx_buf)?;

        let controls: [u8; CONTROLS_SIZE] = rx_buf[1..CONTROLS_SIZE + 1].try_into().unwrap();
        self.controls = controls.into();

        Ok(())
    }

    /// Read controller link stats data.
    // pub fn read_link_stats(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_link_stats(&mut self) -> Result<(), io::Error> {
        // todo: DRY with port. Trouble passing it as a param due to box<dyn
        let port = match self.interface.serial_port.as_mut() {
            Some(p) => p,
            None => {
                return Err(io::Error::new(
                    io::ErrorKind::NotConnected,
                    "Flight controller not connected",
                ))
            }
        };

        let crc_tx = calc_crc(
            &CRC_LUT,
            &[MsgType::ReqLinkStats as u8],
            MsgType::ReqLinkStats.payload_size() as u8 + 1,
        );
        let xmit_buf = &[MsgType::ReqLinkStats as u8, crc_tx];

        // todo: DRY between these calls
        port.write_all(xmit_buf)?;

        let mut rx_buf = [0; LINK_STATS_SIZE + 2];
        port.read_exact(&mut rx_buf)?;

        let link_stats: [u8; LINK_STATS_SIZE] = rx_buf[1..LINK_STATS_SIZE + 1].try_into().unwrap();

        self.link_stats = link_stats.into();

        Ok(())
    }

    /// Read waypoints data from the flight controller.
    // pub fn read_waypoints(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_waypoints(&mut self) -> Result<(), io::Error> {
        // todo: DRY with port. Trouble passing it as a param due to box<dyn
        let port = match self.interface.serial_port.as_mut() {
            Some(p) => p,
            None => {
                return Err(io::Error::new(
                    io::ErrorKind::NotConnected,
                    "Flight controller not connected",
                ))
            }
        };

        let crc_tx = calc_crc(
            &CRC_LUT,
            &[MsgType::ReqWaypoints as u8],
            MsgType::ReqWaypoints.payload_size() as u8 + 1,
        );
        let xmit_buf = &[MsgType::ReqWaypoints as u8, crc_tx];

        port.write_all(xmit_buf)?;

        let mut rx_buf = [0; WAYPOINTS_SIZE + 2];
        port.read_exact(&mut rx_buf)?;

        let mut wp_buf = [0; WAYPOINTS_SIZE];
        wp_buf.clone_from_slice(&rx_buf[1..WAYPOINTS_SIZE + 1]);

        let waypoints_data = waypoints_from_buf(wp_buf);

        self.waypoints = waypoints_data;

        // todo: Lat, Lon

        Ok(())
    }

    /// Request several types of data from the flight controller over USB serial. Return a struct
    /// containing the data.
    pub fn read_all(&mut self) -> Result<(), io::Error> {
        let mut port = match self.interface.serial_port.as_mut() {
            Some(p) => p,
            None => {
                return Err(io::Error::new(
                    io::ErrorKind::NotConnected,
                    "Flight controller not connected",
                ))
            }
        };

        // self.read_params(port)?;
        // self.read_controls(port)?;
        // self.read_link_stats(port)?;
        // self.read_waypoints(port)?;
        //
        self.read_params()?;
        self.read_sys_ap_status()?;
        self.read_controls()?;
        self.read_link_stats()?;
        self.read_waypoints()?;

        Ok(())
    }

    pub fn send_arm_command(&mut self) -> Result<(), io::Error> {
        if let Some(port) = self.interface.serial_port.as_mut() {
            let msg_type = MsgType::ArmMotors;
            let crc = calc_crc(
                &CRC_LUT,
                &[msg_type as u8],
                msg_type.payload_size() as u8 + 1,
            );
            let xmit_buf = &[msg_type as u8, crc];
            port.write_all(xmit_buf)?;

            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "Flight controller not connected",
            ))
        }
    }

    pub fn send_disarm_command(&mut self) -> Result<(), io::Error> {
        if let Some(port) = self.interface.serial_port.as_mut() {
            let msg_type = MsgType::DisarmMotors;
            let crc = calc_crc(
                &CRC_LUT,
                &[msg_type as u8],
                msg_type.payload_size() as u8 + 1,
            );

            let xmit_buf = &[msg_type as u8, crc];
            port.write_all(xmit_buf)?;

            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "Flight controller not connected",
            ))
        }
    }

    // todo: These are incomplete. you need to pass which motor etc.
    pub fn send_start_motor_command(&mut self, motor: RotorPosition) -> Result<(), io::Error> {
        if let Some(port) = self.interface.serial_port.as_mut() {
            let msg_type = MsgType::StartMotor;
            let crc = calc_crc(
                &CRC_LUT,
                &[msg_type as u8],
                msg_type.payload_size() as u8 + 1,
            );

            let xmit_buf = &[msg_type as u8, motor as u8, crc];
            port.write_all(xmit_buf)?;

            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "Flight controller not connected",
            ))
        }
    }

    pub fn send_stop_motor_command(&mut self, motor: RotorPosition) -> Result<(), io::Error> {
        if let Some(port) = self.interface.serial_port.as_mut() {
            let msg_type = MsgType::StopMotor;
            let crc = calc_crc(
                &CRC_LUT,
                &[msg_type as u8],
                msg_type.payload_size() as u8 + 1,
            );
            let xmit_buf = &[msg_type as u8, motor as u8, crc];
            port.write_all(xmit_buf)?;

            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "Flight controller not connected",
            ))
        }
    }

    pub fn send_set_servo_posit_command(
        &mut self,
        servo_posit: ServoWingPosition,
        value: f32,
    ) -> Result<(), io::Error> {
        if let Some(port) = self.interface.serial_port.as_mut() {
            let msg_type = MsgType::SetServoPosit;
            let crc = calc_crc(
                &CRC_LUT,
                &[msg_type as u8],
                msg_type.payload_size() as u8 + 1,
            );

            let v = value.to_be_bytes();
            let xmit_buf = &[
                msg_type as u8,
                servo_posit as u8,
                v[0],
                v[1],
                v[2],
                v[3],
                crc,
            ];

            port.write_all(xmit_buf)?;

            Ok(())
        } else {
            Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "Flight controller not connected",
            ))
        }
    }

    /// Close the serial port
    pub fn close(&mut self) {}
}

/// Convert radians to degrees
fn to_degrees(v: f32) -> f32 {
    v * 360. / TAU
}

fn quat_from_buf(p: &[u8; QUATERNION_SIZE]) -> Quaternion {
    Quaternion {
        w: bytes_to_float(&p[0..4]),
        x: bytes_to_float(&p[4..8]),
        y: bytes_to_float(&p[8..12]),
        z: bytes_to_float(&p[12..16]),
    }
}

impl From<[u8; SYS_STATUS_SIZE]> for SystemStatus {
    fn from(p: [u8; SYS_STATUS_SIZE]) -> Self {
        // todo: You could achieve more efficient packing.
        SystemStatus {
            imu: p[0].try_into().unwrap(),
            baro: p[1].try_into().unwrap(),
            gps: p[2].try_into().unwrap(),
            tof: p[3].try_into().unwrap(),
            magnetometer: p[4].try_into().unwrap(),
            esc_telemetry: p[5].try_into().unwrap(),
            esc_rpm: p[6].try_into().unwrap(),
            rf_control_link: p[7].try_into().unwrap(),
            rf_control_fault: p[8] != 0,
        }
    }
}

impl From<[u8; CONTROLS_SIZE]> for ChannelData {
    /// 19 f32s x 4 = 76. In the order we have defined in the struct.
    fn from(p: [u8; CONTROLS_SIZE]) -> Self {
        ChannelData {
            pitch: bytes_to_float(&p[0..4]),
            roll: bytes_to_float(&p[4..8]),
            yaw: bytes_to_float(&p[8..12]),
            throttle: bytes_to_float(&p[12..16]),

            arm_status: p[16].try_into().unwrap(),
            input_mode: p[17].try_into().unwrap(),
        }
    }
}

impl From<[u8; LINK_STATS_SIZE]> for LinkStats {
    fn from(p: [u8; LINK_STATS_SIZE]) -> Self {
        LinkStats {
            //     uplink_rssi_1: bytes_to_float(&p[0..4]),
            //     uplink_rssi_2: bytes_to_float(&p[4..8]),
            //     uplink_link_quality: bytes_to_float(&p[8..12]),
            //     uplink_snr: bytes_to_float(&p[12..16]),
            uplink_rssi_1: p[0],
            uplink_rssi_2: p[1],
            uplink_link_quality: p[2],
            uplink_snr: p[3] as i8,
            uplink_tx_power: p[4],
            ..Default::default() // other fields not used.
        }
    }
}

// impl From<[u8; WAYPOINTS_SIZE]> for [Option<Location>; MAX_WAYPOINTS] {
/// Standalone fn instead of impl due to a Rust restriction.
fn waypoints_from_buf(w: [u8; WAYPOINTS_SIZE]) -> [Option<Location>; MAX_WAYPOINTS] {
    // let mut result = [None; MAX_WAYPOINTS];
    let mut result = [(); MAX_WAYPOINTS].map(|_| Option::<Location>::default());

    for i in 0..MAX_WAYPOINTS {
        let wp_start_i = i * WAYPOINT_SIZE;

        // First bit per waypoint indicates if the Waypoint is used or not.
        // ie if 0, leave as None.
        if w[wp_start_i] == 1 {
            let name =
                std::str::from_utf8(&w[wp_start_i + 1..wp_start_i + 1 + WAYPOINT_MAX_NAME_LEN])
                    .unwrap();

            let coords_start_i = wp_start_i + 1 + WAYPOINT_MAX_NAME_LEN;

            let x = bytes_to_float(&w[coords_start_i..coords_start_i + 4]);
            let y = bytes_to_float(&w[coords_start_i + 4..coords_start_i + 8]);
            let z = bytes_to_float(&w[coords_start_i + 8..coords_start_i + 12]);

            result[i] = Some(Location {
                name: name.to_owned(),
                x,
                y,
                z,
            });
        }
    }

    result
}

#[derive(Debug)]
struct SetServoPositionData {
    servo: ServoWingPosition,
    value: f32,
}

// End code reversed from `quadcopter`.

// todo: Baud cfg?

// pub enum SerialError {};

/// Convert bytes to a float
pub fn bytes_to_float(bytes: &[u8]) -> f32 {
    let bytes: [u8; 4] = bytes.try_into().unwrap();
    f32::from_bits(u32::from_be_bytes(bytes))
}

/// This mirrors that in the Python driver
pub struct SerialInterface {
    /// This `Box<dyn Trait>` is a awk, but part of the `serial_port`. This is for cross-platform
    /// compatibity, since Windows and Linux use different types; the `serial_port` docs are build
    /// for Linux, and don't show the Windows type. (ie `TTYPort vs COMPort`)
    pub serial_port: Option<Box<dyn SerialPort>>,
}

impl SerialInterface {
    pub fn new() -> Self {
        if let Ok(ports) = serialport::available_ports() {
            for port_info in &ports {
                if let SerialPortType::UsbPort(info) = &port_info.port_type {
                    if let Some(sn) = &info.serial_number {
                        if sn == FC_SERIAL_NUMBER {
                            match serialport::new(&port_info.port_name, BAUD)
                                .timeout(Duration::from_millis(TIMEOUT_MILIS))
                                .open()
                            {
                                // }
                                Ok(port) => {
                                    return Self {
                                        serial_port: Some(port),
                                    };
                                }
                                Err(serialport::Error { kind, description }) => {
                                    match kind {
                                        // serialport::ErrorKind::Io(io_kind) => {
                                        //     println!("IO error openin the port");
                                        // }
                                        serialport::ErrorKind::NoDevice => {
                                            println!("No device: {:?}", description);
                                        }
                                        _ => {
                                            println!(
                                                "Error opening the port: {:?} - {:?}",
                                                kind, description
                                            );
                                        }
                                    }
                                }
                            }
                            // .expect("Failed to open serial port");
                        }
                    }
                }
            }
        }

        Self { serial_port: None }
    }
}

fn main() {
    let mut state = State::default();
    state.interface = SerialInterface::new();

    // todo: Separate threads for querying FC, and render?
    render::run(state);
}
