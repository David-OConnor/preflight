use std::{
    convert::TryInto,
    f32::consts::TAU,
    io::{self, Read},
    // mem, thread,
    thread,
    time::{self, Instant},
};

use serialport::{self, SerialPortType};

use lin_alg2::f32::Quaternion;
use types::*;

mod render;
mod types;
mod ui;

const FC_SERIAL_NUMBER: &'static str = "AN";

const BAUD: u32 = 9_600;

// At this interval, in seconds, request new data from the FC.
// todo: Do you want some (or all?) of the read data to be pushed at a regular
// todo interval on request from this program, or pushed at an interval from the FC
// todo without explicitly requesting here?

const READ_INTERVAL: f32 = 0.2;

/// Data passed by the flight controller
struct State {
    pub attitude: Quaternion,
    pub controls: ChannelData,
    pub link_stats: LinkStats,
    pub waypoints: [Option<Location>; MAX_WAYPOINTS],
    pub altimeter_baro: f32,
    pub altimeter_agl: Option<f32>,
    pub batt_v: f32,
    pub current: f32,
    pub last_attitude_update: Instant,
    pub last_controls_update: Instant,
    pub last_link_stats_update: Instant,
    pub aircraft_type: AircraftType,
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
            controls: Default::default(),
            link_stats: Default::default(),
            waypoints,
            altimeter_baro: 0.,
            altimeter_agl: None,
            batt_v: 0.,
            current: 0.,
            last_attitude_update: Instant::now(),
            last_controls_update: Instant::now(),
            last_link_stats_update: Instant::now(),
            aircraft_type: AircraftType::Quadcopter,
        }
    }
}

// // todo: temp static mut
// static mut STATE: State = State {
//     attitude: Quaternion { w: 0., x: 0., y: 0., z: 0. },
//     controls: ChannelData {
//         roll: 0.,
//         pitch: 0.,
//         throttle: 0.,
//         yaw: 0.,
//         arm_status: ArmStatus::Disarmed,
//         input_mode: InputModeSwitch::Acro,
//     },
//     link_stats: LinkStats {
//         timestamp: 0,
//         uplink_rssi_1: 0,
//         uplink_rssi_2: 0,
//         uplink_link_quality: 0,
//         uplink_snr: 0,
//         active_antenna: 0,
//         rf_mode: 0,
//         uplink_tx_power: 0,
//         downlink_rssi: 0,
//         downlink_link_quality: 0,
//         downlink_snr: 0,
//     },
//     waypoints: [None; MAX_WAYPOINTS],
//     altimeter_baro: 0.,
//     altimeter_agl: None,
//     batt_v: 0.,
//     current: 0.,
//     last_attitude_update: Instant::now(),
//     last_controls_update: Instant::now(),
//     last_link_stats_update: Instant::now(),
//     aircraft_type: AircraftType::Quadcopter,
// };

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
struct Fc {
    pub ser: Box<dyn serialport::SerialPort>,
}

impl Fc {
    pub fn new() -> Result<Self, io::Error> {
        if let Ok(ports) = serialport::available_ports() {
            for port_info in &ports {
                if let SerialPortType::UsbPort(info) = &port_info.port_type {
                    if let Some(sn) = &info.serial_number {
                        if sn == FC_SERIAL_NUMBER {
                            let port = serialport::new(&port_info.port_name, BAUD)
                                .open()
                                // todo: Why is the console being spammed with this error?
                                .unwrap();
                            // .expect("Failed to open serial port");

                            return Ok(Self { ser: port });
                        }
                    }
                }
            }
        }

        Err(io::Error::new(
            io::ErrorKind::Other,
            "Unable to connect to the flight controller.",
        ))
    }

    /// Request several types of data from the flight controller over USB serial. Return a struct
    /// containing the data.
    pub fn read_all(&mut self, state: &mut State) -> Result<(), io::Error> {
        let crc_tx_params = calc_crc(
            &CRC_LUT,
            &[MsgType::ReqParams as u8],
            MsgType::ReqParams.payload_size() as u8 + 1,
        );
        let xmit_buf_params = &[MsgType::ReqParams as u8, crc_tx_params];

        // Write the buffer requesting params from the FC.
        self.ser.write(xmit_buf_params)?;

        // Read the params passed by the FC in response.
        let mut rx_buf = [0; PARAMS_SIZE + 2];
        self.ser.read(&mut rx_buf)?;

        // The order (or equivalently indices) of params here must match the FC firmware. Use it
        // as a reference.
        let mut i = 1;

        let attitude_data: [u8; QUATERNION_SIZE] =
            rx_buf[i..QUATERNION_SIZE + i].try_into().unwrap();

        state.attitude = quat_from_buf(&attitude_data);
        i += QUATERNION_SIZE;

        state.altimeter_baro = f32::from_be_bytes(rx_buf[i..F32_BYTES + i].try_into().unwrap());
        i += F32_BYTES;

        state.altimeter_agl = match rx_buf[i] {
            0 => None,
            _ => Some(f32::from_be_bytes(
                rx_buf[i + 1..F32_BYTES + i + 1].try_into().unwrap(),
            )),
        };
        i += F32_BYTES + 1;

        state.batt_v = f32::from_be_bytes(rx_buf[i..F32_BYTES + i].try_into().unwrap());
        i += F32_BYTES;

        state.current = f32::from_be_bytes(rx_buf[i..F32_BYTES + i].try_into().unwrap());
        i += F32_BYTES;

        let crc_tx_controls = calc_crc(
            &CRC_LUT,
            &[MsgType::ReqControls as u8],
            MsgType::ReqControls.payload_size() as u8 + 1,
        );
        let xmit_buf_controls = &[MsgType::ReqControls as u8, crc_tx_controls];

        self.ser.write(xmit_buf_controls)?;

        // let mut rx_buf = [0; CONTROLS_SIZE + 2]; // todo: Bogus leading 1?
        let mut rx_buf = [0; CONTROLS_SIZE + 2];
        self.ser.read(&mut rx_buf)?;

        thread::sleep(time::Duration::from_millis(5)); // todo TS

        let controls: [u8; CONTROLS_SIZE] = rx_buf[1..CONTROLS_SIZE + 1].try_into().unwrap();
        state.controls = controls.into();

        let crc_tx_link_stats = calc_crc(
            &CRC_LUT,
            &[MsgType::ReqLinkStats as u8],
            MsgType::ReqLinkStats.payload_size() as u8 + 1,
        );
        let xmit_buf_link_stats = &[MsgType::ReqLinkStats as u8, crc_tx_link_stats];

        // todo: DRY between these calls
        // self.ser.write(xmit_buf_link_stats)?;  // todo put back

        let mut rx_buf = [0; LINK_STATS_SIZE + 2];
        self.ser.read(&mut rx_buf)?;

        thread::sleep(time::Duration::from_millis(5)); // todo TS

        let link_stats: [u8; LINK_STATS_SIZE] = rx_buf[1..LINK_STATS_SIZE + 1].try_into().unwrap();

        state.link_stats = link_stats.into();

        let crc_waypoints = calc_crc(
            &CRC_LUT,
            &[MsgType::ReqWaypoints as u8],
            MsgType::ReqWaypoints.payload_size() as u8 + 1,
        );
        let xmit_buf_waypoints = &[MsgType::ReqWaypoints as u8, crc_waypoints];

        // self.ser.write(xmit_buf_waypoints)?; // todo put back

        let mut rx_buf = [0; WAYPOINTS_SIZE + 2];
        self.ser.read(&mut rx_buf)?;

        thread::sleep(time::Duration::from_millis(5)); // todo TS

        let mut wp_buf = [0; WAYPOINTS_SIZE];
        wp_buf.clone_from_slice(&rx_buf[1..WAYPOINTS_SIZE + 1]);

        println!("WP BUF: {:?}", wp_buf);

        let waypoints_data = waypoints_from_buf(wp_buf);

        state.waypoints = waypoints_data;

        let payload_size = MsgType::ReqParams.payload_size();
        // let crc_rx_expected = calc_crc(
        //     &CRC_LUT,
        //     &rx_buf[..payload_size + 1],
        //     payload_size as u8 + 1,
        // );

        Ok(())
    }

    pub fn send_arm_command(&mut self) -> Result<(), io::Error> {
        let msg_type = MsgType::ArmMotors;
        let crc = calc_crc(
            &CRC_LUT,
            &[msg_type as u8],
            msg_type.payload_size() as u8 + 1,
        );
        let xmit_buf = &[msg_type as u8, crc];
        self.ser.write(xmit_buf)?;

        Ok(())
    }

    pub fn send_disarm_command(&mut self) -> Result<(), io::Error> {
        let msg_type = MsgType::DisarmMotors;
        let crc = calc_crc(
            &CRC_LUT,
            &[msg_type as u8],
            msg_type.payload_size() as u8 + 1,
        );
        let xmit_buf = &[msg_type as u8, crc];
        self.ser.write(xmit_buf)?;

        Ok(())
    }

    // todo: These are incomplete. you need to pass which motor etc.
    pub fn send_start_motor_command(&mut self, motor: RotorPosition) -> Result<(), io::Error> {
        let msg_type = MsgType::StartMotor;
        let crc = calc_crc(
            &CRC_LUT,
            &[msg_type as u8],
            msg_type.payload_size() as u8 + 1,
        );
        let xmit_buf = &[msg_type as u8, motor as u8, crc];
        self.ser.write(xmit_buf)?;

        Ok(())
    }

    pub fn send_stop_motor_command(&mut self, motor: RotorPosition) -> Result<(), io::Error> {
        let msg_type = MsgType::StopMotor;
        let crc = calc_crc(
            &CRC_LUT,
            &[msg_type as u8],
            msg_type.payload_size() as u8 + 1,
        );
        let xmit_buf = &[msg_type as u8, motor as u8, crc];
        self.ser.write(xmit_buf)?;

        Ok(())
    }

    pub fn send_set_servo_posit_command(
        &mut self,
        servo_posit: ServoWingPosition,
        value: f32,
    ) -> Result<(), io::Error> {
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
        self.ser.write(xmit_buf)?;

        Ok(())
    }

    /// Close the serial port
    pub fn close(&mut self) {}
}

fn main() {
    let fc_ = Fc::new();

    unsafe { render::run() };

    let mut state = State::default();

    if let Ok(mut fc) = fc_ {
        fc.read_all(&mut state);

        thread::sleep(time::Duration::from_millis((READ_INTERVAL * 1_000.) as u64));
        // fc.send_arm_command();

        fc.close();
    } else {
        // Err(io::Error::new(
        //     io::ErrorKind::Other,
        //     "Can't find the flight controller.",
        // ))
    }
}
