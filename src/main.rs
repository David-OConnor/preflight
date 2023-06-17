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

use pc_interface_shared::{self, send_cmd, send_payload};

use anyleaf_usb::{CRC_LEN, PAYLOAD_START_I};

use lin_alg2::f32::Quaternion;
use types::*;

mod render;
mod types;
mod ui;

// At this interval, in seconds, request new data from the FC.
// todo: Do you want some (or all?) of the read data to be pushed at a regular
// todo interval on request from this program, or pushed at an interval from the FC
// todo without explicitly requesting here?
// todo: Also: multiple intervals for different sorts of data, eg update
// todo attitude at a higher rate than other things.
const READ_INTERVAL: f32 = 0.05;
pub const READ_INTERVAL_MS: u128 = (READ_INTERVAL * 1_000.) as u128;

/// Data passed by the flight controller
pub struct State {
    pub common: pc_interface_shared::StateCommon,
    pub attitude: Quaternion,
    pub attitude_commanded: Quaternion,
    pub controls: Option<ChannelData>,
    pub link_stats: LinkStats,
    pub waypoints: [Option<Location>; MAX_WAYPOINTS],
    pub altitude_baro: f32,
    pub pressure_static: f32,
    pub temp_baro: f32,
    pub altitude_agl: Option<f32>,
    pub batt_v: f32,
    pub current: f32,
    pub lat: Option<f32>,
    pub lon: Option<f32>,
    // todo: aftleft etc, or 1-4?
    pub rpm_readings: RpmReadings,
    pub autopilot_status: AutopilotStatus,
    pub last_attitude_update: Instant,
    pub last_controls_update: Instant,
    pub last_link_stats_update: Instant,
    pub system_status: SystemStatus,
    pub aircraft_type: AircraftType,
    // todo: Use an enum for control mapping.
    pub control_mapping_quad: ControlMappingQuad,
    pub control_mapping_fixed_wing: ControlMappingFixedWing,
    // todo: ui_state field?
    pub editing_motor_mapping: bool,
    pub batt_cell_count: BattCellCount,
    current_pwr: MotorPower,
    pub pwr_commanded_from_ui: MotorPower,
    pub rpms_commanded_from_ui: MotorRpms,
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
            common: Default::default(),
            attitude: Quaternion::new_identity(),
            attitude_commanded: Quaternion::new_identity(),
            controls: Default::default(),
            link_stats: Default::default(),
            waypoints,
            altitude_baro: 0.,
            pressure_static: 0.,
            temp_baro: 0.,
            altitude_agl: None,
            batt_v: 0.,
            current: 0.,
            lat: None,
            lon: None,
            rpm_readings: Default::default(),
            autopilot_status: Default::default(),
            last_attitude_update: Instant::now(),
            last_controls_update: Instant::now(),
            last_link_stats_update: Instant::now(),
            system_status: Default::default(),
            aircraft_type: AircraftType::Quadcopter,
            control_mapping_quad: Default::default(),
            control_mapping_fixed_wing: Default::default(),
            editing_motor_mapping: false,
            batt_cell_count: Default::default(),
            current_pwr: Default::default(),
            pwr_commanded_from_ui: Default::default(),
            rpms_commanded_from_ui: Default::default(),
        }
    }
}

impl State {
    /// Read parameters, such as attitude and altitutde
    // pub fn read_params(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_params(&mut self) -> Result<(), io::Error> {
        let port = self.common.get_port()?;

        send_cmd::<MsgType>(MsgType::ReqParams, port)?;

        // Read the params passed by the FC in response.
        let mut rx_buf = [0; PARAMS_SIZE + PAYLOAD_START_I + CRC_LEN];
        port.read_exact(&mut rx_buf)?;

        // The order (or equivalently indices) of params here must match the FC firmware. Use it
        // as a reference.
        let mut i = PAYLOAD_START_I;

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

        self.pressure_static = f32::from_be_bytes(rx_buf[i..F32_SIZE + i].try_into().unwrap());
        i += F32_SIZE;

        self.temp_baro = f32::from_be_bytes(rx_buf[i..F32_SIZE + i].try_into().unwrap());
        i += F32_SIZE;

        self.rpm_readings.front_left = match rx_buf[i] {
            1 => Some(u16::from_be_bytes(rx_buf[i + 1..i + 3].try_into().unwrap())),
            _ => None,
        };
        i += 3;

        self.rpm_readings.aft_left = match rx_buf[i] {
            1 => Some(u16::from_be_bytes(rx_buf[i + 1..i + 3].try_into().unwrap())),

            _ => None,
        };
        i += 3;

        self.rpm_readings.front_right = match rx_buf[i] {
            1 => Some(u16::from_be_bytes(rx_buf[i + 1..i + 3].try_into().unwrap())),
            _ => None,
        };
        i += 3;

        self.rpm_readings.aft_right = match rx_buf[i] {
            1 => Some(u16::from_be_bytes(rx_buf[i + 1..i + 3].try_into().unwrap())),
            _ => None,
        };
        i += 3;

        self.aircraft_type = rx_buf[i].try_into().unwrap();
        i += 1;

        self.current_pwr.front_left = f32::from_be_bytes(rx_buf[i..i + 4].try_into().unwrap());
        i += 4;
        self.current_pwr.aft_left = f32::from_be_bytes(rx_buf[i..i + 4].try_into().unwrap());
        i += 4;
        self.current_pwr.front_right = f32::from_be_bytes(rx_buf[i..i + 4].try_into().unwrap());
        i += 4;
        self.current_pwr.aft_right = f32::from_be_bytes(rx_buf[i..i + 4].try_into().unwrap());
        i += 4;

        Ok(())
    }

    /// Read system status, and autopilot status
    // pub fn read_controls(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_sys_ap_status(&mut self) -> Result<(), io::Error> {
        // todo: DRY with port. Trouble passing it as a param due to box<dyn
        let port = self.common.get_port()?;

        send_cmd::<MsgType>(MsgType::ReqSysApStatus, port)?;

        // todo: Just sys status for now; do AP too.
        let mut rx_buf = [0; SYS_AP_STATUS_SIZE + PAYLOAD_START_I + CRC_LEN];
        port.read_exact(&mut rx_buf)?;

        let sys_status: [u8; SYS_AP_STATUS_SIZE] = rx_buf
            [PAYLOAD_START_I..SYS_AP_STATUS_SIZE + PAYLOAD_START_I]
            .try_into()
            .unwrap();
        self.system_status = sys_status.into();

        Ok(())
    }

    /// Read control channel data.
    // pub fn read_controls(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_controls(&mut self) -> Result<(), io::Error> {
        let port = self.common.get_port()?;

        send_cmd::<MsgType>(MsgType::ReqControls, port)?;

        let mut rx_buf = [0; CONTROLS_SIZE + PAYLOAD_START_I + CRC_LEN];
        port.read_exact(&mut rx_buf)?;

        let controls: [u8; CONTROLS_SIZE] = rx_buf
            [PAYLOAD_START_I..CONTROLS_SIZE + PAYLOAD_START_I]
            .try_into()
            .unwrap();
        self.controls = controls_from_buf(controls);

        Ok(())
    }

    /// Read controller link stats data.
    // pub fn read_link_stats(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_link_stats(&mut self) -> Result<(), io::Error> {
        // todo: DRY with port. Trouble passing it as a param due to box<dyn
        let port = self.common.get_port()?;

        send_cmd::<MsgType>(MsgType::ReqLinkStats, port)?;

        let mut rx_buf = [0; LINK_STATS_SIZE + PAYLOAD_START_I + CRC_LEN];
        port.read_exact(&mut rx_buf)?;

        let link_stats: [u8; LINK_STATS_SIZE] = rx_buf
            [PAYLOAD_START_I..LINK_STATS_SIZE + PAYLOAD_START_I]
            .try_into()
            .unwrap();

        self.link_stats = link_stats.into();

        Ok(())
    }

    /// Read waypoints data from the flight controller.
    // pub fn read_waypoints(&mut self, port: &Box<dyn SerialPort>) -> Result<(), io::Error> {
    pub fn read_waypoints(&mut self) -> Result<(), io::Error> {
        let port = self.common.get_port()?;

        send_cmd::<MsgType>(MsgType::ReqWaypoints, port)?;

        let mut rx_buf = [0; WAYPOINTS_SIZE + PAYLOAD_START_I + CRC_LEN];
        port.read_exact(&mut rx_buf)?;

        let mut wp_buf = [0; WAYPOINTS_SIZE];
        wp_buf.clone_from_slice(&rx_buf[PAYLOAD_START_I..WAYPOINTS_SIZE + PAYLOAD_START_I]);

        let waypoints_data = waypoints_from_buf(wp_buf);

        self.waypoints = waypoints_data;

        // todo: Lat, Lon

        Ok(())
    }

    /// Read motor or servo control mapping from the FC.
    pub fn read_control_mapping(&mut self) -> Result<(), io::Error> {
        let port = self.common.get_port()?;

        send_cmd::<MsgType>(MsgType::ReqControlMapping, port)?;

        let mut rx_buf = [0; CONTROL_MAPPING_QUAD_SIZE + PAYLOAD_START_I + CRC_LEN];
        port.read_exact(&mut rx_buf)?;

        let mut buf = [0; CONTROL_MAPPING_QUAD_SIZE];
        buf.clone_from_slice(&rx_buf[PAYLOAD_START_I..CONTROL_MAPPING_QUAD_SIZE + PAYLOAD_START_I]);

        let control_mapping = buf.into();

        // todo: Fixed wing A/R.
        self.control_mapping_quad = control_mapping;

        Ok(())
    }

    /// Request several types of data from the flight controller over USB serial. Return a struct
    /// containing the data.
    pub fn read_all(&mut self) -> Result<(), io::Error> {
        // let port = self.common.get_port()?;

        self.read_params()?;
        self.read_sys_ap_status()?;
        self.read_controls()?;
        self.read_link_stats()?;
        // todo: Put back; issue with it to correct.
        // self.read_waypoints()?;

        // todo: Don't read some things like control mapping each time.
        self.read_control_mapping()?;

        Ok(())
    }

    pub fn send_arm_command(&mut self) -> Result<(), io::Error> {
        send_cmd::<MsgType>(MsgType::ArmMotors, self.common.get_port()?)
    }

    pub fn send_disarm_command(&mut self) -> Result<(), io::Error> {
        send_cmd::<MsgType>(MsgType::DisarmMotors, self.common.get_port()?)
    }

    // todo: These are incomplete. you need to pass which motor etc.
    pub fn send_start_motor_command(&mut self, motor: RotorPosition) -> Result<(), io::Error> {
        send_payload::<MsgType, { 1 }>(
            MsgType::StartMotors,
            &[motor as u8],
            self.common.get_port()?,
        )
    }

    pub fn send_stop_motor_command(&mut self, motor: RotorPosition) -> Result<(), io::Error> {
        send_payload::<MsgType, { 1 }>(MsgType::StopMotors, &[motor as u8], self.common.get_port()?)
    }

    pub fn send_set_servo_posit_command(
        &mut self,
        servo_posit: ServoWingPosition,
        value: f32,
    ) -> Result<(), io::Error> {
        let v = value.to_be_bytes();
        let payload = [servo_posit as u8, v[0], v[1], v[2], v[3]];

        send_payload::<MsgType, { 4 }>(MsgType::SetServoPosit, &payload, self.common.get_port()?)
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
        SystemStatus {
            // todo: Getting some errors here.
            imu: p[0].try_into().unwrap_or_default(),
            baro: p[1].try_into().unwrap_or_default(),
            gps: p[2].try_into().unwrap_or_default(),
            tof: p[3].try_into().unwrap_or_default(),
            magnetometer: p[4].try_into().unwrap_or_default(),
            esc_telemetry: p[5].try_into().unwrap_or_default(),
            esc_rpm: p[6].try_into().unwrap_or_default(),
            rf_control_link: p[7].try_into().unwrap_or_default(),
            flash_spi: p[8].try_into().unwrap_or_default(),
            rf_control_fault: p[9] != 0,
            esc_rpm_fault: p[10] != 0,
        }
    }
}

/// 19 f32s x 4 = 76. In the order we have defined in the struct.
pub fn controls_from_buf(p: [u8; CONTROLS_SIZE]) -> Option<ChannelData> {
    if p[0] == 0 {
        return None;
    }

    Some(ChannelData {
        pitch: bytes_to_float(&p[1..5]),
        roll: bytes_to_float(&p[5..9]),
        yaw: bytes_to_float(&p[9..13]),
        throttle: bytes_to_float(&p[13..17]),

        arm_status: p[17].try_into().unwrap(),
        input_mode: p[18].try_into().unwrap(),
    })
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

// todo: Fixed wing too
impl From<[u8; CONTROL_MAPPING_QUAD_SIZE]> for ControlMappingQuad {
    fn from(p: [u8; CONTROL_MAPPING_QUAD_SIZE]) -> Self {
        ControlMappingQuad {
            m1: (p[0] & 0b11).try_into().unwrap(),
            m2: ((p[0] >> 2) & 0b11).try_into().unwrap(),
            m3: ((p[0] >> 4) & 0b11).try_into().unwrap(),
            m4: ((p[0] >> 6) & 0b11).try_into().unwrap(),
            m1_reversed: (p[1] & 1) != 0,
            m2_reversed: ((p[1] >> 1) & 1) != 0,
            m3_reversed: ((p[1] >> 2) & 1) != 0,
            m4_reversed: ((p[1] >> 3) & 1) != 0,
            frontleft_aftright_dir: ((p[1] >> 4) & 1).try_into().unwrap(),
        }
    }
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

fn main() {
    let mut state = State::default();

    render::run(state);
}
