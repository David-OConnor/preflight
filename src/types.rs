//! This module contains types etc that are copy+pasted from the firmware.

use anyleaf_usb::MessageType;
use num_enum::TryFromPrimitive; // Enum from integer

pub const F32_SIZE: usize = 4;

pub const QUATERNION_SIZE: usize = F32_SIZE * 4; // Quaternion (4x4 + altimeter + voltage reading + current reading)
pub const PARAMS_SIZE: usize = 2 * QUATERNION_SIZE + 6 * F32_SIZE + 1 + 4 * 3 + 1 + F32_SIZE * 4;
pub const CONTROLS_SIZE: usize = 19;
pub const LINK_STATS_SIZE: usize = 5; // Only the first 4 fields.

pub const MAX_WAYPOINTS: usize = 30;
pub const WAYPOINT_SIZE: usize = F32_SIZE * 3 + WAYPOINT_MAX_NAME_LEN + 1;
pub const WAYPOINTS_SIZE: usize = MAX_WAYPOINTS * WAYPOINT_SIZE;
pub const SET_SERVO_POSIT_SIZE: usize = 1 + F32_SIZE;
pub const WAYPOINT_MAX_NAME_LEN: usize = 7;
pub const SYS_STATUS_SIZE: usize = 11;
pub const AP_STATUS_SIZE: usize = 0; // todo
pub const SYS_AP_STATUS_SIZE: usize = SYS_STATUS_SIZE + AP_STATUS_SIZE;
pub const CONTROL_MAPPING_QUAD_SIZE: usize = 2; // For quad only atm. Address this.

pub const SET_MOTOR_POWER_SIZE: usize = F32_SIZE * 4;

pub const CONFIG_SIZE: usize = F32_SIZE * 3; // todo: Currently PID only.

pub struct DecodeError {}

// Note that serialize, and for ArmStatus, default, are not part of the firmware

#[derive(Clone, Copy, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum InputModeSwitch {
    /// Acro mode
    Acro = 0,
    /// Command if GPS is present; Attitude if not
    AttitudeCommand = 1,
}

impl Default for InputModeSwitch {
    fn default() -> Self {
        Self::Acro
    }
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, TryFromPrimitive)]
pub enum ArmStatus {
    /// Motors are [pre]disarmed
    Disarmed = 0,
    /// Motors are [pre]armed
    Armed = 1,
}

impl Default for ArmStatus {
    fn default() -> Self {
        Self::Disarmed
    }
}

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive, Debug)]
#[repr(u8)]
pub enum MsgType {
    Params = 0,
    SetMotorDirs = 1,
    ReqParams = 2,
    Ack = 3,
    Controls = 4,
    ReqControls = 5,
    LinkStats = 6,
    ReqLinkStats = 7,
    ArmMotors = 8,
    DisarmMotors = 9,
    StartMotors = 10,
    StopMotors = 11,
    ReqWaypoints = 12,
    Updatewaypoints = 13,
    Waypoints = 14,
    SetServoPosit = 15,
    ReqSysApStatus = 16,
    SysApStatus = 17,
    ReqControlMapping = 18,
    ControlMapping = 19,
    SetMotorPowers = 20,
    SetMotorRpms = 21,
    Config = 22,
    ReqConfig = 23,
    SaveConfig = 24,
}

impl MessageType for MsgType {
    fn val(&self) -> u8 {
        *self as u8
    }

    fn payload_size(&self) -> usize {
        match self {
            Self::Params => PARAMS_SIZE,
            Self::SetMotorDirs => 1, // Packed bits: motors 1-4, R-L. True = CW.
            Self::ReqParams => 0,
            Self::Ack => 0,
            Self::Controls => CONTROLS_SIZE,
            Self::ReqControls => 0,
            Self::LinkStats => LINK_STATS_SIZE,
            Self::ReqLinkStats => 0,
            Self::ArmMotors => 0,
            Self::DisarmMotors => 0,
            Self::StartMotors => 1,
            Self::StopMotors => 1,
            Self::ReqWaypoints => 0,
            Self::Updatewaypoints => 10, // todo?
            Self::Waypoints => WAYPOINTS_SIZE,
            Self::SetServoPosit => SET_SERVO_POSIT_SIZE,
            Self::ReqSysApStatus => 0,
            Self::SysApStatus => SYS_AP_STATUS_SIZE,
            Self::ReqControlMapping => 0,
            Self::ControlMapping => CONTROL_MAPPING_QUAD_SIZE,
            Self::SetMotorPowers => SET_MOTOR_POWER_SIZE,
            Self::SetMotorRpms => SET_MOTOR_POWER_SIZE,
            Self::Config => CONFIG_SIZE,
            Self::ReqConfig => 0,
            Self::SaveConfig => CONFIG_SIZE,
        }
    }
}

#[derive(Default, Clone)]
pub struct PidCoeffs {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

#[derive(Default, Clone)]
pub struct UserConfig {
    pub pid_coeffs: PidCoeffs,
}

impl UserConfig {
    pub fn from_bytes(buf: &[u8]) -> Self {
        let pid_coeffs = PidCoeffs {
            p: f32::from_be_bytes(buf[0..4].try_into().unwrap()),
            i: f32::from_be_bytes(buf[4..8].try_into().unwrap()),
            d: f32::from_be_bytes(buf[8..12].try_into().unwrap()),
        };
        Self {
            pid_coeffs,
            ..Default::default()
        }
    }

    pub fn to_bytes(&self) -> [u8; CONFIG_SIZE] {
        let mut result = [0; CONFIG_SIZE];

        result[..4].clone_from_slice(&self.pid_coeffs.p.to_be_bytes());
        result[4..8].clone_from_slice(&self.pid_coeffs.i.to_be_bytes());
        result[8..12].clone_from_slice(&self.pid_coeffs.d.to_be_bytes());

        result
    }
}

#[derive(Default, Clone)]
pub struct ChannelData {
    /// Aileron, -1. to 1.
    pub roll: f32,
    /// Elevator, -1. to 1.
    pub pitch: f32,
    /// Throttle, 0. to 1., or -1. to 1. depending on if stick auto-centers.
    pub throttle: f32,
    /// Rudder, -1. to 1.
    pub yaw: f32,
    pub arm_status: ArmStatus,
    pub input_mode: InputModeSwitch,
    // pub alt_hold: AltHoldSwitch, // todo
    // todo: Auto-recover commanded, auto-TO/land/RTB, obstacle avoidance etc.
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Rotor {
    R1 = 0,
    R2 = 1,
    R3 = 2,
    R4 = 3,
}

impl Default for Rotor {
    fn default() -> Self {
        Self::R1
    }
}

#[derive(Clone, Copy, Debug, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum RotorPosition {
    FrontLeft = 0,
    FrontRight = 1,
    AftLeft = 2,
    AftRight = 3,
}

impl Default for RotorPosition {
    fn default() -> Self {
        Self::FrontLeft
    }
}

#[derive(Clone, Copy, Debug)]
pub enum ServoWing {
    S1,
    S2,
}

impl Default for ServoWing {
    fn default() -> Self {
        Self::S1
    }
}

#[derive(Clone, Copy, Debug, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum ServoWingPosition {
    Left = 0,
    Right = 1,
}

impl Default for ServoWingPosition {
    fn default() -> Self {
        Self::Left
    }
}

#[derive(Clone, Copy, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum RotationDir {
    Clockwise = 0,
    CounterClockwise = 1,
}

impl Default for RotationDir {
    fn default() -> Self {
        Self::Clockwise
    }
}

#[derive(Clone, Copy, Default)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Clone, Copy, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum AircraftType {
    Quadcopter = 0,
    FixedWing = 1,
}

#[derive(Clone, Default)]
/// https://www.expresslrs.org/2.0/faq/#how-many-channels-does-elrs-support
pub struct LinkStats {
    /// Timestamp these stats were recorded. (TBD format; processed locally; not part of packet from tx).
    pub timestamp: u32,
    /// Uplink - received signal strength antenna 1 (RSSI). RSSI dBm as reported by the RX. Values
    /// vary depending on mode, antenna quality, output power and distance. Ranges from -128 to 0.
    pub uplink_rssi_1: u8,
    /// Uplink - received signal strength antenna 2 (RSSI). Second antenna RSSI, used in diversity mode
    /// (Same range as rssi_1)
    pub uplink_rssi_2: u8,
    /// Uplink - link quality (valid packets). The number of successful packets out of the last
    /// 100 from TX → RX
    pub uplink_link_quality: u8,
    /// Uplink - signal-to-noise ratio. SNR reported by the RX. Value varies mostly by radio chip
    /// and gets lower with distance (once the agc hits its limit)
    pub uplink_snr: i8,
    /// Active antenna for diversity RX (0 - 1)
    pub active_antenna: u8,
    pub rf_mode: u8,
    /// Uplink - transmitting power. (mW?) 50mW reported as 0, as CRSF/OpenTX do not have this option
    pub uplink_tx_power: u8,
    /// Downlink - received signal strength (RSSI). RSSI dBm of telemetry packets received by TX.
    pub downlink_rssi: u8,
    /// Downlink - link quality (valid packets). An LQ indicator of telemetry packets received RX → TX
    /// (0 - 100)
    pub downlink_link_quality: u8,
    /// Downlink - signal-to-noise ratio. SNR reported by the TX for telemetry packets
    pub downlink_snr: i8,
}

#[derive(Default, Clone)]
pub struct Location {
    // Note: unlike Location in the main program, we ommit location type, and use String for name.
    pub name: String,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Clone, Copy, PartialEq, TryFromPrimitive)]
#[repr(u8)] // for USB ser
pub enum SensorStatus {
    Pass = 0,
    Fail = 1,
    /// Either an external sensor not plugged in, or a complete failture, werein it's not recognized.
    NotConnected = 2,
}

impl Default for SensorStatus {
    fn default() -> Self {
        Self::NotConnected
    }
}

#[derive(Clone, Default)]
pub struct SystemStatus {
    pub imu: SensorStatus,
    pub baro: SensorStatus,
    /// The GPS module is connected. Detected on init.
    pub gps: SensorStatus,
    /// The time-of-flight sensor module is connected. Detected on init.
    pub tof: SensorStatus,
    ///  magnetometer is connected. Likely on the same module as GPS. Detected on init.
    pub magnetometer: SensorStatus,
    pub esc_telemetry: SensorStatus,
    pub esc_rpm: SensorStatus,
    pub rf_control_link: SensorStatus,
    pub flash_spi: SensorStatus,
    pub rf_control_fault: bool,
    pub esc_rpm_fault: bool,
}

#[derive(Clone, Copy, PartialEq)]
pub enum AltType {
    /// Above ground level (eg from a TOF sensor)
    Agl,
    /// Mean sea level (eg from GPS or baro)
    Msl,
}

#[repr(u8)] // for USB serialization
#[derive(Clone, Copy)]
pub enum YawAssist {
    Disabled = 0,
    YawAssist = 1,
    /// Automatically adjust roll (rate? angle?) to zero out slip, ie based on rudder inputs.
    RollAssist = 2,
}

impl Default for YawAssist {
    fn default() -> Self {
        Self::Disabled
    }
}

/// Categories of control mode, in regards to which parameters are held fixed.
/// Note that some settings are mutually exclusive.
#[derive(Clone, Default)]
pub struct AutopilotStatus {
    /// Altitude is fixed. (MSL or AGL)
    pub alt_hold: Option<(AltType, f32)>,
    /// Heading is fixed.
    pub hdg_hold: Option<f32>,
    pub yaw_assist: YawAssist,
    pub velocity_vector: Option<(f32, f32)>, // pitch, yaw
    /// Fly direct to a point
    pub direct_to_point: Option<Location>,
    /// The aircraft will fly a fixed profile between sequence points
    pub sequence: bool,
    /// Terrain following mode. Similar to TF radar in a jet. Require a forward-pointing sensor.
    /// todo: Add a forward (or angled) TOF sensor, identical to the downward-facing one?
    pub terrain_following: Option<f32>, // AGL to hold
    /// Take off automatically
    pub takeoff: bool, // todo: takeoff cfg struct[s].
    /// Land automatically
    pub land: Option<LandingCfg>,
    /// Recover to stable, altitude-holding flight. Generally initiated by a "panic button"-style
    /// switch activation
    pub recover: Option<f32>, // value is MSL alt to hold, eg our alt at time of command.
    // #[cfg(feature = "quad")]
    /// Maintain a geographic position and altitude
    pub loiter: Option<Location>,
    // #[cfg(feature = "fixed-wing")]
    /// Orbit over a point on the ground
    pub orbit: Option<Orbit>,
}

// #[cfg(feature = "fixed-wing")]
#[derive(Clone, Default)]
pub struct LandingCfg {
    /// Radians magnetic.
    pub heading: f32,
    /// radians, down from level
    pub glideslope: f32,
    /// Touchdown location, ideally with GPS (requirement?)
    pub touchdown_point: Location,
    /// Groundspeed in m/s
    /// todo: Remove ground_speed in favor of AOA once you figure out how to measure AOA.
    pub ground_speed: f32,
    /// Angle of attack in radians
    /// todo: Include AOA later once you figure out how to measure it.
    pub angle_of_attack: f32,
    /// Altitude to start the flare in AGL. Requires TOF sensor or similar.
    pub flare_alt_agl: f32,
    /// Minimum ground track distance in meters the craft must fly while aligned on the heading
    pub min_ground_track: f32,
    // quad below; fixed-wing above (touchdown pt is both)
    pub descent_starting_alt_msl: f32, // altitude to start the descent in QFE msl.
    pub descent_speed: f32,            // m/s
}

// fixed-wing only
#[derive(Clone)]
pub struct Orbit {
    pub shape: OrbitShape,
    pub center_lat: f32,   // radians
    pub center_lon: f32,   // radians
    pub radius: f32,       // m
    pub ground_speed: f32, // m/s
    pub direction: OrbitDirection,
}

#[derive(Clone, Copy)]
pub enum OrbitDirection {
    Clockwise,
    CounterClockwise,
}

#[cfg(feature = "fixed-wing")]
impl Default for OrbitDirection {
    fn default() -> Self {
        OrbitDirection::Clockwise
    }
}

#[derive(Clone, Copy)]
pub enum OrbitShape {
    Circular,
    Racetrack,
}

#[derive(Default)]
pub struct ControlMappingQuad {
    pub m1: RotorPosition,
    pub m2: RotorPosition,
    pub m3: RotorPosition,
    pub m4: RotorPosition,
    /// It's common to arbitrarily wire motors to the ESC. Reverse each from its
    /// default direction, as required.
    pub m1_reversed: bool,
    pub m2_reversed: bool,
    pub m3_reversed: bool,
    pub m4_reversed: bool,
    pub frontleft_aftright_dir: RotationDir,
}

#[derive(Default)]
pub struct ControlMappingFixedWing {
    pub s1: ServoWingPosition,
    pub s2: ServoWingPosition,
    /// Reverse direction is somewhat arbitrary.
    pub s1_reversed: bool,
    pub s2_reversed: bool,
    /// These represent full scale deflection of the evelons, on a scale of -1 to +1.
    /// We don't use full ARR for max high, since that would be full high the whole time.
    /// Note that the 0 position is fixed; we don't map between these two values; we map between
    /// 0 and each of these.
    /// Note: We currently clamp high and low to be on opposite sides of 0. This may not reflect
    /// control surface reality, but keeps things simple, and should be good enough to start.
    pub servo_high: f32,
    // pub servo_low: f32,
}

#[derive(Clone, Copy, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum BattCellCount {
    S2 = 2,
    S3 = 3,
    S4 = 4,
    S6 = 6,
    S8 = 8,
}

impl Default for BattCellCount {
    fn default() -> Self {
        Self::S4
    }
}

impl BattCellCount {
    pub fn num_cells(&self) -> f32 {
        // float since it interacts with floats.
        match self {
            Self::S2 => 2.,
            Self::S3 => 3.,
            Self::S4 => 4.,
            Self::S6 => 6.,
            Self::S8 => 8.,
        }
    }

    pub fn as_str(&self) -> &str {
        match self {
            Self::S2 => "2S",
            Self::S3 => "3S",
            Self::S4 => "4S",
            Self::S6 => "6S",
            Self::S8 => "8S",
        }
    }
}

/// Represents power levels for the rotors. These map from 0. to 1.; 0% to 100% power.
#[derive(Clone, Default)]
pub struct MotorPower {
    pub front_left: f32,
    pub front_right: f32,
    pub aft_left: f32,
    pub aft_right: f32,
}

/// Represents power levels for the rotors. These map from 0. to 1.; 0% to 100% power.
#[derive(Clone, Default)]
pub struct MotorRpms {
    pub front_left: f32,
    pub front_right: f32,
    pub aft_left: f32,
    pub aft_right: f32,
}

/// We use a f32 on the FC, but send as u16 over USB.
#[derive(Default)]
pub struct RpmReadings {
    pub front_left: Option<u16>,
    pub front_right: Option<u16>,
    pub aft_left: Option<u16>,
    pub aft_right: Option<u16>,
}
