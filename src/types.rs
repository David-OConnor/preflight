//! This module contains types etc that are copy+pasted from the firmware.

pub const F32_SIZE: usize = 4;

pub const CRC_POLY: u8 = 0xab;
pub const CRC_LUT: [u8; 256] = crc_init(CRC_POLY);

pub const QUATERNION_SIZE: usize = F32_SIZE * 4; // Quaternion (4x4 + altimeter + voltage reading + current reading)
pub const PARAMS_SIZE: usize = 2 * QUATERNION_SIZE + 4 * F32_SIZE + 1; //
pub const CONTROLS_SIZE: usize = 18;
pub const LINK_STATS_SIZE: usize = 5; // Only the first 4 fields.

pub const MAX_WAYPOINTS: usize = 30;
pub const WAYPOINT_SIZE: usize = F32_SIZE * 3 + WAYPOINT_MAX_NAME_LEN + 1;
pub const WAYPOINTS_SIZE: usize = MAX_WAYPOINTS * WAYPOINT_SIZE;
pub const SET_SERVO_POSIT_SIZE: usize = 1 + F32_SIZE;
pub const WAYPOINT_MAX_NAME_LEN: usize = 7;
pub const SYS_STATUS_SIZE: usize = 7; // Sensor status (u8) * 7
pub const AP_STATUS_SIZE: usize = 0; // todo
pub const SYS_AP_STATUS_SIZE: usize = SYS_STATUS_SIZE + AP_STATUS_SIZE;

// Packet sizes are payload size + 2. Additional data are message type, and CRC.
pub const PARAMS_PACKET_SIZE: usize = PARAMS_SIZE + 2;
pub const CONTROLS_PACKET_SIZE: usize = CONTROLS_SIZE + 2;
pub const LINK_STATS_PACKET_SIZE: usize = LINK_STATS_SIZE + 2;
pub const WAYPOINTS_PACKET_SIZE: usize = WAYPOINTS_SIZE + 2;
pub const SYS_AP_STATUS_PACKET_SIZE: usize = SYS_AP_STATUS_SIZE + 2;

pub struct DecodeError {}

// Time between querying the FC for readings, in ms.
pub const REFRESH_INTERVAL: u32 = 50;

use num_enum::TryFromPrimitive; // Enum from integer

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

#[derive(Clone, Copy, Eq, PartialEq, TryFromPrimitive)]
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
    StartMotor = 10,
    StopMotor = 11,
    ReqWaypoints = 12,
    Updatewaypoints = 13,
    Waypoints = 14,
    SetServoPosit = 15,
    ReqSysApStatus = 16,
    SysApStatus = 17,
}

impl MsgType {
    pub fn payload_size(&self) -> usize {
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
            Self::StartMotor => 1,
            Self::StopMotor => 1,
            Self::ReqWaypoints => 0,
            Self::Updatewaypoints => 10, // todo?
            Self::Waypoints => WAYPOINTS_SIZE,
            Self::SetServoPosit => SET_SERVO_POSIT_SIZE,
            Self::ReqSysApStatus => 0,
            Self::SysApStatus => SYS_AP_STATUS_SIZE,
        }
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

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum RotorPosition {
    FrontLeft = 0,
    FrontRight = 1,
    AftLeft = 2,
    AftRight = 3,
}

#[derive(Clone, Copy, Debug)]
pub enum ServoWing {
    S1,
    S2,
}

#[derive(Clone, Copy, Debug)]
#[repr(u8)]
pub enum ServoWingPosition {
    Left = 0,
    Right = 1,
}

pub const fn crc_init(poly: u8) -> [u8; 256] {
    let mut lut = [0; 256];

    let mut i = 0;
    while i < 256 {
        // Can't use for loops in const fns
        let mut crc = i as u8;

        let mut j = 0;
        while j < 8 {
            crc = (crc << 1) ^ (if (crc & 0x80) > 0 { poly } else { 0 });
            j += 1;
        }
        lut[i] = crc;

        i += 1;
    }

    lut
}

/// CRC8 using a specific poly, includes all bytes from type (buffer[2]) to end of payload.
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
pub fn calc_crc(lut: &[u8; 256], data: &[u8], mut size: u8) -> u8 {
    let mut crc = 0;
    let mut i = 0;

    while size > 0 {
        size -= 1;
        crc = lut[(crc ^ data[i]) as usize];
        i += 1;
    }
    crc
}

#[derive(Clone, Copy, Default)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Clone, Copy, PartialEq)]
pub enum AircraftType {
    Quadcopter,
    FlyingWing,
}

#[derive(Clone, Default)]
/// https://www.expresslrs.org/2.0/faq/#how-many-channels-does-elrs-support
pub struct LinkStats {
    /// Timestamp these stats were recorded. (TBD format; processed locally; not part of packet from tx).
    pub timestamp: u32,
    /// Uplink - received signal strength antenna 1 (RSSI). RSSI dBm as reported by the RX. Values
    /// vary depending on mode, antenna quality, output power and distance. Ranges from -128 to 0.
    pub uplink_rssi_1: u8,
    /// Uplink - received signal strength antenna 2 (RSSI).  	Second antenna RSSI, used in diversity mode
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
    /// Downlink - signal-to-noise ratio. 	SNR reported by the TX for telemetry packets
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
