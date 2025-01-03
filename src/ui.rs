//! This module contains code for the GUI

use std::f32::consts::TAU;

use egui::{
    self, Button, Color32, ComboBox, CursorIcon::Default, ImageData::Color, ProgressBar, RichText,
    Ui,
};

use graphics::{EngineUpdates, Scene};

use pc_interface_shared::{send_cmd, send_payload, ConnectionStatus};

use anyleaf_usb::{CRC_LEN, PAYLOAD_START_I};

use crate::{
    types::{
        AircraftType, AltType, ArmStatus, LinkStats, MotorPower, MotorRpms, MsgType, SensorStatus,
        SystemStatus, YawAssist, CONFIG_SIZE,
    },
    BattCellCount, ChannelData, RotorPosition, State,
};

// pub const UI_PANEL_SIZE: f32 = 1_600.;

const ITEM_SPACING_X: f32 = 10.;
const ITEM_SPACING_Y: f32 = 12.;
const SPACE_BETWEEN_SECTIONS: f32 = 30.;
const SPACING_HORIZONTAL: f32 = 26.;
const SPACING_HORIZONTAL_TIGHT: f32 = 16.;
const CONTROL_BAR_WIDTH: f32 = 200.; // eg pitch, roll, yaw, throttle control.
const BATT_LIFE_WIDTH: f32 = 200.; // eg pitch, roll, yaw, throttle control.
const MOTOR_MAPPING_DROPDOWN_WIDTH: f32 = 90.;
const MIN_UI_WIDTH: f32 = 800.;
const SLIDER_WIDTH: f32 = 200.;
const FLOAT_EDIT_WIDTH: f32 = 60.;

// todo: PUt this elsewhere
pub const PAYLOAD_SIZE_CONFIG: usize = crate::CONFIG_SIZE + PAYLOAD_START_I + CRC_LEN;

impl SensorStatus {
    // Note Not included in Corvus
    fn as_str(&self) -> &str {
        match self {
            Self::Pass => "Pass",
            Self::Fail => "Fail",
            Self::NotConnected => "Not connected",
        }
    }

    fn as_color(&self) -> Color32 {
        match self {
            Self::Pass => Color32::LIGHT_GREEN,
            Self::Fail => Color32::LIGHT_RED,
            Self::NotConnected => Color32::GOLD,
        }
    }

    /// For systems that are required, or are very important to fly.
    fn as_color_critical_system(&self) -> Color32 {
        match self {
            Self::Pass => Color32::LIGHT_GREEN,
            Self::Fail => Color32::LIGHT_RED,
            Self::NotConnected => Color32::LIGHT_RED,
        }
    }
}

// todo: Fixed wing arm status as well
impl ArmStatus {
    fn as_str(&self) -> &str {
        match self {
            Self::Armed => "Armed",
            Self::Disarmed => "Disarmed",
        }
    }

    fn as_color(&self) -> Color32 {
        match self {
            Self::Armed => Color32::YELLOW,
            Self::Disarmed => Color32::GREEN,
        }
    }
}

impl RotorPosition {
    fn as_str(&self) -> &str {
        match self {
            Self::FrontLeft => "Front left",
            Self::FrontRight => "Front right",
            Self::AftLeft => "Aft left",
            Self::AftRight => "Aft right",
        }
    }
}

fn link_qual_to_color(lq: u8) -> Color32 {
    match lq {
        95..=100 => Color32::LIGHT_GREEN,
        70..=94 => Color32::LIGHT_YELLOW,
        _ => Color32::LIGHT_RED,
    }
}

/// Uses the raw value provided over CRSF, ie positive values.
/// These are for ground testing, with the drone next to the radio, and
/// antennas plugged in.
fn rssi_to_color(rssi: u8) -> Color32 {
    match rssi {
        0..=50 => Color32::LIGHT_GREEN,
        51..=70 => Color32::LIGHT_YELLOW,
        _ => Color32::LIGHT_RED,
    }
}

fn test_val_to_color(v: bool) -> Color32 {
    if v {
        Color32::LIGHT_RED
    } else {
        Color32::LIGHT_GRAY
    }
}

fn motor_dir_format(v: bool) -> String {
    if v { "Reversed" } else { "Normal" }.to_owned()
}

/// See Corvus, `ElrsTxPower`
fn tx_pwr_from_val(val: u8) -> String {
    match val {
        1 => "10mW",
        2 => "25mW",
        8 => "50mW",
        3 => "100mW",
        7 => "250mW",
        _ => "(Unknown)",
    }
        .to_owned()
}

enum LatLon {
    Lat,
    Lon,
}

/// Convert radians to Degrees, Minutes, Seconds; formatted.
fn rad_to_dms(rad: f32, lat_lon: LatLon) -> String {
    let deg = rad * 360. / TAU;

    let deg_whole = deg as u16;
    let mins = (deg - (deg_whole as f32)) * 60.;

    let mins_whole = mins as u8;
    let secs = (mins - (mins_whole as f32)) * 60.;

    // todo: We don't need floats on deg here right?
    let (n_s, deg_whole) = match lat_lon {
        LatLon::Lat => {
            if deg_whole > 90 {
                ("N", deg_whole - 90)
            } else {
                ("S", 90 - deg_whole)
            }
        }
        LatLon::Lon => {
            if deg_whole > 90 {
                ("E", deg_whole - 180)
            } else {
                ("W", 180 - deg_whole)
            }
        }
    };

    format!("{deg_whole}° {mins_whole}' {:.2}\" {n_s}", secs)
}

// fn text_edit_float(val: &mut f32, _default: f64, ui: &mut Ui) {
//     let mut entry = val.to_string();
//
//     let response = ui.add(egui::TextEdit::singleline(&mut entry).desired_width(FLOAT_EDIT_WIDTH));
//     if response.changed() {
//         *val = entry.parse::<f32>().unwrap_or(0.001);
//     }
// }

fn text_edit_int(val: &mut u32, ui: &mut Ui) {
    let mut entry = val.to_string();

    let response = ui.add(egui::TextEdit::singleline(&mut entry).desired_width(FLOAT_EDIT_WIDTH));
    if response.changed() {
        *val = entry.parse::<u32>().unwrap_or(0);
    }
}

fn format_rpm(rpm: Option<u16>) -> String {
    match rpm {
        Some(r) => r.to_string(),
        None => "(no data)".to_owned(),
    }
}

fn format_ap_bool(v: bool) -> String {
    if v {
        "Enabled".to_owned()
    } else {
        "Off".to_owned()
    }
}

/// Add a sensor status indicator.
fn add_sensor_status(label: &str, val: SensorStatus, ui: &mut Ui, critical: bool) {
    let color = if critical {
        val.as_color_critical_system()
    } else {
        val.as_color()
    };

    ui.vertical(|ui| {
        ui.label(label);
        ui.label(RichText::new(val.as_str()).color(color));
    });
    ui.add_space(SPACING_HORIZONTAL);
}

/// Add a progress bar displaying the current position of a control.
fn add_control_disp(mut val: f32, text: &str, throttle: bool, ui: &mut Ui) {
    ui.label(text);
    // convert from -1. to 1. to 0. to 1.
    let v = if throttle { val / 2. + 0.5 } else { val };
    let bar = ProgressBar::new(v).desired_width(CONTROL_BAR_WIDTH);
    // let range = if throttle {
    //     0.0..=0.1
    // } else {
    //     -1.0..=1.0
    // };

    // let v2 = &mut val;
    // let slider = egui::Slider::new(v2, range);
    // ui.add(slider);
    ui.add(bar);
}

fn add_link_stats(status: SensorStatus, link_stats: &LinkStats, ui: &mut Ui) {
    // todo: convert things like tx power to actual power disp
    if status != SensorStatus::Pass {
        ui.heading(RichText::new("(Radio control link not connected)").color(Color32::GOLD));
        return;
    }

    ui.horizontal(|ui| {
        ui.vertical(|ui| {
            // todo: (130+RSSI_dBm)/130*100?
            //  dbm = lambda v: v * 130/100 - 130
            ui.label("Uplink RSSI 1 (dBm)");
            let val = &link_stats.uplink_rssi_1;
            ui.label(RichText::new(format!("-{}", val)).color(rssi_to_color(*val)));
        });
        ui.add_space(SPACING_HORIZONTAL_TIGHT);

        ui.vertical(|ui| {
            ui.label("Uplink RSSI 2");
            let val = &link_stats.uplink_rssi_2;
            ui.label(RichText::new(format!("-{}", val)).color(rssi_to_color(*val)));
        });
        ui.add_space(SPACING_HORIZONTAL_TIGHT);

        ui.vertical(|ui| {
            ui.label("Link qual");
            let val = &link_stats.uplink_link_quality;
            ui.label(RichText::new(val.to_string()).color(link_qual_to_color(*val)));
        });
        ui.add_space(SPACING_HORIZONTAL_TIGHT);

        ui.vertical(|ui| {
            ui.label("Uplink SNR");
            ui.label(&link_stats.uplink_snr.to_string());
        });
        ui.add_space(SPACING_HORIZONTAL_TIGHT);

        // ui.vertical(|ui| {
        //     ui.label("Active antenna");
        //     ui.label(&link_stats.active_antenna.to_string());
        // });
        // ui.add_space(SPACING_HORIZONTAL_TIGHT);

        ui.vertical(|ui| {
            ui.label("RF mode");
            ui.label(&link_stats.rf_mode.to_string());
        });
        ui.add_space(SPACING_HORIZONTAL);

        ui.vertical(|ui| {
            ui.label("Uplink Tx pwr");
            ui.label(&tx_pwr_from_val(link_stats.uplink_tx_power));
        });
        ui.add_space(SPACING_HORIZONTAL_TIGHT);

        ui.vertical(|ui| {
            ui.label("Downlink RSSI");
            ui.label(&link_stats.downlink_rssi.to_string());
        });
        ui.add_space(SPACING_HORIZONTAL_TIGHT);

        ui.vertical(|ui| {
            ui.label("Downlink link qual");
            let val = &link_stats.downlink_link_quality;
            ui.label(RichText::new(val.to_string()).color(link_qual_to_color(*val)));
        });
        ui.add_space(SPACING_HORIZONTAL_TIGHT);

        ui.vertical(|ui| {
            ui.label("Downlink SNR");
            ui.label(&link_stats.downlink_snr.to_string());
        });
    });
}

fn add_control_data(status: SensorStatus, ch_data: &Option<ChannelData>, ui: &mut Ui) {
    ui.vertical(|ui| {
        ui.heading(RichText::new("Control commands").color(Color32::WHITE));

        if status == SensorStatus::Pass && ch_data.is_some() {
            let controls = ch_data.as_ref().unwrap();

            ui.horizontal(|ui| {
                add_control_disp(controls.pitch, "Pitch", true, ui);
                add_control_disp(controls.roll, "Roll", true, ui);
            });
            ui.horizontal(|ui| {
                add_control_disp(controls.yaw, "Yaw", true, ui);
                add_control_disp(controls.throttle, "Throttle", false, ui);
            });

            // todo Important: This isn't the actual arm status state! It's the control input!
            // todo: Potentially misleading.
            ui.vertical(|ui| {
                ui.label("Motor arm switch");
                let val = controls.arm_status;
                ui.label(RichText::new(val.as_str()).color(val.as_color()));
            });
        } else {
            ui.heading(RichText::new("(Radio control link not connected)").color(Color32::GOLD));
        }
    });
}

fn add_not_connected_page(ui: &mut Ui) {
    let text = "No data received from the flight controller";
    ui.heading(RichText::new(text).color(Color32::GOLD).size(60.));
    ui.add_space(300.); // So not aligned to bottom of the window.
}

fn add_motor_commands(ui: &mut Ui, power: &mut MotorPower, rpms: &mut MotorRpms) {
    return;
    ui.heading(
        "Send commands to motors. ⚠️Warning: Causes motors to spin. Disconnect all \
                propellers before using.⚠️",
    );

    for (motor_pwr, label) in [
        (&mut power.front_left, "Front left power"),
        (&mut power.front_right, "Front right power"),
        (&mut power.aft_left, "Aft left power"),
        (&mut power.aft_right, "Aft right power"),
    ]
        .into_iter()
    {
        ui.add(
            // Offsets are to avoid gimball lock.
            egui::Slider::from_get_set(0.0..=1.0, |v| {
                if let Some(v_) = v {
                    *motor_pwr = v_ as f32;
                }

                *motor_pwr as f64
            })
                .text(label),
        );
    }
}

/// From Corvus
const BATT_LUT: [(f32, f32); 21] = [
    (3.27, 0.),
    (3.61, 0.05),
    (3.69, 0.1),
    (3.71, 0.15),
    (3.73, 0.2),
    (3.75, 0.25),
    (3.77, 0.3),
    (3.79, 0.35),
    (3.8, 0.4),
    (3.82, 0.45),
    (3.84, 0.5),
    (3.85, 0.55),
    (3.87, 0.6),
    (3.91, 0.65),
    (3.95, 0.7),
    (3.98, 0.75),
    (4.02, 0.8),
    (4.08, 0.85),
    (4.11, 0.9),
    (4.15, 0.95),
    (4.20, 1.),
];

/// Returns an estimate of battery life, with 0. being empty, and 1. being full.
/// Input is in volts.
/// From Corvus.
/// [Reference with table](https://blog.ampow.com/lipo-voltage-chart/)
fn batt_left_from_v(v: f32, cell_count: BattCellCount) -> f32 {
    let per_cell = v / cell_count.num_cells();
    // todo: Temp. Refine this.
    // let empty_v = 3.5;
    // let full_v = 4.2;

    let mut i = 0;

    // todo: Refactor to use an if/else cascade
    if per_cell > BATT_LUT[1].0 {
        i = 1;
    }
    if per_cell > BATT_LUT[2].0 {
        i = 2;
    }
    if per_cell > BATT_LUT[3].0 {
        i = 3;
    }
    if per_cell > BATT_LUT[4].0 {
        i = 4;
    }
    if per_cell > BATT_LUT[5].0 {
        i = 5;
    }
    if per_cell > BATT_LUT[6].0 {
        i = 6;
    }
    if per_cell > BATT_LUT[7].0 {
        i = 7;
    }
    if per_cell > BATT_LUT[8].0 {
        i = 8;
    }
    if per_cell > BATT_LUT[9].0 {
        i = 9;
    }
    if per_cell > BATT_LUT[10].0 {
        i = 10;
    }
    if per_cell > BATT_LUT[11].0 {
        i = 11;
    }
    if per_cell > BATT_LUT[12].0 {
        i = 12;
    }
    if per_cell > BATT_LUT[13].0 {
        i = 13;
    }
    if per_cell > BATT_LUT[14].0 {
        i = 14;
    }
    if per_cell > BATT_LUT[15].0 {
        i = 15;
    }
    if per_cell > BATT_LUT[16].0 {
        i = 16;
    }
    if per_cell > BATT_LUT[17].0 {
        i = 17;
    }
    if per_cell > BATT_LUT[18].0 {
        i = 18;
    }
    if per_cell > BATT_LUT[19].0 {
        i = 19;
    }
    // if per_cell > BATT_LUT[20].0 {
    //     i = 20;
    // }

    // todo. Not linear! Just for now.

    let port_through = (per_cell - BATT_LUT[i].0) / (BATT_LUT[i + 1].0 - BATT_LUT[i].0);
    port_through * (BATT_LUT[i + 1].1 - BATT_LUT[i].1) + BATT_LUT[i].1
}

/// Charge is on a scale of 0. to 1.
fn batt_charge_to_color(charge: f32) -> Color32 {
    let min_color = (1., 0., 0.);

    let max_color = (0., 1., 0.);

    // We are cheating a bit here, knowing our results are both pure channels...

    let r = ((1. - charge) * 255.) as u8;
    let g = (charge * 255.) as u8;
    let b = 0;

    Color32::from_rgb(r, g, b)
}

fn add_system_status(ui: &mut Ui, system_status: &SystemStatus) {
    ui.heading(RichText::new("System status").color(Color32::WHITE));
    // ui.heading("System status"); // todo: Get this from FC; update on both sides.

    ui.horizontal(|ui| {
        add_sensor_status("IMU: ", system_status.imu, ui, true);
        add_sensor_status("RF control link: ", system_status.rf_control_link, ui, true);
        add_sensor_status("OSD: ", system_status.osd, ui, true);
        add_sensor_status("RPM readings: ", system_status.esc_rpm, ui, false);
        add_sensor_status("Baro altimeter: ", system_status.baro, ui, false);
        add_sensor_status("AGL altimeter: ", system_status.tof, ui, false);
        add_sensor_status("GNSS (ie GPS): ", system_status.gps, ui, false);
        add_sensor_status("Magnetometer: ", system_status.magnetometer, ui, false);
        add_sensor_status("SPI flash: ", system_status.flash_spi, ui, false);
        // add_sensor_status("ESC telemetry: ", system_status.esc_telemetry, ui);

        // todo: Probably a separate row for faults?
        // todo:  Helper as above for bit statuses.
        // let val = system_status.rf_control_fault;
        // ui.vertical(|ui| {
        //     ui.label("RF faults: ");
        //     ui.label(RichText::new(val.to_string()).color(test_val_to_color(val)));
        // });
        // ui.add_space(SPACING_HORIZONTAL);
        //
        // let val = system_status.esc_rpm_fault;
        // ui.vertical(|ui| {
        //     ui.label("RPM faults: ");
        //     ui.label(RichText::new(val.to_string()).color(test_val_to_color(val)));
        // });
        // ui.add_space(SPACING_HORIZONTAL);
    });
}

pub fn run(state: &mut State, ctx: &egui::Context, scene: &mut Scene) -> EngineUpdates {
    // pub fn run() -> impl FnMut(&egui::Context) {
    //     move |state: &mut State, ctx: &egui::Context| {

    let agl = match &state.altitude_agl {
        Some(a) => a.to_string() + " M",
        None => "(Not connected)".to_owned(), // todo: Currently doesn't show
    };

    let lat = match state.lat {
        Some(l) => rad_to_dms(l, LatLon::Lat),
        None => "(No GNSS connected)".to_owned(), // todo: Currently doesn't show
    };

    let lon = match state.lon {
        Some(l) => rad_to_dms(l, LatLon::Lon),
        None => "(No GNSS connected)".to_owned(), // todo: Currently doesn't show
    };

    // todo: For formatting, try RichText, ie
    // todo: ui.heading(RichText::new("AnyLeaf Preflight").size(10.));

    let alt_hold = match state.autopilot_status.alt_hold {
        Some((alt_type, val)) => {
            let type_lbl = match alt_type {
                AltType::Msl => "MSL",
                AltType::Agl => "AGL",
            };
            // format!("{}m {type_lbl}", val as u16)
            format!("{:.1}m {type_lbl}", val)
        }
        None => "Off".to_owned(),
    };

    let hdg_hold = match state.autopilot_status.hdg_hold {
        Some(val) => format!("{}°", val * 360. / TAU),
        None => "Off".to_owned(),
    };

    let yaw_assist = match state.autopilot_status.yaw_assist {
        YawAssist::Disabled => "Off",
        YawAssist::YawAssist => "Auto yaw",
        YawAssist::RollAssist => "Auto roll",
    };

    let mut engine_updates = EngineUpdates::default();

    let panel = egui::SidePanel::left("UI panel").min_width(MIN_UI_WIDTH); // ID must be unique among panels.

    panel.show(ctx, |ui| {
        engine_updates.ui_size = ui.available_height();

        match state.common.connection_status {
            ConnectionStatus::Connected => (),
            ConnectionStatus::NotConnected => {
                add_not_connected_page(ui);
                state.config_ui = None;
                return; // todo?
            }
        }

        egui::containers::ScrollArea::vertical()
            // .max_height(400.)
            .show(ui, |ui| {

                ui.spacing_mut().item_spacing = egui::vec2(ITEM_SPACING_X, ITEM_SPACING_Y);

                ui.spacing_mut().slider_width = SLIDER_WIDTH;

                let aircraft_type = match state.aircraft_type {
                    AircraftType::Quadcopter => "Quadcopter",
                    AircraftType::FixedWing => "Fixed wing",
                };

                ui.heading(format!("Aircraft type: {}", aircraft_type));

                add_system_status(ui, &state.system_status);

                ui.add_space(SPACE_BETWEEN_SECTIONS);

                ui.vertical(|ui| {
                    ui.heading(RichText::new("Sensors").color(Color32::WHITE));

                    ui.horizontal(|ui| {
                        // todo: Center these.
                        ui.vertical(|ui| {
                            ui.label("Altitude baro:");
                            ui.label(format!("{:.1} m", state.altitude_baro));
                        });

                        ui.vertical(|ui| {
                            ui.label("Pressure (kPa):");
                            ui.label(format!("{:.3}", state.pressure_static / 1_000.));
                        });

                        ui.vertical(|ui| {
                            ui.label("Temp (°C):");
                            ui.label(format!("{:.1}", state.temp_baro - 273.15));
                        });
                        // todo: Display pressure and temperature here to, to QC.
                        ui.add_space(SPACING_HORIZONTAL);

                        if state.altitude_agl.is_some() {
                            ui.vertical(|ui| {
                                ui.label("Altitude AGL:");
                                ui.label(&agl);
                            });
                            ui.add_space(SPACING_HORIZONTAL);
                        }

                        ui.vertical(|ui| {
                            ui.label("Batt cell count:");
                            let selected = &mut state.batt_cell_count;
                            ComboBox::from_id_source(0)
                                .width(60.)
                                .selected_text(selected.as_str())
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(selected, BattCellCount::S2, "2S");
                                    ui.selectable_value(selected, BattCellCount::S3, "3S");
                                    ui.selectable_value(selected, BattCellCount::S4, "4S");
                                    ui.selectable_value(selected, BattCellCount::S6, "6S");
                                    ui.selectable_value(selected, BattCellCount::S8, "8S");
                                });
                        });
                        ui.add_space(SPACING_HORIZONTAL);

                        // todo: Add user-selectable battery cell field. Have this save somewhere
                        // todo on the computer.
                        // todo: Color-code appropriately.

                        let batt_connected = state.batt_v > 0.1;

                        // todo: Consider filtering instead with a LP.
                        // Round to avoid a jittery progress bar.
                        // let batt_v = (state.batt_v * 10.).round() / 10.;
                        let batt_v = state.batt_v;

                        let batt_life = batt_left_from_v(batt_v, state.batt_cell_count);
                        ui.vertical(|ui| {
                            ui.label("Batt Volts:");
                            // ui.label(format!("{:.1}", &state.batt_v));
                            if batt_connected {
                                ui.label(
                                    RichText::new(format!("{:.1}", batt_v))
                                        .color(batt_charge_to_color(batt_life)),
                                );
                            } else {
                                ui.label(RichText::new("Not connected").color(Color32::GOLD));
                            }
                        });
                        ui.add_space(SPACING_HORIZONTAL);

                        if batt_connected {
                            // tood: Color the bar.
                            ui.vertical(|ui| {
                                ui.label("Batt Life");
                                let bar = ProgressBar::new(batt_life).desired_width(BATT_LIFE_WIDTH);
                                ui.add(bar);
                            });
                            ui.add_space(SPACING_HORIZONTAL);
                        }

                        ui.vertical(|ui| {
                            ui.label("ESC current (A):");
                            ui.label(format!("{:.1}", &state.current));
                        });
                        ui.add_space(SPACING_HORIZONTAL);

                        if state.lat.is_some() && state.lon.is_some() {
                            // todo: Separate PPKS section A/r
                            ui.vertical(|ui| {
                                ui.label("Latitude:");
                                ui.label(lat);
                            });

                            ui.vertical(|ui| {
                                ui.label("Longitude:");
                                ui.label(lon);
                            });
                        } else {
                            // ui.label("GNSS not connected)");
                            // ui.label(""); // Spacer
                        }
                    });

                    ui.add_space(SPACE_BETWEEN_SECTIONS);

                    add_control_data(state.system_status.rf_control_link, &state.controls, ui);
                });

                // todo: Input mode switch etc.

                ui.add_space(SPACE_BETWEEN_SECTIONS);

                ui.heading(RichText::new("Control link signal").color(Color32::WHITE));
                // todo: Evaluate which of these you want.
                // todo: RSSI 2, antenna etc A/R only if full diversity

                add_link_stats(state.system_status.rf_control_link, &state.link_stats, ui);

                ui.add_space(SPACE_BETWEEN_SECTIONS);

                ui.heading(RichText::new("Autopilot status").color(Color32::WHITE));

                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.label("Alt hold");
                        ui.label(&alt_hold);
                    });
                    ui.add_space(SPACING_HORIZONTAL);

                    ui.vertical(|ui| {
                        ui.label("Heading hold");
                        ui.label(&hdg_hold);
                    });
                    ui.add_space(SPACING_HORIZONTAL);

                    // todo: When you put yaw and/or roll assist back, you probably
                    // todo want one field for it.
                    ui.vertical(|ui| {
                        ui.label("Yaw assist");
                        ui.label(yaw_assist);
                    });
                });

                ui.add_space(SPACE_BETWEEN_SECTIONS);

                ui.heading(RichText::new("Motor power settings and RPM").color(Color32::WHITE));


                // todo: Put back
                // todo: Color-code power and perhaps RPM.
                // ui.horizontal(|ui| {
                //     ui.vertical(|ui| {
                //         ui.label("Motor 1 power");
                //         ui.label(format!("{:.3}", &state.current_pwr.front_left));
                //     });
                //     ui.add_space(SPACING_HORIZONTAL);
                //
                //     ui.vertical(|ui| {
                //         ui.label("Motor 2 power");
                //         ui.label(format!("{:.3}", &state.current_pwr.front_right));
                //     });
                //     ui.add_space(SPACING_HORIZONTAL);
                //
                //     ui.vertical(|ui| {
                //         ui.label("Motor 3 power");
                //         ui.label(format!("{:.3}", &state.current_pwr.aft_left));
                //     });
                //     ui.add_space(SPACING_HORIZONTAL);
                //
                //     ui.vertical(|ui| {
                //         ui.label("Motor 4 Rpower");
                //         ui.label(format!("{:.3}", &state.current_pwr.aft_right));
                //     });
                // });

                // todo: Motors 1-4 vs positions?

                ui.add_space(SPACE_BETWEEN_SECTIONS);

                match state.aircraft_type {
                    AircraftType::Quadcopter => {

                        //
                        //
                        // ui.horizontal(|ui| {
                        //     ui.horizontal(|ui| {
                        //         ui.vertical(|ui| {
                        //             ui.label("Motor 1 RPM");
                        //             ui.label(&format_rpm(state.rpm_readings.front_left));
                        //         });
                        //         ui.add_space(SPACING_HORIZONTAL);
                        //
                        //         ui.vertical(|ui| {
                        //             ui.label("Motor 2 RPM");
                        //             ui.label(&format_rpm(state.rpm_readings.front_right));
                        //         });
                        //         ui.add_space(SPACING_HORIZONTAL);
                        //
                        //         ui.vertical(|ui| {
                        //             ui.label("Motor 3 RPM");
                        //             ui.label(&format_rpm(state.rpm_readings.aft_left));
                        //         });
                        //         ui.add_space(SPACING_HORIZONTAL);
                        //
                        //         ui.vertical(|ui| {
                        //             ui.label("Motor 4 RPM");
                        //             ui.label(&format_rpm(state.rpm_readings.aft_right));
                        //         });
                        //     });
                        //
                        //     ui.vertical(|ui| {
                        //         ui.heading("Motor mapping");
                        //
                        //         // todo: Warning color
                        //         if ui
                        //             .add(
                        //                 Button::new(
                        //                     RichText::new("Change motor mapping").color(Color32::BLACK),
                        //                 )
                        //                 .fill(Color32::from_rgb(220, 120, 10)),
                        //             )
                        //             .clicked()
                        //         {
                        //             state.editing_motor_mapping = !state.editing_motor_mapping;
                        //         };
                        //
                        //         if state.editing_motor_mapping {
                        //             ui.horizontal(|ui| {
                        //                 for (label, value, reversed, id) in [
                        //                     (
                        //                         "Motor 1",
                        //                         state.control_mapping_quad.m1,
                        //                         &mut state.control_mapping_quad.m1_reversed,
                        //                         0,
                        //                     ),
                        //                     (
                        //                         "Motor 2",
                        //                         state.control_mapping_quad.m2,
                        //                         &mut state.control_mapping_quad.m2_reversed,
                        //                         1,
                        //                     ),
                        //                     (
                        //                         "Motor 3",
                        //                         state.control_mapping_quad.m3,
                        //                         &mut state.control_mapping_quad.m3_reversed,
                        //                         2,
                        //                     ),
                        //                     (
                        //                         "Motor 4",
                        //                         state.control_mapping_quad.m4,
                        //                         &mut state.control_mapping_quad.m4_reversed,
                        //                         3,
                        //                     ),
                        //                 ]
                        //                 .into_iter()
                        //                 {
                        //                     // todo: For now, only set up for quad
                        //                     ui.vertical(|ui| {
                        //                         ui.label(label);
                        //                         ui.label(value.as_str());
                        //
                        //                         let mut selected = reversed;
                        //                         ComboBox::from_id_source(id)
                        //                             .width(MOTOR_MAPPING_DROPDOWN_WIDTH)
                        //                             .selected_text(motor_dir_format(*selected))
                        //                             .show_ui(ui, |ui| {
                        //                                 ui.selectable_value(selected, false, "Normal");
                        //                                 ui.selectable_value(selected, true, "Reversed");
                        //                             });
                        //                     });
                        //                     ui.add_space(SPACING_HORIZONTAL);
                        //                 }
                        //             });
                        //         } else {
                        //             ui.horizontal(|ui| {
                        //                 for (label, value, reversed) in [
                        //                     (
                        //                         "Motor 1",
                        //                         state.control_mapping_quad.m1,
                        //                         state.control_mapping_quad.m1_reversed,
                        //                     ),
                        //                     (
                        //                         "Motor 2",
                        //                         state.control_mapping_quad.m2,
                        //                         state.control_mapping_quad.m2_reversed,
                        //                     ),
                        //                     (
                        //                         "Motor 3",
                        //                         state.control_mapping_quad.m3,
                        //                         state.control_mapping_quad.m3_reversed,
                        //                     ),
                        //                     (
                        //                         "Motor 4",
                        //                         state.control_mapping_quad.m4,
                        //                         state.control_mapping_quad.m4_reversed,
                        //                     ),
                        //                 ]
                        //                 .into_iter()
                        //                 {
                        //                     ui.vertical(|ui| {
                        //                         ui.label(label);
                        //                         ui.label(value.as_str());
                        //                         ui.label(motor_dir_format(reversed));
                        //                     });
                        //                     ui.add_space(SPACING_HORIZONTAL);
                        //                 }
                        //             });
                        //         }
                        //     });
                        // });

                        ui.add_space(SPACE_BETWEEN_SECTIONS);

                        add_motor_commands(
                            ui,
                            &mut state.pwr_commanded_from_ui,
                            &mut state.rpms_commanded_from_ui,
                        );

                        ui.horizontal(|ui| {
                            if ui
                                .add(
                                    Button::new(RichText::new("Start Motors").color(Color32::BLACK))
                                        .fill(Color32::from_rgb(220, 120, 10)),
                                )
                                .clicked()
                            {
                                match state.common.get_port() {
                                    Ok(p) => {
                                        send_cmd::<MsgType>(MsgType::StartMotors, p).ok();
                                    }
                                    Err(_) => {}
                                }
                            };

                            // todo: DRY. Use a loop.(?)

                            if ui
                                .add(
                                    Button::new(RichText::new("Stop Motors").color(Color32::BLACK))
                                        .fill(Color32::from_rgb(220, 120, 10)),
                                )
                                .clicked()
                            {
                                match state.common.get_port() {
                                    Ok(p) => {
                                        send_cmd::<MsgType>(MsgType::StopMotors, p).ok();
                                    }
                                    Err(_) => {}
                                }
                            };
                        });
                    }

                    AircraftType::FixedWing => {
                        ui.heading(RichText::new("Servo commands").color(Color32::WHITE));

                        ui.add_space(SPACE_BETWEEN_SECTIONS);
                    }
                }

                ui.add_space(SPACE_BETWEEN_SECTIONS);

                ui.heading(RichText::new("Rate PID coefficients").color(Color32::WHITE));

                ui.horizontal(|ui| match &mut state.config_ui {
                    Some(config_ui) => {
                        let p_label =
                            &("P: ".to_owned() + &state.config_on_device.pid_coeffs.p.to_string());
                        let i_label =
                            &("I: ".to_owned() + &state.config_on_device.pid_coeffs.i.to_string());
                        let d_label =
                            &("D: ".to_owned() + &state.config_on_device.pid_coeffs.d.to_string());
                        let att_ttc_label =
                            &("Att TTC: ".to_owned() + &state.config_on_device.pid_coeffs.att_ttc.to_string());

                        const FACTOR: f32 = 1_000.;
                        // todo tmep to deal with poor float handling

                        let mut p_int = (config_ui.pid_coeffs.p * FACTOR) as u32;
                        let mut i_int = (config_ui.pid_coeffs.i * FACTOR) as u32;
                        let mut d_int = (config_ui.pid_coeffs.d * FACTOR) as u32;
                        let mut att_ttc_int = (config_ui.pid_coeffs.att_ttc * FACTOR) as u32;

                        ui.label(p_label);
                        // text_edit_float(&mut config_ui.pid_coeffs.p, 0., ui);
                        text_edit_int(&mut p_int, ui);

                        ui.label(i_label);
                        // text_edit_float(&mut config_ui.pid_coeffs.i, 0., ui);
                        text_edit_int(&mut i_int, ui);

                        ui.label(d_label);
                        // text_edit_float(&mut config_ui.pid_coeffs.d, 0., ui);
                        text_edit_int(&mut d_int, ui);

                        ui.label(att_ttc_label);
                        // text_edit_float(&mut config_ui.pid_coeffs.d, 0., ui);
                        text_edit_int(&mut att_ttc_int, ui);

                        config_ui.pid_coeffs.p = (p_int as f32) / FACTOR;
                        config_ui.pid_coeffs.i = (i_int as f32) / FACTOR;
                        config_ui.pid_coeffs.d = (d_int as f32) / FACTOR;
                        config_ui.pid_coeffs.att_ttc = (att_ttc_int as f32) / FACTOR;

                        ui.add_space(SPACE_BETWEEN_SECTIONS);

                        if ui
                            .add(
                                Button::new(RichText::new("Save config to device").color(Color32::BLACK))
                                    .fill(Color32::from_rgb(220, 120, 10)),
                            )
                            .clicked()
                        {
                            if send_payload::<MsgType, PAYLOAD_SIZE_CONFIG>(
                                MsgType::SaveConfig,
                                &config_ui.to_bytes(),
                                state.common.get_port().unwrap(),
                            )
                                .is_err()
                            {
                                println!("Error saving config.")
                            };
                        };
                    }
                    None => (),
                });
            });
    });

    engine_updates
}
