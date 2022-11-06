//! This module contains code for the GUI

use std::f32::consts::TAU;

use egui::ImageData::Color;
use egui::{self, Button, Color32, ComboBox, CursorIcon::Default, RichText, Ui};

use graphics::{EngineUpdates, Scene};

use crate::{
    types::{AircraftType, AltType, ArmStatus, LinkStats, SensorStatus, YawAssist},
    RotorPosition, State,
};

// pub const UI_PANEL_SIZE: f32 = 1_600.;

const ITEM_SPACING_X: f32 = 10.;
const ITEM_SPACING_Y: f32 = 12.;
const SPACE_BETWEEN_SECTIONS: f32 = 30.;
const SPACING_HORIZONTAL: f32 = 26.;
const SPACING_HORIZONTAL_TIGHT: f32 = 16.;
const CONTROL_BAR_WIDTH: f32 = 200.; // eg pitch, roll, yaw, throttle control.
const MOTOR_MAPPING_DROPDOWN_WIDTH: f32 = 100.;

impl SensorStatus {
    // Note Not included in Corvus
    fn as_str(&self) -> &str {
        match self {
            Self::Pass => "Pass",
            Self::Fail => "Failure",
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

fn format_rpm(rpm: Option<f32>) -> String {
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
fn add_sensor_status(label: &str, val: SensorStatus, ui: &mut Ui) {
    ui.vertical(|ui| {
        ui.label(label);
        ui.label(RichText::new(val.as_str()).color(val.as_color()));
    });
    ui.add_space(SPACING_HORIZONTAL);
}

/// Add a progress bar displaying the current position of a control.
fn add_control_disp(mut val: f32, text: &str, throttle: bool, ui: &mut Ui) {
    ui.label(text);
    // convert from -1. to 1. to 0. to 1.
    let v = if throttle { val / 2. + 0.5 } else { val };
    let bar = egui::ProgressBar::new(v).desired_width(CONTROL_BAR_WIDTH);
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

fn add_link_stats(link_stats: &LinkStats, ui: &mut Ui) {
    // todo: convert things like tx power to actual power disp
    ui.horizontal(|ui| {
        ui.vertical(|ui| {
            ui.label("Uplink RSSI 1");
            ui.label(&link_stats.uplink_rssi_1.to_string());
        });
        ui.add_space(SPACING_HORIZONTAL_TIGHT);

        // ui.vertical(|ui| {
        //     ui.label("Uplink RSSI 2");
        //     ui.label(&link_stats.uplink_rssi_2.to_string());
        // });
        // ui.add_space(SPACING_HORIZONTAL_TIGHT);

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

fn add_not_connected_page(ui: &mut Ui) {
    let text = "No data received from the flight controller";
    ui.heading(RichText::new(text).color(Color32::GOLD).size(60.));
    ui.add_space(300.); // So not aligned to bottom of the window.
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
            format!("{}m {type_lbl}", val as u16)
        }
        None => "Off".to_owned(),
    };

    let hdg_hold = match state.autopilot_status.alt_hold {
        Some((alt_type, val)) => format!("{}°", val * 360. / TAU),
        None => "Off".to_owned(),
    };

    let yaw_assist = match state.autopilot_status.yaw_assist {
        YawAssist::Disabled => "Off",
        YawAssist::YawAssist => "Auto yaw",
        YawAssist::RollAssist => "Auto roll",
    };

    let panel = egui::TopBottomPanel::bottom("UI panel"); // ID must be unique among panels.

    panel.show(ctx, |ui| {
        if !state.connected_to_fc {
            add_not_connected_page(ui);
            return; // todo?
        }
        // ui.vscroll(true);

        ui.spacing_mut().item_spacing = egui::vec2(ITEM_SPACING_X, ITEM_SPACING_Y);

        // ui.heading("AnyLeaf Preflight");

        ui.heading("System status"); // todo: Get this from FC; update on both sides.

        ui.horizontal(|ui| {
            add_sensor_status("IMU: ", state.system_status.imu, ui);
            add_sensor_status("RF control link: ", state.system_status.rf_control_link, ui);
            add_sensor_status("RPM readings: ", state.system_status.esc_rpm, ui);
            add_sensor_status("Baro altimeter: ", state.system_status.baro, ui);
            add_sensor_status("AGL altimeter: ", state.system_status.tof, ui);
            add_sensor_status("GNSS (ie GPS): ", state.system_status.gps, ui);
            add_sensor_status("Magnetometer: ", state.system_status.magnetometer, ui);
            // add_sensor_status("ESC telemetry: ", state.system_status.esc_telemetry, ui);

            // todo: Probably a separate row for faults?
            // todo:  Helper as above for bit statuses.
            let val = state.system_status.rf_control_fault;
            ui.vertical(|ui| {
                ui.label("RF faults: ");
                ui.label(RichText::new(val.to_string()).color(test_val_to_color(val)));
            });
            ui.add_space(SPACING_HORIZONTAL);
        });

        ui.add_space(SPACE_BETWEEN_SECTIONS);

        ui.vertical(|ui| {
            ui.heading("Sensors");

            ui.horizontal(|ui| {
                // todo: Center these.
                ui.vertical(|ui| {
                    ui.label("Altitude baro:");
                    ui.label(&(state.altitude_baro.to_string() + " m"));
                });

                if state.altitude_agl.is_some() {
                    ui.vertical(|ui| {
                        ui.label("Altitude AGL:");
                        ui.label(&agl);
                    });
                }

                ui.vertical(|ui| {
                    ui.label("Battery V:");
                    ui.label(&state.batt_v.to_string());
                });

                ui.vertical(|ui| {
                    ui.label("ESC current (A):");
                    ui.label(&state.current.to_string());
                });

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

            ui.vertical(|ui| {
                ui.heading("Control commands");

                if state.system_status.rf_control_link == SensorStatus::Pass {
                    ui.horizontal(|ui| {
                        add_control_disp(state.controls.pitch, "Pitch", true, ui);
                        add_control_disp(state.controls.roll, "Roll", true, ui);
                    });
                    ui.horizontal(|ui| {
                        add_control_disp(state.controls.yaw, "Yaw", true, ui);
                        add_control_disp(state.controls.throttle, "Throttle", false, ui);
                    });

                    // todo Important: This isn't the actual arm status state! It's the control input!
                    // todo: Potentially misleading.
                    ui.vertical(|ui| {
                        ui.label("Motor arm switch");
                        let val = state.controls.arm_status;
                        ui.label(RichText::new(val.as_str()).color(val.as_color()));
                    });
                } else {
                    ui.heading(
                        RichText::new("(Radio control link not connected)").color(Color32::GOLD),
                    );
                }
            });
        });

        // todo: Input mode switch etc.

        ui.add_space(SPACE_BETWEEN_SECTIONS);

        ui.heading("Control link signal");
        // todo: Evaluate which of these you want.
        // todo: RSSI 2, antenna etc A/R only if full diversity

        if state.system_status.rf_control_link == SensorStatus::Pass {
            add_link_stats(&state.link_stats, ui);
        } else {
            ui.heading(RichText::new("(Radio control link not connected)").color(Color32::GOLD));
        }

        ui.add_space(SPACE_BETWEEN_SECTIONS);

        ui.heading("Autopilot status");

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

        ui.heading("Motor power settings and RPM");

        // todo: Motors 1-4 vs positions?
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.label("Motor 1 RPM");
                ui.label(&format_rpm(state.rpm1));
            });
            ui.add_space(SPACING_HORIZONTAL);

            ui.vertical(|ui| {
                ui.label("Motor 2 RPM");
                ui.label(&format_rpm(state.rpm2));
            });
            ui.add_space(SPACING_HORIZONTAL);

            ui.vertical(|ui| {
                ui.label("Motor 3 RPM");
                ui.label(&format_rpm(state.rpm3));
            });
            ui.add_space(SPACING_HORIZONTAL);

            ui.vertical(|ui| {
                ui.label("Motor 4 RPM");
                ui.label(&format_rpm(state.rpm3));
            });
        });

        ui.add_space(SPACE_BETWEEN_SECTIONS);

        match state.aircraft_type {
            AircraftType::Quadcopter => {
                ui.heading("Motor mapping");

                // todo: Warning color
                if ui
                    .add(
                        Button::new(RichText::new("Change motor mapping").color(Color32::BLACK))
                            .fill(Color32::from_rgb(220, 120, 10)),
                    )
                    .clicked()
                {
                    state.editing_motor_mapping = !state.editing_motor_mapping;
                };

                // todo: Helper fns here to reduce rep
                if state.editing_motor_mapping {
                    ui.horizontal(|ui| {
                        // todo: For now, only set up for quad
                        ui.vertical(|ui| {
                            ui.label("Motor 1");
                            ui.label(state.control_mapping_quad.m1.as_str());

                            let mut selected = &mut state.control_mapping_quad.m1_reversed;
                            ComboBox::from_id_source(0)
                                .width(MOTOR_MAPPING_DROPDOWN_WIDTH)
                                .selected_text(motor_dir_format(*selected))
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(selected, false, "Normal");
                                    ui.selectable_value(selected, true, "Reversed");
                                });
                        });
                        ui.add_space(SPACING_HORIZONTAL);

                        ui.vertical(|ui| {
                            ui.label("Motor 2");
                            ui.label(state.control_mapping_quad.m1.as_str());

                            let mut selected = &mut state.control_mapping_quad.m2_reversed;
                            ComboBox::from_id_source(1)
                                .width(MOTOR_MAPPING_DROPDOWN_WIDTH)
                                .selected_text(motor_dir_format(*selected))
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(selected, false, "Normal");
                                    ui.selectable_value(selected, true, "Reversed");
                                });
                        });
                        ui.add_space(SPACING_HORIZONTAL);

                        ui.vertical(|ui| {
                            ui.label("Motor 3");
                            ui.label(state.control_mapping_quad.m1.as_str());

                            let mut selected = &mut state.control_mapping_quad.m3_reversed;
                            ComboBox::from_id_source(2)
                                .width(MOTOR_MAPPING_DROPDOWN_WIDTH)
                                .selected_text(motor_dir_format(*selected))
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(selected, false, "Normal");
                                    ui.selectable_value(selected, true, "Reversed");
                                });
                        });
                        ui.add_space(SPACING_HORIZONTAL);

                        ui.vertical(|ui| {
                            ui.label("Motor 4");
                            ui.label(state.control_mapping_quad.m1.as_str());

                            let mut selected = &mut state.control_mapping_quad.m4_reversed;
                            ComboBox::from_id_source(3)
                                .width(MOTOR_MAPPING_DROPDOWN_WIDTH)
                                .selected_text(motor_dir_format(*selected))
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(selected, false, "Normal");
                                    ui.selectable_value(selected, true, "Reversed");
                                });
                        });
                        ui.add_space(SPACING_HORIZONTAL);
                    });
                } else {
                    ui.horizontal(|ui| {
                        // todo: For now, only set up for quad
                        ui.vertical(|ui| {
                            ui.label("Motor 1");
                            ui.label(state.control_mapping_quad.m1.as_str());
                            ui.label(motor_dir_format(state.control_mapping_quad.m1_reversed));
                        });
                        ui.add_space(SPACING_HORIZONTAL);

                        ui.vertical(|ui| {
                            ui.label("Motor 2");
                            ui.label(state.control_mapping_quad.m2.as_str());
                            ui.label(motor_dir_format(state.control_mapping_quad.m2_reversed));
                        });
                        ui.add_space(SPACING_HORIZONTAL);

                        ui.vertical(|ui| {
                            ui.label("Motor 3");
                            ui.label(state.control_mapping_quad.m3.as_str());
                            ui.label(motor_dir_format(state.control_mapping_quad.m3_reversed));
                        });
                        ui.add_space(SPACING_HORIZONTAL);

                        ui.vertical(|ui| {
                            ui.label("Motor 4");
                            ui.label(state.control_mapping_quad.m4.as_str());
                            ui.label(motor_dir_format(state.control_mapping_quad.m4_reversed));
                        });
                        ui.add_space(SPACING_HORIZONTAL);
                    });
                }

                ui.add_space(SPACE_BETWEEN_SECTIONS);
            }
            AircraftType::FlyingWing => {
                ui.heading("Servo commands");

                ui.add_space(SPACE_BETWEEN_SECTIONS);
            }
        }
    });

    EngineUpdates::default()
}
