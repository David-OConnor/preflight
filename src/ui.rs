//! This module contains code for the GUI

use std::f32::consts::TAU;

use egui::{self, Color32, RichText};

use graphics::{EngineUpdates, Scene};

use crate::{
    types::{AircraftType, AltType, ArmStatus, SensorStatus, YawAssist},
    State,
};

// pub const UI_PANEL_SIZE: f32 = 1_600.;

const ITEM_SPACING_X: f32 = 10.;
const ITEM_SPACING_Y: f32 = 12.;
const SPACE_BETWEEN_SECTIONS: f32 = 30.;
const SPACING_HORIZONTAL: f32 = 26.;
const SPACING_HORIZONTAL_TIGHT: f32 = 16.;

impl SensorStatus {
    // Note Not included in Corvus
    fn to_str(&self) -> &str {
        match self {
            Self::Pass => "Pass",
            Self::Fail => "Failure",
            Self::NotConnected => "Not connected",
        }
    }

    fn to_color(&self) -> Color32 {
        match self {
            Self::Pass => Color32::LIGHT_GREEN,
            Self::Fail => Color32::LIGHT_RED,
            Self::NotConnected => Color32::GOLD,
        }
    }
}

// todo: Fixed wing arm status as well
impl ArmStatus {
    fn to_str(&self) -> &str {
        match self {
            Self::Armed => "Armed",
            Self::Disarmed => "Disarmed",
        }
    }

    fn to_color(&self) -> Color32 {
        match self {
            Self::Armed => Color32::YELLOW,
            Self::Disarmed => Color32::GREEN,
        }
    }
}

/// See Corvus, `ElrsTxPower`
fn tx_pwr_from_val(val: u8) -> String {
    match val {
        1 => "10mW".to_owned(),
        2 => "25mW".to_owned(),
        8 => "50mW".to_owned(),
        3 => "100mW".to_owned(),
        7 => "250mW".to_owned(),
        _ => "(Unknown)".to_owned(),
    }
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
fn add_sensor_status(label: &str, val: SensorStatus, ui: &mut egui::Ui) {
    ui.vertical(|ui| {
        ui.label(label);
        ui.label(RichText::new(val.to_str()).color(val.to_color()));
    });
    ui.add_space(SPACING_HORIZONTAL);
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
        // ui.vscroll(true);

        ui.spacing_mut().item_spacing = egui::vec2(ITEM_SPACING_X, ITEM_SPACING_Y);

        ui.heading("AnyLeaf Preflight");

        ui.heading("System status"); // todo: Get this from FC; update on both sides.

        ui.horizontal(|ui| {
            add_sensor_status("IMU: ", state.system_status.imu, ui);
            add_sensor_status("RPM readings: ", state.system_status.esc_rpm, ui);
            add_sensor_status("Baro altimeter: ", state.system_status.baro, ui);
            add_sensor_status("AGL altimeter: ", state.system_status.tof, ui);
            add_sensor_status("GNSS (ie GPS): ", state.system_status.gps, ui);
            add_sensor_status("Magnetometer: ", state.system_status.magnetometer, ui);
            // add_sensor_status("ESC telemetry: ", state.system_status.esc_telemetry, ui);
        });

        ui.add_space(SPACE_BETWEEN_SECTIONS);

        ui.horizontal(|ui| {
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
            });

            ui.add_space(80.);

            ui.vertical(|ui| {
                ui.heading("Control commands");
                ui.horizontal(|ui| {
                    ui.vertical(|ui| {
                        ui.label("Pitch");
                        ui.label(format!("{:.2}", &state.controls.pitch));
                    });
                    ui.add_space(SPACING_HORIZONTAL);

                    ui.vertical(|ui| {
                        ui.label("Roll");
                        ui.label(format!("{:.2}", &state.controls.roll));
                    });
                    ui.add_space(SPACING_HORIZONTAL);

                    ui.vertical(|ui| {
                        ui.label("Yaw");
                        ui.label(format!("{:.2}", &state.controls.yaw));
                    });
                    ui.add_space(SPACING_HORIZONTAL);

                    ui.vertical(|ui| {
                        ui.label("Throttle");
                        ui.label(format!("{:.2}", &state.controls.throttle));
                    });
                    ui.add_space(SPACING_HORIZONTAL);

                    // todo Important: This isn't the actual arm status state! It's the control input!
                    // todo: Potentially misleading.
                    ui.vertical(|ui| {
                        ui.label("Motor arm switch");
                        let val = state.controls.arm_status;
                        ui.label(RichText::new(val.to_str()).color(val.to_color()));
                    });
                });
            });
        });

        // todo: Input mode switch etc.

        ui.add_space(SPACE_BETWEEN_SECTIONS);

        ui.heading("Control link signal");
        // todo: Evaluate which of these you want.
        // todo: RSSI 2, antenna etc A/R only if full diversity

        // todo: convert things like tx power to actual power disp
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.label("Uplink RSSI 1");
                ui.label(&state.link_stats.uplink_rssi_1.to_string());
            });
            ui.add_space(SPACING_HORIZONTAL_TIGHT);

            // ui.vertical(|ui| {
            //     ui.label("Uplink RSSI 2");
            //     ui.label(&state.link_stats.uplink_rssi_2.to_string());
            // });
            // ui.add_space(SPACING_HORIZONTAL_TIGHT);

            ui.vertical(|ui| {
                ui.label("Link qual");
                ui.label(&state.link_stats.uplink_link_quality.to_string());
            });
            ui.add_space(SPACING_HORIZONTAL_TIGHT);

            ui.vertical(|ui| {
                ui.label("Uplink SNR");
                ui.label(&state.link_stats.uplink_snr.to_string());
            });
            ui.add_space(SPACING_HORIZONTAL_TIGHT);

            // ui.vertical(|ui| {
            //     ui.label("Active antenna");
            //     ui.label(&state.link_stats.active_antenna.to_string());
            // });
            // ui.add_space(SPACING_HORIZONTAL_TIGHT);

            ui.vertical(|ui| {
                ui.label("RF mode");
                ui.label(&state.link_stats.rf_mode.to_string());
            });
            ui.add_space(SPACING_HORIZONTAL);

            ui.vertical(|ui| {
                ui.label("Uplink Tx pwr");
                ui.label(&tx_pwr_from_val(state.link_stats.uplink_tx_power));
            });
            ui.add_space(SPACING_HORIZONTAL_TIGHT);

            ui.vertical(|ui| {
                ui.label("Downlink RSSI");
                ui.label(&state.link_stats.downlink_rssi.to_string());
            });
            ui.add_space(SPACING_HORIZONTAL_TIGHT);

            ui.vertical(|ui| {
                ui.label("Downlink link qual");
                ui.label(&state.link_stats.downlink_link_quality.to_string());
            });
            ui.add_space(SPACING_HORIZONTAL_TIGHT);

            ui.vertical(|ui| {
                ui.label("Downlink SNR");
                ui.label(&state.link_stats.downlink_snr.to_string());
            });
        });

        ui.add_space(SPACE_BETWEEN_SECTIONS);

        ui.heading("Autopilot status");

        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.label("Alt hold");
                ui.label(&alt_hold);
            });
            ui.vertical(|ui| {
                ui.label("Heading hold");
                ui.label(&hdg_hold);
            });
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
            ui.vertical(|ui| {
                ui.label("Motor 2 RPM");
                ui.label(&format_rpm(state.rpm2));
            });
            ui.vertical(|ui| {
                ui.label("Motor 3 RPM");
                ui.label(&format_rpm(state.rpm3));
            });
            ui.vertical(|ui| {
                ui.label("Motor 4 RPM");
                ui.label(&format_rpm(state.rpm3));
            });
        });

        ui.add_space(SPACE_BETWEEN_SECTIONS);

        match state.aircraft_type {
            AircraftType::Quadcopter => {
                ui.heading("Motor mapping");

                ui.add_space(SPACE_BETWEEN_SECTIONS);
            }
            AircraftType::FlyingWing => {
                ui.heading("Servo commands");

                ui.add_space(SPACE_BETWEEN_SECTIONS);
            }
        }

        // ui.text_edit_singleline(&mut name);

        // ui.add(egui::Slider::new(&mut age, 0..=120).text("age"));
        // if ui.button("Click each year").clicked() {
        // Perform action here.
        // }
    });
    // }

    Default::default()
}
