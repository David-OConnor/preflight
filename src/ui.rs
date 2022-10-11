//! This module contains code for the GUI

use std::f32::consts::TAU;

use egui;

use crate::{
    types::{AircraftType, ArmStatus, SensorStatus},
    State,
};

// pub const UI_PANEL_SIZE: f32 = 1_600.;

const ITEM_SPACING_X: f32 = 10.;
const ITEM_SPACING_Y: f32 = 12.;
const SPACE_BETWEEN_SECTIONS: f32 = 30.;

impl SensorStatus {
    // Note Not included in Corvus
    fn to_string(&self) -> String {
        match self {
            Self::Pass => "✓".to_owned(),
            Self::Fail => "Failure".to_owned(),
            Self::NotConnected => "Not connected".to_owned(),
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

pub fn run(state: State) -> Box<dyn Fn(&egui::Context)> {
    Box::new(move |ctx: &egui::Context| {
        let panel = egui::TopBottomPanel::bottom("UI panel"); // ID must be unique among panels.

        // .default_width(UI_PANEL_SIZE);

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

        let arm_status = match state.controls.arm_status {
            ArmStatus::Disarmed => "Disarmed",
            ArmStatus::Armed => "Armed",
        };

        panel.show(ctx, |ui| {
            ui.spacing_mut().item_spacing = egui::vec2(ITEM_SPACING_X, ITEM_SPACING_Y);

            ui.heading("AnyLeaf Preflight");

            ui.heading("System status"); // todo: Get this from FC; update on both sides.

            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label("IMU:");
                    ui.label(&state.system_status.imu.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Baro altimeter:");
                    ui.label(&state.system_status.baro.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("GNSS (ie GPS):");
                    ui.label(&state.system_status.gps.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Time of Flight (AGL altimeter):");
                    ui.label(&state.system_status.tof.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Magnetometer:");
                    ui.label(&state.system_status.magnetometer.to_string());
                });
                // ui.vertical(|ui| {
                //     ui.label("ESC telemetry:");
                //     ui.label(&state.system_status.esc_telemetryu.to_string());
                // });
                ui.vertical(|ui| {
                    ui.label("RPM sensor");
                    ui.label(&state.system_status.esc_rpm.to_string());
                });
            });

            ui.heading("Sensors");

            // todo: Center these.
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label("Altitude baro:");
                    ui.label(&(state.altitude_baro.to_string() + " m"));
                });

                if state.altitude_agl.is_some() {
                    ui.vertical(|ui| {
                        ui.label("Altitude AGL:");
                        ui.label(agl);
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

            ui.heading("Control commands:");

            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label("Pitch");
                    ui.label(&state.controls.pitch.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Roll");
                    ui.label(&state.controls.roll.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Yaw");
                    ui.label(&state.controls.yaw.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Throttle");
                    ui.label(&state.controls.throttle.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Motor arm status");
                    ui.label(arm_status);
                });

                // todo: Input mode switch etc.
            });

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
                // ui.vertical(|ui| {
                //     ui.label("Uplink RSSI 2");
                //     ui.label(&state.link_stats.uplink_rssi_2.to_string());
                // });
                ui.vertical(|ui| {
                    ui.label("Link Quality");
                    ui.label(&state.link_stats.uplink_link_quality.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Uplink SNR");
                    ui.label(&state.link_stats.uplink_snr.to_string());
                });
                // ui.vertical(|ui| {
                //     ui.label("Active antenna");
                //     ui.label(&state.link_stats.active_antenna.to_string());
                // });
                ui.vertical(|ui| {
                    ui.label("RF mode");
                    ui.label(&state.link_stats.rf_mode.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Uplink Tx power");
                    ui.label(&tx_pwr_from_val(state.link_stats.uplink_tx_power));
                });
                ui.vertical(|ui| {
                    ui.label("Downlink RSSI");
                    ui.label(&state.link_stats.downlink_rssi.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Downlink link quality");
                    ui.label(&state.link_stats.downlink_link_quality.to_string());
                });
                ui.vertical(|ui| {
                    ui.label("Downlink SNR");
                    ui.label(&state.link_stats.downlink_snr.to_string());
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
    })
}
