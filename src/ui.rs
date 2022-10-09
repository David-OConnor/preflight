//! This module contains code for the GUI

use egui;

pub fn draw_ui(ctx: &egui::Context) {
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.heading("AnyLeaf Preflight");

        ui.heading("Attitude, and commanded attitude");

        ui.heading("Sensors");
        // Baro
        // TOF
        // GPS
        // Battery voltage
        // ESC current

        ui.heading("Control positions");

        ui.heading("Control link signal");

        ui.heading("Motor power settings and RPM");

        ui.heading("Motor mapping");

        // ui.heading("Servo commands");

        ui.horizontal(|ui| {
            // ui.text_edit_singleline(&mut name);
        });
        // ui.add(egui::Slider::new(&mut age, 0..=120).text("age"));
        // if ui.button("Click each year").clicked() {
        // Perform action here.
        // }
    });
}
