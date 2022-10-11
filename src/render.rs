//! This module contains code related to the 3D render (Aircraft attitude depiction etc)

use std::{boxed::Box, time::Instant};

use graphics::{
    self, DeviceEvent, Entity, InputSettings, LightType, Lighting, Mesh, PointLight, Scene,
    UiSettings,
};

use lin_alg2::{
    self,
    f32::{Quaternion, Vec3},
};

// todo: Fake horizon in background

use crate::{render, ui, Fc, State, READ_INTERVAL_MS};

pub const BACKGROUND_COLOR: (f32, f32, f32) = (0.9, 0.9, 0.9);
const WINDOW_TITLE: &'static str = "AnyLeaf Preflight";
const WINDOW_WIDTH: f32 = 900.;
const WINDOW_HEIGHT: f32 = 800.;

fn event_handler(
    _event: DeviceEvent,
    _scene: &mut Scene,
    _dt: f32, // in seconds
) -> bool {
    false
}

/// The entry point for our renderer.
pub fn run(mut fc: Fc, mut state: State) {
    let entities = vec![
        // Aircraft estimated atoms.
        Entity::new(
            0,
            Vec3::new(-4., 4., 10.),
            Quaternion::new_identity(),
            1.,
            (1., 0., 1.),
            1.,
        ),
        // Commanded attitutde
        Entity::new(
            1,
            Vec3::new(4., 4., 10.),
            Quaternion::new_identity(),
            1.,
            (1., 0., 1.),
            1.,
        ),
    ];

    let scene = Scene {
        meshes: vec![Mesh::new_box(1., 1., 1.), Mesh::new_box(1., 1., 1.)],
        entities,
        lighting: Lighting {
            ambient_color: [1., 1., 0., 0.5],
            ambient_intensity: 0.15,
            point_lights: vec![PointLight {
                type_: LightType::Omnidirectional,
                position: Vec3::new(0., 20., 0.),
                diffuse_color: [1., 1., 1., 0.5],
                specular_color: [1., 1., 1., 0.5],
                diffuse_intensity: 1_000_000.,
                specular_intensity: 1_000_000.,
            }],
        },
        background_color: render::BACKGROUND_COLOR,
        window_size: (WINDOW_WIDTH, WINDOW_HEIGHT),
        window_title: WINDOW_TITLE.to_owned(),
        ..Default::default()
    };

    let input_settings = InputSettings::default();
    let ui_settings = UiSettings {};

    // We clone `state` here due to its use in 2 closures, with the mutable one
    // declared first.
    let state2 = state.clone();

    let render_handler = move |scene: &mut Scene| {
        // todo: Don't have a sleep here! Instead, you need some way to sequence
        // todo the reads, ie not every frame.
        if fc.serial_port.is_some() && state.last_fc_query.elapsed().as_millis() > READ_INTERVAL_MS
        {
            fc.read_all(&mut state).unwrap();

            scene.entities[0].orientation = state.attitude;

            scene.entities[1].orientation = state.attitude_commanded;
            state.last_fc_query = Instant::now();

            true
        } else {
            false
        }
    };

    graphics::run(
        scene,
        input_settings,
        ui_settings,
        Box::new(render_handler),
        Box::new(event_handler),
        // Box::new(draw_ui),
        ui::run(state2),
    );
}

// todo: Use state cam? Should it be in `Scene`?
