//! This module contains code related to the 3D render (Aircraft attitude depiction etc)

use std::{boxed::Box, time::Instant};

use graphics::{
    self, Camera, DeviceEvent, Entity, InputSettings, LightType, Lighting, Mesh, PointLight, Scene,
    UiSettings, EngineUpdates
};

use lin_alg2::{
    self,
    f32::{Quaternion, Vec3},
};

// todo: Fake horizon in background

use crate::{render, ui, SerialInterface, State, READ_INTERVAL_MS};

pub const BACKGROUND_COLOR: (f32, f32, f32) = (0.9, 0.9, 0.9);
const WINDOW_TITLE: &str = "AnyLeaf Preflight";
const WINDOW_WIDTH: f32 = 900.;
const WINDOW_HEIGHT: f32 = 800.;

// fn make_render_handler() -> impl FnMut(&mut State, &mut Scene) -> bool {
fn render_handler(state: &mut State, scene: &mut Scene, dt: f32) -> EngineUpdates {
    // move |state: &mut State, scene: &mut Scene| {
    let mut engine_updates = EngineUpdates::default();

    if state.interface.serial_port.is_some()
        && state.last_fc_query.elapsed().as_millis() > READ_INTERVAL_MS
    {
        state.read_all().unwrap();

        scene.entities[0].orientation = state.attitude;

        scene.entities[1].orientation = state.attitude_commanded;
        state.last_fc_query = Instant::now();

        engine_updates.entities = true;
    }

    engine_updates
}

fn event_handler(
    _state: &mut State,
    _event: DeviceEvent,
    _scene: &mut Scene,
    _dt: f32, // in seconds
) -> EngineUpdates {
    Default::default()
}

/// The entry point for our renderer.
pub fn run(fc: SerialInterface, mut state: State) {
    let entities = vec![
        // Aircraft estimated attitude
        Entity::new(
            0,
            Vec3::new(-6., 9., 0.),
            Quaternion::new_identity(),
            1.,
            (1., 0., 1.),
            1.,
        ),
        // Commanded attitutde
        Entity::new(
            0,
            Vec3::new(6., 9., 0.),
            Quaternion::new_identity(),
            1.,
            (1., 0., 1.),
            1.,
        ),
    ];

    // todo: Engine-side, you need to set it up so the window is centered
    // todo: on the visible part; not overlayed over.

    let scene = Scene {
        // todo: Change these meshes A/R.
        meshes: vec![Mesh::new_box(1., 1., 1.), Mesh::new_sphere(1., 50, 50)],
        entities,
        lighting: Lighting {
            ambient_color: [1., 1., 0., 1.],
            ambient_intensity: 0.15,
            point_lights: vec![PointLight {
                type_: LightType::Omnidirectional,
                position: Vec3::new(0., 30., -5.),
                diffuse_color: [1., 1., 1., 1.],
                specular_color: [1., 1., 1., 1.],
                diffuse_intensity: 1000.,
                specular_intensity: 1000.,
            }],
        },
        background_color: render::BACKGROUND_COLOR,
        window_size: (WINDOW_WIDTH, WINDOW_HEIGHT),
        window_title: WINDOW_TITLE.to_owned(),
        camera: Camera {
            position: Vec3::new(0., 0., -20.),
            ..Default::default()
        },
    };

    let input_settings = InputSettings::default();
    let ui_settings = UiSettings { width: 0. };

    graphics::run(
        state,
        scene,
        input_settings,
        ui_settings,
        render_handler,
        event_handler,
        ui::run,
    );
}

// todo: Use state cam? Should it be in `Scene`?
