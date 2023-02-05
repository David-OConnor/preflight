//! This module contains code related to the 3D render (Aircraft attitude depiction etc)

use std::time::Duration;
use std::{
    boxed::Box,
    sync::atomic::{AtomicUsize, Ordering},
    time::Instant,
};

use graphics::{
    self, Camera, DeviceEvent, EngineUpdates, Entity, InputSettings, LightType, Lighting, Mesh,
    PointLight, Scene, UiLayout, UiSettings, Vertex,
};

use lin_alg2::{
    self,
    f32::{Quaternion, Vec3},
};

// todo: Fake horizon in background

use crate::{render, ui, SerialInterface, State, READ_INTERVAL_MS};

pub const BACKGROUND_COLOR: (f32, f32, f32) = (0.9, 0.9, 0.9);
const WINDOW_TITLE: &str = "Corvus Preflight";
const WINDOW_WIDTH: f32 = 900.;
const WINDOW_HEIGHT: f32 = 800.;

// fn make_render_handler() -> impl FnMut(&mut State, &mut Scene) -> bool {
fn render_handler(state: &mut State, scene: &mut Scene, dt: f32) -> EngineUpdates {
    // move |state: &mut State, scene: &mut Scene| {
    let mut engine_updates = EngineUpdates::default();

    if state.last_fc_query.elapsed().as_millis() > READ_INTERVAL_MS {
        // todo: Troubleshooting acces is denied error.
        state.interface = SerialInterface::new();

        state.last_fc_query = Instant::now();

        match &state.interface.serial_port {
            Some(_port) => {
                // state.read_all().unwrap();
                state.read_all().ok(); // todo temp!

                scene.entities[0].orientation = state.attitude;

                scene.entities[1].orientation = state.attitude_commanded;

                state.connected_to_fc = true;
                state.last_fc_response = Instant::now();

                engine_updates.entities = true;
            }
            None => {}
        }

        // todo: Don't hard-code this: Use a const etc for the thresh
        if (Instant::now() - state.last_fc_response) > Duration::from_millis(500) {
            state.connected_to_fc = false;
        };
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

fn make_aircraft_mesh() -> graphics::Mesh {
    // todo: For now, triangular
    let height = 0.2;

    // Top face
    let fwd_top = [0., height, 1.];
    let al_top = [-1., height, -1.];
    let ar_top = [1., height, -1.];

    // Bottom face
    let fwd_bt = [0., -height, 1.];
    let al_bt = [-1., -height, -1.];
    let ar_bt = [1., -height, -1.];

    // Normal vectors
    let aft = Vec3::new(0., 0., -1.);
    let fl = Vec3::new(-1., -1., 0.).to_normalized();
    let fr = Vec3::new(1., 1., 0.).to_normalized();
    let t = Vec3::new(0., 1., 0.);
    let b = Vec3::new(0., -1., 0.);

    let vertices = vec![
        // Top
        Vertex::new(fwd_top, t),
        Vertex::new(al_top, t),
        Vertex::new(ar_top, t),
        // Bottom
        Vertex::new(fwd_bt, b),
        Vertex::new(al_bt, b),
        Vertex::new(ar_bt, b),
        // Aft
        Vertex::new(al_top, aft),
        Vertex::new(al_bt, aft),
        Vertex::new(ar_bt, aft),
        Vertex::new(ar_top, aft),
        // Front left
        Vertex::new(al_top, fl),
        Vertex::new(fwd_top, fl),
        Vertex::new(fwd_bt, fl),
        Vertex::new(al_bt, fl),
        // Front right
        Vertex::new(ar_top, fr),
        Vertex::new(ar_bt, fr),
        Vertex::new(fwd_bt, fr),
        Vertex::new(fwd_top, fr),
    ];

    // Make triangles from indices
    let faces = [
        [0, 1, 2],    // t
        [3, 4, 5],    // b
        [6, 7, 8],    // aft 1
        [7, 8, 9],    // aft 2
        [10, 11, 12], // fl1
        [11, 12, 13], // fl2
        [14, 15, 16], // fr1
        [15, 16, 17], //fr2
    ];

    let mut indices = Vec::new();
    for face in &faces {
        indices.append(&mut vec![
            face[0], face[1], face[2], face[0], face[2], face[3],
        ]);
    }

    graphics::Mesh {
        vertices,
        indices,
        material: 0,
    }
}

/// The entry point for our renderer.
pub fn run(state: State) {
    let entities = vec![
        // Aircraft estimated attitude
        Entity::new(
            0,
            Vec3::new(-6., 0., 0.),
            Quaternion::new_identity(),
            1.,
            (1., 0., 1.),
            1.,
        ),
        // Commanded attitutde
        Entity::new(
            0,
            Vec3::new(6., 0., 0.),
            Quaternion::new_identity(),
            1.,
            (1., 0., 1.),
            1.,
        ),
    ];

    let aircraft_mesh = Mesh::new_box(1., 1., 1.);

    let scene = Scene {
        // todo: Change these meshes A/R.
        meshes: vec![aircraft_mesh, Mesh::new_sphere(1., 50, 50)],
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
            position: Vec3::new(0., 0., -10.),
            ..Default::default()
        },
    };

    let input_settings = InputSettings::default();
    let ui_settings = UiSettings {
        layout: UiLayout::Bottom,
        size: 0., // todo: Bad API here.
        icon_path: None,
    };

    graphics::run(
        state,
        scene,
        input_settings,
        ui_settings,
        render_handler,
        event_handler,
        ui::run,
        include_str!("shader_compute.wgsl"),
    );
}

// todo: Use state cam? Should it be in `Scene`?
