//! This module contains code related to the 3D render (Aircraft attitude depiction etc)

use std::{
    boxed::Box,
    f32::consts::TAU,
    sync::atomic::{AtomicUsize, Ordering},
    time::Duration,
    time::Instant,
};

use graphics::{
    self, Camera, DeviceEvent, EngineUpdates, Entity, InputSettings, LightType, Lighting, Mesh,
    PointLight, Scene, UiLayout, UiSettings, Vertex,
};

use obj;

use lin_alg2::{
    self,
    f32::{Quaternion, Vec3},
};

use pc_interface_shared::{
    ConnectionStatus, ConnectionType, Port, SerialInterface, DISCONNECTED_TIMEOUT_MS,
};

// todo: Fake horizon in background

use crate::{render, ui, State, READ_INTERVAL_MS};

pub const BACKGROUND_COLOR: (f32, f32, f32) = (0.9, 0.9, 0.9);
const WINDOW_TITLE: &str = "Corvus Preflight";
const WINDOW_WIDTH: f32 = 1_400.;
const WINDOW_HEIGHT: f32 = 1_000.;

/// Convert from the coordinate system we use on the aircraft, to the one used by the rendering
/// system.
fn convert_quat_coords(quat_in: Quaternion) -> Quaternion {
    Quaternion {
        // w: quat_in.w,
        // x: -quat_in.y,
        // y: quat_in.z,
        // z: -quat_in.x,
        // todo: if you end up with this setup, remove this fn.
        w: quat_in.w,
        x: quat_in.x,
        y: quat_in.y,
        z: quat_in.z,
    }
}

// fn make_render_handler() -> impl FnMut(&mut State, &mut Scene) -> bool {
fn render_handler(state: &mut State, scene: &mut Scene, dt: f32) -> EngineUpdates {
    // move |state: &mut State, scene: &mut Scene| {
    let mut engine_updates = EngineUpdates::default();

    let now = Instant::now();

    // Request config data
    if state.common.last_query.elapsed().as_millis() > READ_INTERVAL_MS {
        state.common.last_query = now;

        // todo: COnnect alsways to TS problems; get rid of this.
        state.common.interface = SerialInterface::connect();

        if state.common.interface.serial_port.is_none() {
            // println!("No port found; re-opening");
            state.common.interface = SerialInterface::connect();
        }

        match state.read_all() {
            Ok(_) => {
                // println!("Att: {:?}", state.attitude);
                scene.entities[0].orientation = convert_quat_coords(state.attitude);

                scene.entities[1].orientation = convert_quat_coords(state.attitude_commanded);

                state.common.last_response = Instant::now();
                state.common.connection_status = ConnectionStatus::Connected;

                engine_updates.entities = true;
            }
            Err(e) => {
                state.common.interface = SerialInterface::connect();
            }
        }
    }

    if (now - state.common.last_response) > Duration::from_millis(DISCONNECTED_TIMEOUT_MS) {
        state.common.connection_status = ConnectionStatus::NotConnected;
    };

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
    let fwd_top = [0., height * 3., 4.];
    let al_top = [-1., height, -1.];
    let ar_top = [1., height, -1.];

    // Bottom face
    let fwd_bt = [0., -height, 4.];
    let al_bt = [-1., -height, -1.];
    let ar_bt = [1., -height, -1.];

    // a point on top
    // let top_pt = [0., height * 1.3, 0.5];

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
        Vertex::new(ar_bt, b),
        Vertex::new(al_bt, b),
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
        [6, 7, 9],    // aft 1
        [7, 8, 9],    // aft 2
        [10, 11, 12], // fl1
        [10, 12, 13], // fl2
        [14, 16, 17], // fr1
        [14, 15, 16], // fr2
    ];

    let mut indices = Vec::new();
    for face in &faces {
        indices.append(&mut vec![face[0], face[1], face[2]]);
    }

    graphics::Mesh {
        vertices,
        indices,
        material: 0,
    }
}

fn load_aircraft_mesh() -> graphics::Mesh {
    let source = include_bytes!("../resources/plane_model.obj");
    let data = obj::ObjData::load_buf(&source[..]).unwrap();

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    for object in data.objects {
        for group in object.groups {
            let mut ind = 0;
            for poly in group.polys {
                for end_index in 2..poly.0.len() {
                    for &index in &[0, end_index - 1, end_index] {
                        let obj::IndexTuple(position_id, _texture_id, normal_id) = poly.0[index];

                        // todo...
                        indices.push(ind);
                        ind += 1;

                        // vertices.push(Vertex {
                        //     pos: data.position[position_id],
                        //     normal: data.normal[normal_id.unwrap()],
                        // })

                        let normal = data.normal[normal_id.unwrap()];

                        vertices.push(Vertex::new(
                            data.position[position_id],
                            Vec3::new(normal[0], normal[1], normal[2]),
                        ));
                    }
                }
            }
        }
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
            Vec3::new(0., 2., 0.),
            Quaternion::new_identity(),
            0.5,
            (1., 0., 1.),
            1.,
        ),
        // Commanded attitutde
        Entity::new(
            0,
            Vec3::new(0., -2., 0.),
            Quaternion::new_identity(),
            0.5,
            (1., 0., 1.),
            1.,
        ),
    ];

    // let aircraft_mesh = Mesh::new_box(1., 1., 1.);
    // let aircraft_mesh = make_aircraft_mesh();
    let aircraft_mesh = load_aircraft_mesh();

    let scene = Scene {
        // todo: Change these meshes A/R.
        meshes: vec![aircraft_mesh, Mesh::new_sphere(1., 50, 50)],
        entities,
        lighting: Lighting {
            ambient_color: [1., 1., 0., 1.],
            ambient_intensity: 0.15,
            point_lights: vec![
                PointLight {
                    type_: LightType::Omnidirectional,
                    position: Vec3::new(0., 20., -5.),
                    diffuse_color: [1., 1., 1., 1.],
                    specular_color: [1., 1., 1., 1.],
                    diffuse_intensity: 200.,
                    specular_intensity: 200.,
                },
                PointLight {
                    type_: LightType::Omnidirectional,
                    position: Vec3::new(0., -20., -5.),
                    diffuse_color: [1., 1., 1., 1.],
                    specular_color: [1., 1., 1., 1.],
                    diffuse_intensity: 200.,
                    specular_intensity: 200.,
                },
            ],
        },
        background_color: render::BACKGROUND_COLOR,
        window_size: (WINDOW_WIDTH, WINDOW_HEIGHT),
        window_title: WINDOW_TITLE.to_owned(),
        camera: Camera {
            position: Vec3::new(0., 0., -10.),
            fov_y: TAU / 8.,
            ..Default::default()
        },
    };

    let input_settings = InputSettings::default();

    let ui_settings = UiSettings {
        layout: UiLayout::Left,
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
