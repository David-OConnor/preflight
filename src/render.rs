//! This module contains code related to the 3D render (Aircraft attitude depiction etc)

use graphics::{
    self, lighting, Camera, DeviceEvent, ElementState, Entity, InputSettings, Lighting, Mesh,
    Scene, UiSettings,
};

use lin_alg2::{
    self,
    f32::{Quaternion, Vec3},
};

use crate::{render, ui, State};

pub const BACKGROUND_COLOR: (f32, f32, f32) = (0.9, 0.9, 0.9);

// // todo: Use a mutex instead of static mut here.
// static mut state: State = unsafe {
//     State {
//         cam: render::Camera {
//             position: Vec3 {
//                 x: 0.,
//                 y: 0.,
//                 z: 0.,
//             },
//             orientation: Quaternion {
//                 w: 0.,
//                 x: 0.,
//                 y: 0.,
//                 z: 0.,
//             },
//         },
//     }
// };

/// The entry point for our renderer.
pub unsafe fn run() {
    // let mut state = State::default();
    // let state = Rc::new(Mutex::new(State::default());
    // todo: Initialize our static mut; bit of a hack
    let state = State::default();

    // Render our atoms.
    let entities = vec![Entity::new(
        0,
        Vec3::new(0., 0., 0.),
        Quaternion::new_identity(),
        1.,
        (0., 0., 0.),
        1.,
    )];

    let mut scene = Scene {
        meshes: vec![
            // Mesh::new_tetrahedron(render::SIDE_LEN),
            Mesh::new_box(1., 1., 1.),
        ],
        entities,
        camera: Camera::default(),
        lighting: Lighting::default(),
        background_color: render::BACKGROUND_COLOR,
        window_size: (900., 600.),
        window_title: "Peptide".to_owned(),
    };

    let input_settings = InputSettings::default();
    let ui_settings = UiSettings {};

    graphics::run(
        scene,
        input_settings,
        ui_settings,
        render_handler,
        device_event_handler,
        ui::draw_ui,
    );
}

// todo: Use state cam? Should it be in `Scene`?

fn render_handler() -> Option<Vec<Entity>> {
    // fn render_handler(scene: &mut Scene) -> Option<Vec<Entity>> {
    None
}

fn device_event_handler(
    event: DeviceEvent,
    // state: &mut State,
    scene: &mut Scene,
    dt: f32, // in seconds
) -> bool {
    false
}
