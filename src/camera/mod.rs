use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, PrimaryWindow};

pub struct FpsCameraPlugin;

#[derive(Component)]
pub struct FpsCamera {
    pub sensitivity: f32,
}

impl FpsCamera {
    pub fn new(sensitivity: f32) -> FpsCamera {
        FpsCamera { sensitivity }
    }
}

impl Plugin for FpsCameraPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, handle_fps_cameras)
            .add_systems(Startup, setup);
    }
}

fn setup(mut q_windows: Query<&mut Window, With<PrimaryWindow>>) {
    let mut primary_window = q_windows.single_mut();

    primary_window.cursor_options.grab_mode = CursorGrabMode::Locked;
    primary_window.cursor_options.visible = false;
}

fn handle_fps_cameras(
    mut query: Query<(Entity, &FpsCamera, Option<&Parent>)>,
    mut motion_evr: EventReader<MouseMotion>,
    mut transform_query: Query<&mut Transform>,
    time: Res<Time>,
) {
    for (entity, free_cam, maybe_parent) in &mut query {
        for ev in motion_evr.read() {
            let rotation_dir = -ev.delta * free_cam.sensitivity * time.delta_secs();
            if let Some(parent) = maybe_parent {
                let Ok([mut transform, mut parent_transform]) =
                    transform_query.get_many_mut([entity, parent.get()])
                else {
                    continue;
                };
                transform.rotate_axis(Dir3::X, rotation_dir.y);
                parent_transform.rotate_axis(Dir3::Y, rotation_dir.x);
            } else {
                let Ok(mut transform) = transform_query.get_mut(entity) else {
                    continue;
                };
                let right = transform.right();
                transform.rotate_axis(right, rotation_dir.y);
                transform.rotate_axis(Dir3::Y, rotation_dir.x);
            }
        }
    }
}
