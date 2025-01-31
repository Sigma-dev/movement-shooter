use avian3d::{
    prelude::{
        Collider, ColliderConstructor, ColliderConstructorHierarchy, Friction, Gravity,
        LinearDamping, LockedAxes, Mass, PhysicsDebugPlugin, RigidBody,
    },
    PhysicsPlugins,
};
use bevy::{math::VectorSpace, prelude::*};
use camera::{FpsCamera, FpsCameraPlugin};
use player_movement::{
    GroundFriction, HoverSpring, KinematicGravity, PlayerMovement, PlayerMovementPlugin,
};

mod camera;
mod player_movement;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
        ))
        .add_plugins((FpsCameraPlugin, PlayerMovementPlugin))
        .add_systems(Startup, setup)
        .add_systems(Update, debug_update)
        .run();
}

fn setup(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.spawn((
        SceneRoot(asset_server.load("models/levels/level1.glb#Scene0")),
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh),
        RigidBody::Static,
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: light_consts::lux::OVERCAST_DAY,
            shadows_enabled: true,
            ..default()
        },
        Transform {
            translation: Vec3::new(0.0, 2.0, 0.0),
            rotation: Quat::from_rotation_x(-3.14 / 4.),
            ..default()
        },
    ));
    commands
        .spawn((
            PlayerMovement::new(100., 10., 10.),
            RigidBody::Kinematic,
            Mass(70.),
            Collider::capsule(0.15, 1.2),
            Transform::from_xyz(0., 1., 0.),
            LockedAxes::ROTATION_LOCKED,
            GroundFriction(0.1),
            HoverSpring::new(1.2, 0.95, 100.),
            KinematicGravity(15.),
            Friction::ZERO,
        ))
        .with_child((
            FpsCamera::new(0.1),
            Camera3d::default(),
            Transform::from_xyz(0.0, 0.6, 0.0),
        ));
    commands.spawn((
        //Camera3d::default(),
        Transform::from_xyz(-10., 0., 0.).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

fn debug_update(query: Query<&GlobalTransform, With<FpsCamera>>, mut gizmos: Gizmos) {
    for q in query.iter() {
        /* gizmos.sphere(
            Isometry3d::from_translation(q.translation()),
            0.1,
            Color::WHITE,
        ); */
    }
}
