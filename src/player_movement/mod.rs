use avian3d::{
    math::{AdjustPrecision, Scalar, Vector},
    prelude::*,
};
use bevy::prelude::*;

pub struct PlayerMovementPlugin;

#[derive(Component)]
pub struct PlayerMovement {
    pub acceleration: f32,
    pub air_acceleration: f32,
    pub jump_force: f32,
}

#[derive(Component)]
pub struct MaxSlopeAngle(pub f32);

#[derive(Component)]
pub struct Grounded;

#[derive(Component)]
pub struct KinematicGravity(pub f32);

#[derive(Component)]
pub struct GroundFriction(pub f32);

#[derive(Component)]
pub struct HoverSpring {
    target_height: f32,
    dampening_factor: f32,
    strength_factor: f32,
}

impl HoverSpring {
    pub fn new(target_height: f32, dampening_factor: f32, strength_factor: f32) -> HoverSpring {
        HoverSpring {
            target_height,
            dampening_factor,
            strength_factor,
        }
    }
}

impl PlayerMovement {
    pub fn new(acceleration: f32, air_acceleration: f32, jump_force: f32) -> PlayerMovement {
        PlayerMovement {
            acceleration,
            air_acceleration,
            jump_force,
        }
    }
}

impl Plugin for PlayerMovementPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            Update,
            (
                handle_spring,
                update_grounded,
                handle_movement,
                handle_friction,
                handle_gravity,
                handle_jump,
            ),
        )
        .add_systems(PostProcessCollisions, kinematic_controller_collisions);
    }
}

fn handle_spring(
    mut commands: Commands,
    spatial_query: SpatialQuery,
    mut springs: Query<(Entity, &Transform, &mut LinearVelocity, &HoverSpring)>,
    time: Res<Time>,
) {
    for (entity, transform, mut vel, spring) in springs.iter_mut() {
        let maybe_hit = spatial_query.cast_ray(
            transform.translation,
            transform.down(),
            spring.target_height,
            false,
            &SpatialQueryFilter::from_excluded_entities([entity]),
        );
        if let Some(hit) = maybe_hit {
            let diff = spring.target_height - hit.distance;
            vel.y += diff * spring.strength_factor * time.delta_secs();
            commands.entity(entity).insert(Grounded);
        } else {
            commands.entity(entity).remove::<Grounded>();
        }
    }
}

fn handle_movement(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<(
        &Transform,
        &mut LinearVelocity,
        &PlayerMovement,
        Option<&Grounded>,
    )>,
    time: Res<Time>,
) {
    let mut move_input = Vec2::ZERO;
    if keyboard.pressed(KeyCode::KeyA) {
        move_input.x = -1.;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        move_input.x = 1.;
    }
    if keyboard.pressed(KeyCode::KeyW) {
        move_input.y = 1.;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        move_input.y = -1.;
    }
    for (transform, mut vel, movement, maybe_grounded) in query.iter_mut() {
        let mut world_input = transform.forward() * move_input.y + transform.right() * move_input.x;
        if world_input != Vec3::ZERO {
            world_input = world_input.normalize();
        }
        let accel = if maybe_grounded.is_some()
            || (transform.forward().dot(world_input) < 0. && vel.0.dot(world_input) < 0.)
        {
            movement.acceleration
        } else {
            movement.air_acceleration
        };
        vel.0 += world_input * accel * time.delta_secs();
    }
}

fn handle_jump(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&mut LinearVelocity, &PlayerMovement), With<Grounded>>,
) {
    if keyboard.just_pressed(KeyCode::Space) {
        for (mut vel, pm) in query.iter_mut() {
            vel.0.y = pm.jump_force;
        }
    }
}

fn handle_gravity(mut query: Query<(&mut LinearVelocity, &KinematicGravity)>, time: Res<Time>) {
    for (mut vel, gravity) in query.iter_mut() {
        vel.0.y -= gravity.0 * time.delta_secs();
    }
}

fn handle_friction(
    mut character_controllers: Query<
        (&mut LinearVelocity, &GroundFriction),
        (With<RigidBody>, With<PlayerMovement>, With<Grounded>),
    >,
) {
    for (mut vel, ground_friction) in character_controllers.iter_mut() {
        vel.0 *= 1. - ground_friction.0;
    }
}

fn kinematic_controller_collisions(
    collisions: Res<Collisions>,
    bodies: Query<&RigidBody>,
    collider_parents: Query<&ColliderParent, Without<Sensor>>,
    mut character_controllers: Query<
        (
            &mut Position,
            &Rotation,
            &mut LinearVelocity,
            Option<&MaxSlopeAngle>,
        ),
        (With<RigidBody>, With<PlayerMovement>),
    >,
    time: Res<Time>,
) {
    // Iterate through collisions and move the kinematic body to resolve penetration
    for contacts in collisions.iter() {
        // Get the rigid body entities of the colliders (colliders could be children)
        let Ok([collider_parent1, collider_parent2]) =
            collider_parents.get_many([contacts.entity1, contacts.entity2])
        else {
            continue;
        };

        // Get the body of the character controller and whether it is the first
        // or second entity in the collision.
        let is_first: bool;

        let character_rb: RigidBody;
        let is_other_dynamic: bool;

        let (mut position, rotation, mut linear_velocity, max_slope_angle) =
            if let Ok(character) = character_controllers.get_mut(collider_parent1.get()) {
                is_first = true;
                character_rb = *bodies.get(collider_parent1.get()).unwrap();
                is_other_dynamic = bodies
                    .get(collider_parent2.get())
                    .is_ok_and(|rb| rb.is_dynamic());
                character
            } else if let Ok(character) = character_controllers.get_mut(collider_parent2.get()) {
                is_first = false;
                character_rb = *bodies.get(collider_parent2.get()).unwrap();
                is_other_dynamic = bodies
                    .get(collider_parent1.get())
                    .is_ok_and(|rb| rb.is_dynamic());
                character
            } else {
                continue;
            };

        // This system only handles collision response for kinematic character controllers.
        if !character_rb.is_kinematic() {
            continue;
        }

        // Iterate through contact manifolds and their contacts.
        // Each contact in a single manifold shares the same contact normal.
        for manifold in contacts.manifolds.iter() {
            let normal = if is_first {
                -manifold.global_normal1(rotation)
            } else {
                -manifold.global_normal2(rotation)
            };

            let mut deepest_penetration: Scalar = Scalar::MIN;

            // Solve each penetrating contact in the manifold.
            for contact in manifold.contacts.iter() {
                if contact.penetration > 0.0 {
                    position.0 += normal * contact.penetration;
                }
                deepest_penetration = deepest_penetration.max(contact.penetration);
            }

            // For now, this system only handles velocity corrections for collisions against static geometry.
            if is_other_dynamic {
                continue;
            }

            // Determine if the slope is climbable or if it's too steep to walk on.
            let slope_angle = normal.angle_between(Vector::Y);
            let climbable = max_slope_angle.is_some_and(|angle| slope_angle.abs() <= angle.0);

            if deepest_penetration > 0.0 {
                // If the slope is climbable, snap the velocity so that the character
                // up and down the surface smoothly.
                if climbable {
                    // Points in the normal's direction in the XZ plane.
                    let normal_direction_xz =
                        normal.reject_from_normalized(Vector::Y).normalize_or_zero();

                    // The movement speed along the direction above.
                    let linear_velocity_xz = linear_velocity.dot(normal_direction_xz);

                    // Snap the Y speed based on the speed at which the character is moving
                    // up or down the slope, and how steep the slope is.
                    //
                    // A 2D visualization of the slope, the contact normal, and the velocity components:
                    //
                    //             ╱
                    //     normal ╱
                    // *         ╱
                    // │   *    ╱   velocity_x
                    // │       * - - - - - -
                    // │           *       | velocity_y
                    // │               *   |
                    // *───────────────────*

                    let max_y_speed = -linear_velocity_xz * slope_angle.tan();
                    linear_velocity.y = linear_velocity.y.max(max_y_speed);
                } else {
                    // The character is intersecting an unclimbable object, like a wall.
                    // We want the character to slide along the surface, similarly to
                    // a collide-and-slide algorithm.

                    // Don't apply an impulse if the character is moving away from the surface.
                    if linear_velocity.dot(normal) > 0.0 {
                        continue;
                    }

                    // Slide along the surface, rejecting the velocity along the contact normal.
                    let impulse = linear_velocity.reject_from_normalized(normal);
                    linear_velocity.0 = impulse;
                }
            } else {
                // The character is not yet intersecting the other object,
                // but the narrow phase detected a speculative collision.
                //
                // We need to push back the part of the velocity
                // that would cause penetration within the next frame.

                let normal_speed = linear_velocity.dot(normal);

                // Don't apply an impulse if the character is moving away from the surface.
                if normal_speed > 0.0 {
                    continue;
                }

                // Compute the impulse to apply.
                let impulse_magnitude =
                    normal_speed - (deepest_penetration / time.delta_secs_f64().adjust_precision());
                let mut impulse = impulse_magnitude * normal;

                // Apply the impulse differently depending on the slope angle.
                if climbable {
                    // Avoid sliding down slopes.
                    linear_velocity.y -= impulse.y.min(0.0);
                } else {
                    // Avoid climbing up walls.
                    impulse.y = impulse.y.max(0.0);
                    linear_velocity.0 -= impulse;
                }
            }
        }
    }
}

fn update_grounded(
    mut commands: Commands,
    mut query: Query<(Entity, &ShapeHits, &Rotation, Option<&MaxSlopeAngle>), With<PlayerMovement>>,
) {
    for (entity, hits, rotation, max_slope_angle) in &mut query {
        // The character is grounded if the shape caster has a hit with a normal
        // that isn't too steep.
        let is_grounded = hits.iter().any(|hit| {
            if let Some(angle) = max_slope_angle {
                (rotation * -hit.normal2).angle_between(Vector::Y).abs() <= angle.0
            } else {
                true
            }
        });

        if is_grounded {
            commands.entity(entity).insert(Grounded);
        } else {
            commands.entity(entity).remove::<Grounded>();
        }
    }
}
