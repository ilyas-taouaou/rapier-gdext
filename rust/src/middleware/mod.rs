use crate::backend::RapierPhysicsStateInner;
use godot::bind::{Export, GodotClass, Property};
use godot::builtin::{real, Quaternion, Vector3};
use godot::engine::{
    BoxShape3D, CapsuleShape3D, CylinderShape3D, ProjectSettings, Shape3D, SphereShape3D,
};
use godot::obj::Gd;
use godot::prelude::{Base, Node3D};
use rapier3d_f64::dynamics::{RigidBodyBuilder, RigidBodyHandle, RigidBodyType};
use rapier3d_f64::geometry::{Collider, ColliderHandle};
use rapier3d_f64::math::{AngVector, Isometry, Real, Vector};
use rapier3d_f64::prelude::ColliderBuilder;

#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct RapierPhysicsCollider {
    handle: Option<ColliderHandle>,
    parent: Option<Gd<RapierPhysicsRigidBody>>,
    #[export]
    shape: Option<Gd<Shape3D>>,
    #[base]
    node_3d: Base<Node3D>,
}

#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct RapierPhysicsState {
    inner: RapierPhysicsStateInner,
    #[export]
    gravity: Vector3,
    #[export]
    time_scale: f64,
    #[base]
    node_3d: Base<Node3D>,
}

#[derive(GodotClass)]
#[class(base=Node3D)]
pub struct RapierPhysicsRigidBody {
    handle: Option<RigidBodyHandle>,
    physics_state: Option<Gd<RapierPhysicsState>>,
    #[export]
    body_type: RapierPhysicsRigidBodyType,
    #[base]
    node_3d: Base<Node3D>,
}

#[derive(Property, Export)]
#[repr(u8)]
enum RapierPhysicsRigidBodyType {
    Dynamic = 0,
    Fixed = 1,
    KinematicPositionBased = 2,
    KinematicVelocityBased = 3,
}
impl RapierPhysicsCollider {
    pub fn new(node_3d: Base<Node3D>) -> Self {
        Self {
            shape: None,
            handle: None,
            parent: None,
            node_3d,
        }
    }

    pub fn enter_tree(&mut self) {
        let position = self.node_3d.get_position();
        let rotation = self.node_3d.get_rotation();
        let shape = self.shape.clone().unwrap();
        let collider = if let Ok(shape) = shape.clone().try_cast::<BoxShape3D>() {
            let half_size = shape.get_size() / 2.0;
            ColliderBuilder::cuboid(half_size.x.into(), half_size.y.into(), half_size.z.into())
        } else if let Ok(shape) = shape.clone().try_cast::<SphereShape3D>() {
            ColliderBuilder::ball(shape.get_radius().into())
        } else if let Ok(shape) = shape.clone().try_cast::<CapsuleShape3D>() {
            ColliderBuilder::capsule_y((shape.get_height() / 2.0).into(), shape.get_radius().into())
        } else if let Ok(shape) = shape.clone().try_cast::<CylinderShape3D>() {
            ColliderBuilder::cylinder((shape.get_height() / 2.0).into(), shape.get_radius().into())
        } else {
            unimplemented!("The collision shape isn't implemented yet")
        }
        .position(Isometry::new(
            Vector::new(position.x.into(), position.y.into(), position.z.into()),
            Vector::new(rotation.x.into(), rotation.y.into(), rotation.z.into()),
        ))
        .build();

        let mut parent = self
            .node_3d
            .get_parent()
            .unwrap()
            .cast::<RapierPhysicsRigidBody>();
        parent.bind_mut().insert_collider(collider);
        self.parent = Some(parent);
    }

    pub fn exit_tree(&mut self) {
        self.parent
            .as_mut()
            .unwrap()
            .bind_mut()
            .remove_collider(self.handle.unwrap());
    }
}

impl RapierPhysicsRigidBody {
    pub fn new(node_3d: Base<Node3D>) -> Self {
        Self {
            body_type: RapierPhysicsRigidBodyType::Fixed,
            handle: None,
            physics_state: None,
            node_3d,
        }
    }

    pub fn insert_collider(&mut self, collider: Collider) -> ColliderHandle {
        let mut state_mut = self.physics_state.as_mut().unwrap().bind_mut();
        state_mut
            .inner
            .insert_collider(collider, self.handle.unwrap())
    }

    pub fn remove_collider(&mut self, collider: ColliderHandle) {
        let mut state_mut = self.physics_state.as_mut().unwrap().bind_mut();
        state_mut.inner.remove_collider(collider);
    }

    pub fn render(&mut self) {
        let binding = self.physics_state.as_ref().unwrap().bind();
        let rigid_body = binding.inner.query_rigid_body(self.handle.unwrap());
        let translation = rigid_body.translation();
        let rotation = rigid_body.rotation();
        self.node_3d.set_global_position(Vector3::new(
            translation.x as real,
            translation.y as real,
            translation.z as real,
        ));
        self.node_3d.set_quaternion(Quaternion::new(
            rotation.i as real,
            rotation.j as real,
            rotation.k as real,
            rotation.w as real,
        ));
    }

    pub fn enter_tree(&mut self) {
        let position = self.node_3d.get_global_position();
        let rotation = self.node_3d.get_rotation();
        let rigid_body = RigidBodyBuilder::new(match self.body_type {
            RapierPhysicsRigidBodyType::Dynamic => RigidBodyType::Dynamic,
            RapierPhysicsRigidBodyType::Fixed => RigidBodyType::Fixed,
            RapierPhysicsRigidBodyType::KinematicPositionBased => {
                RigidBodyType::KinematicPositionBased
            }
            RapierPhysicsRigidBodyType::KinematicVelocityBased => {
                RigidBodyType::KinematicVelocityBased
            }
        })
        .translation(Vector::new(
            position.x.into(),
            position.y.into(),
            position.z.into(),
        ))
        .rotation(AngVector::new(
            rotation.x.into(),
            rotation.y.into(),
            rotation.z.into(),
        ))
        .build();
        let mut state = self
            .node_3d
            .get_parent()
            .unwrap()
            .cast::<RapierPhysicsState>();
        {
            let mut state_mut = state.bind_mut();
            self.handle = Some(state_mut.inner.insert_rigid_body(rigid_body));
        }
        self.physics_state = Some(state);
    }

    pub fn exit_tree(&mut self) {
        let mut state_mut = self.physics_state.as_mut().unwrap().bind_mut();
        state_mut.inner.remove_rigid_body(self.handle.unwrap());
    }

    pub fn ready(&mut self) {
        self.node_3d.set_physics_process(true);
    }
}

impl RapierPhysicsState {
    pub fn new(node_3d: Base<Node3D>) -> Self {
        let project_settings = ProjectSettings::singleton();
        Self {
            gravity: project_settings
                .get_setting("physics/3d/default_gravity_vector".into())
                .to::<Vector3>()
                * project_settings
                    .get_setting("physics/3d/default_gravity".into())
                    .to::<real>(),
            time_scale: 1.0,
            inner: RapierPhysicsStateInner::new(),
            node_3d,
        }
    }

    pub fn physics_process(&mut self) {
        self.inner.scaled_step(
            self.node_3d.get_physics_process_delta_time(),
            self.time_scale,
        );
    }

    pub fn enter_tree(&mut self) {
        self.inner.gravity = Vector::<Real>::new(
            self.gravity.x.into(),
            self.gravity.y.into(),
            self.gravity.z.into(),
        )
    }

    pub fn ready(&mut self) {
        self.node_3d.set_physics_process(true);
    }
}
