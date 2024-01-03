use crate::middleware::{RapierPhysicsCollider, RapierPhysicsRigidBody, RapierPhysicsState};
use godot::engine::notify::Node3DNotification;
use godot::engine::Node3D;
use godot::prelude::*;

#[godot_api]
impl INode3D for RapierPhysicsCollider {
    fn init(node_3d: Base<Node3D>) -> Self {
        Self::new(node_3d)
    }

    fn on_notification(&mut self, what: Node3DNotification) {
        match what {
            Node3DNotification::EnterTree => self.enter_tree(),
            /*
            FIXME: freezes the application due to runtime double borrow of Gd smart pointer
            maybe this is a bug in the godot-rust bindings or in godot?
            https://github.com/godot-rust/gdext/pull/501
            */
            Node3DNotification::ExitTree => self.exit_tree(),
            _ => {}
        };
    }
}

#[godot_api]
impl INode3D for RapierPhysicsRigidBody {
    fn init(node_3d: Base<Node3D>) -> Self {
        Self::new(node_3d)
    }

    fn on_notification(&mut self, what: Node3DNotification) {
        match what {
            Node3DNotification::PhysicsProcess => self.render(),
            Node3DNotification::Ready => self.ready(),
            Node3DNotification::EnterTree => self.enter_tree(),
            Node3DNotification::ExitTree => self.exit_tree(),
            _ => {}
        };
    }
}

#[godot_api]
impl INode3D for RapierPhysicsState {
    fn init(node_3d: Base<Node3D>) -> Self {
        Self::new(node_3d)
    }

    fn on_notification(&mut self, what: Node3DNotification) {
        match what {
            Node3DNotification::PhysicsProcess => self.physics_process(),
            Node3DNotification::Ready => self.ready(),
            Node3DNotification::EnterTree => self.enter_tree(),
            _ => {}
        }
    }
}
