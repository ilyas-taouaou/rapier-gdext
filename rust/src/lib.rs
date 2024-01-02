use godot::engine::notify::Node3DNotification;
use godot::engine::{
    BoxShape3D, CapsuleShape3D, CylinderShape3D, Node3D, ProjectSettings, Shape3D, SphereShape3D,
};
use godot::prelude::*;
use rapier3d_f64::na;
use rapier3d_f64::prelude::*;

#[derive(GodotClass)]
#[class(base=Node3D)]
struct RapierPhysicsCollider {
    #[export]
    shape: Option<Gd<Shape3D>>,
    handle: Option<ColliderHandle>,
    parent: Option<Gd<RapierPhysicsRigidBody>>,
    #[base]
    node_3d: Base<Node3D>,
}

impl RapierPhysicsCollider {
    fn new(node_3d: Base<Node3D>) -> Self {
        Self {
            shape: None,
            handle: None,
            parent: None,
            node_3d,
        }
    }
}

#[godot_api]
impl INode3D for RapierPhysicsCollider {
    fn init(node_3d: Base<Node3D>) -> Self {
        Self::new(node_3d)
    }

    fn enter_tree(&mut self) {
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
            godot_error!("Rapier Physics: The collision shape isn't implemented yet");
            unimplemented!("The collision shape isn't implemented yet")
        }
        .build();

        let mut parent = self
            .node_3d
            .get_parent()
            .unwrap()
            .cast::<RapierPhysicsRigidBody>();
        parent.bind_mut().insert_collider(collider);
        self.parent = Some(parent);
    }

    fn exit_tree(&mut self) {
        self.parent
            .as_mut()
            .unwrap()
            .bind_mut()
            .remove_collider(self.handle.unwrap());
    }
}

#[derive(GodotClass)]
#[class(base=Node3D)]
struct RapierPhysicsRigidBody {
    #[export]
    fixed: bool,
    handle: Option<RigidBodyHandle>,
    physics_state: Option<Gd<RapierPhysicsState>>,
    #[base]
    node_3d: Base<Node3D>,
}

impl RapierPhysicsRigidBody {
    fn new(node_3d: Base<Node3D>) -> Self {
        Self {
            fixed: false,
            handle: None,
            physics_state: None,
            node_3d,
        }
    }

    fn insert_collider(&mut self, collider: Collider) -> ColliderHandle {
        let mut state_mut = self.physics_state.as_mut().unwrap().bind_mut();
        state_mut
            .inner
            .insert_collider(collider, self.handle.unwrap())
    }

    fn remove_collider(&mut self, collider: ColliderHandle) {
        let mut state_mut = self.physics_state.as_mut().unwrap().bind_mut();
        state_mut.inner.remove_collider(collider);
    }

    fn render(&mut self) {
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
}

#[godot_api]
impl INode3D for RapierPhysicsRigidBody {
    fn init(node_3d: Base<Node3D>) -> Self {
        Self::new(node_3d)
    }

    fn on_notification(&mut self, what: Node3DNotification) {
        match what {
            Node3DNotification::PhysicsProcess => self.render(),
            Node3DNotification::Ready => self.node_3d.set_physics_process(true),
            _ => {}
        };
    }

    fn enter_tree(&mut self) {
        let position = self.node_3d.get_global_position();
        let rigid_body = if self.fixed {
            RigidBodyBuilder::fixed()
        } else {
            RigidBodyBuilder::dynamic()
        }
        .translation(Vector::new(
            position.x.into(),
            position.y.into(),
            position.z.into(),
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
}

#[derive(Default)]
struct RapierPhysicsStateInner {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    gravity: na::SVector<f64, 3>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    physics_hooks: (),
    event_handler: (),
}

impl RapierPhysicsStateInner {
    fn remove_collider(&mut self, collider: ColliderHandle) {
        self.collider_set.remove(
            collider,
            &mut self.island_manager,
            &mut self.rigid_body_set,
            true,
        );
    }
}

#[derive(GodotClass)]
#[class(base=Node3D)]
struct RapierPhysicsState {
    #[export]
    gravity: Vector3,
    #[export]
    time_scale: f64,
    inner: RapierPhysicsStateInner,
    #[base]
    node_3d: Base<Node3D>,
}

impl RapierPhysicsStateInner {
    fn new() -> Self {
        Default::default()
    }

    fn step(&mut self, delta_time: f64) {
        self.integration_parameters.dt = delta_time;
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &self.physics_hooks,
            &self.event_handler,
        );
    }

    fn scaled_step(&mut self, delta: f64, time_scale: f64) {
        (0..time_scale as usize).for_each(|_| self.step(delta));
        let time_scale_fraction = time_scale.fract();
        if !time_scale_fraction.is_zero_approx() {
            self.step(delta * time_scale_fraction);
        }
    }

    fn insert_rigid_body(&mut self, rigid_body: RigidBody) -> RigidBodyHandle {
        self.rigid_body_set.insert(rigid_body)
    }

    fn insert_collider(&mut self, collider: Collider, parent: RigidBodyHandle) -> ColliderHandle {
        self.collider_set
            .insert_with_parent(collider, parent, &mut self.rigid_body_set)
    }

    fn query_rigid_body(&self, body_handle: RigidBodyHandle) -> &RigidBody {
        &self.rigid_body_set[body_handle]
    }
}

impl RapierPhysicsState {
    fn new(node_3d: Base<Node3D>) -> Self {
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
}

#[godot_api]
impl INode3D for RapierPhysicsState {
    fn init(node_3d: Base<Node3D>) -> Self {
        Self::new(node_3d)
    }

    fn on_notification(&mut self, what: Node3DNotification) {
        match what {
            Node3DNotification::PhysicsProcess => {
                self.inner.scaled_step(
                    self.node_3d.get_physics_process_delta_time(),
                    self.time_scale,
                );
            }
            Node3DNotification::Ready => self.node_3d.set_physics_process(true),
            _ => {}
        }
    }

    fn enter_tree(&mut self) {
        self.inner.gravity = Vector::<Real>::new(
            self.gravity.x.into(),
            self.gravity.y.into(),
            self.gravity.z.into(),
        );
    }
}

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}
