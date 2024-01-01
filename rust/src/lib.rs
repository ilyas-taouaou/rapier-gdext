use godot::engine::{
    BoxShape3D, CapsuleShape3D, CylinderShape3D, Node3D, ProjectSettings, Shape3D, SphereShape3D,
};
use godot::prelude::*;
use rapier3d_f64::na;
use rapier3d_f64::prelude::*;
use std::ops::{Deref, DerefMut};

struct GVector(Vector<Real>);

impl Deref for GVector {
    type Target = Vector<Real>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for GVector {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl From<Vector3> for GVector {
    fn from(value: Vector3) -> Self {
        GVector(Vector::<Real>::new(
            value.x as Real,
            value.y as Real,
            value.z as Real,
        ))
    }
}

impl From<Vector<Real>> for GVector {
    fn from(value: Vector<Real>) -> Self {
        GVector(value)
    }
}

impl Into<Vector<Real>> for GVector {
    fn into(self) -> Vector<Real> {
        self.0
    }
}

impl Into<Vector3> for GVector {
    fn into(self) -> Vector3 {
        Vector3 {
            x: self.x as real,
            y: self.y as real,
            z: self.z as real,
        }
    }
}

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
            let half_size: GVector = (shape.get_size() / 2.0).into();
            ColliderBuilder::cuboid(half_size.x, half_size.y, half_size.z)
        } else if let Ok(shape) = shape.clone().try_cast::<SphereShape3D>() {
            let radius = shape.get_radius();
            ColliderBuilder::ball(radius as Real)
        } else if let Ok(shape) = shape.clone().try_cast::<CapsuleShape3D>() {
            let radius = shape.get_radius() as Real;
            let height = shape.get_height() as Real;
            ColliderBuilder::capsule_y(height / 2.0, radius)
        } else if let Ok(shape) = shape.clone().try_cast::<CylinderShape3D>() {
            let radius = shape.get_radius() as Real;
            let height = shape.get_height() as Real;
            ColliderBuilder::cylinder(height / 2.0, radius)
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
}

#[godot_api]
impl INode3D for RapierPhysicsRigidBody {
    fn init(node_3d: Base<Node3D>) -> Self {
        Self::new(node_3d)
    }

    fn physics_process(&mut self, _delta: f64) {
        let binding = self.physics_state.as_ref().unwrap().bind();
        let rigid_body = binding.inner.query_rigid_body(self.handle.unwrap());
        let position: GVector = (*rigid_body.translation()).into();
        let rotation = rigid_body.rotation();
        self.node_3d.set_global_position(position.into());
        self.node_3d.set_quaternion(Quaternion::new(
            rotation.i as real,
            rotation.j as real,
            rotation.k as real,
            rotation.w as real,
        ));
    }

    fn enter_tree(&mut self) {
        let gvector: GVector = self.node_3d.get_global_position().into();
        let rigid_body = if self.fixed {
            RigidBodyBuilder::fixed()
        } else {
            RigidBodyBuilder::dynamic()
        }
        .translation(gvector.into())
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

    fn step(&mut self, delta: f64, time_scale: f64) {
        self.integration_parameters.dt = delta;
        for _ in 0..time_scale as usize {
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
        let time_scale_fraction = time_scale.fract();
        if !time_scale_fraction.is_zero_approx() {
            self.integration_parameters.dt = delta * time_scale.fract();
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
        Self {
            gravity: ProjectSettings::singleton()
                .get_setting("physics/3d/default_gravity_vector".into())
                .to::<Vector3>()
                * ProjectSettings::singleton()
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

    fn physics_process(&mut self, delta: f64) {
        self.inner.step(delta, self.time_scale);
    }

    fn enter_tree(&mut self) {
        let x: GVector = self.gravity.into();
        self.inner.gravity = x.into();
    }
}

struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}
