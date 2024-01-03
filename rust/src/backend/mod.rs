use rapier3d_f64::na;
use rapier3d_f64::prelude::*;

#[derive(Default)]
pub struct RapierPhysicsStateInner {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    pub gravity: na::SVector<f64, 3>,
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
    pub fn new() -> Self {
        Default::default()
    }

    fn step(&mut self) {
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

    pub fn scaled_step(&mut self, delta_time: f64, time_scale: f64) {
        self.integration_parameters.dt = delta_time;
        (0..time_scale as usize).for_each(|_| self.step());
        let time_scale_fraction = time_scale.fract();
        if time_scale_fraction > 0.001 {
            self.integration_parameters.dt = delta_time * time_scale_fraction;
            self.step();
        }
    }

    pub fn insert_rigid_body(&mut self, rigid_body: RigidBody) -> RigidBodyHandle {
        self.rigid_body_set.insert(rigid_body)
    }

    pub fn remove_rigid_body(&mut self, handle: RigidBodyHandle) {
        self.rigid_body_set.remove(
            handle,
            &mut self.island_manager,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            false,
        );
    }

    pub fn insert_collider(
        &mut self,
        collider: Collider,
        parent: RigidBodyHandle,
    ) -> ColliderHandle {
        self.collider_set
            .insert_with_parent(collider, parent, &mut self.rigid_body_set)
    }

    pub fn query_rigid_body(&self, body_handle: RigidBodyHandle) -> &RigidBody {
        &self.rigid_body_set[body_handle]
    }

    pub fn remove_collider(&mut self, collider: ColliderHandle) {
        self.collider_set.remove(
            collider,
            &mut self.island_manager,
            &mut self.rigid_body_set,
            true,
        );
    }
}
