extern crate nalgebra as nalgebra;
extern crate ncollide3d;
extern crate nphysics3d;
extern crate nphysics_testbed3d;
extern crate nstc;

use nalgebra::{Isometry3, Point3, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::object::{BodyHandle, Material};
use nphysics3d::world::World;
use nphysics_testbed3d::Testbed;
use nstc::actor::Actor;

const COLLIDER_MARGIN: f32 = 0.01;

fn main() {
    /*
     * World
     */
    let mut world = World::new();
    world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

    /*
     * Ground.
     */
    let ground_size = 50.0;
    let ground_shape =
        ShapeHandle::new(Cuboid::new(Vector3::repeat(ground_size - COLLIDER_MARGIN)));
    let ground_pos = Isometry3::new(Vector3::y() * -ground_size, nalgebra::zero());

    world.add_collider(
        COLLIDER_MARGIN,
        ground_shape,
        BodyHandle::ground(),
        ground_pos,
        Material::default(),
    );

    /*
     * Create a simple actor
     */
    let actor = nstc::actor::BlockBody::new(Vector3::new(5.0, 1.0, 2.0));
    nstc::simulate::build_body(actor.body(), &mut world, COLLIDER_MARGIN);

    /*
     * Set up the testbed.
     */
    let mut testbed = Testbed::new(world);
    // testbed.hide_performance_counters();
    testbed.look_at(Point3::new(-4.0, 1.0, -4.0), Point3::new(0.0, 1.0, 0.0));
    testbed.run();
}
