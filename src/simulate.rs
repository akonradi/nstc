use nalgebra::{Isometry3, Real};
use nphysics3d::object::Material;
use nphysics3d::volumetric::Volumetric;
use nphysics3d::world::World;

fn add_body_part<N: Real>(
    part: &super::actor::BodyPart<N>,
    world: &mut World<N>,
    collider_margin: N,
) {
    let geom = &part.geometry;
    let inertia = geom.inertia(N::one());
    let center_of_mass = geom.center_of_mass();

    // TODO fix this
    let zero = ::nalgebra::zero();
    let pos = Isometry3::new(zero, zero);
    let handle = world.add_rigid_body(pos, inertia, center_of_mass);

    world.add_collider(
        collider_margin,
        geom.clone(),
        handle,
        Isometry3::identity(),
        Material::default(),
    );

    for _child in part.children.iter() {
        panic!("children not supported");
    }
}

pub fn build_body<N: Real>(body: &super::actor::Body<N>, world: &mut World<N>, collider_margin: N) {
    print!("Building body {:?}", body);
    add_body_part(&body.tree, world, collider_margin)
}
