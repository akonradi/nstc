use nalgebra::{Real, Unit, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::math::AngularVector;
use std::fmt;

#[derive(Debug)]
pub struct Joint<N: Real> {
    max_angle_rad: N,
    min_angle_rad: N,
    axis: Unit<AngularVector<N>>,
    parent_attachment: Unit<AngularVector<N>>,

    angular_velocity: Option<N>,
}

#[derive(Debug)]
pub struct BodyPartChild<N: Real> {
    pub joint: Joint<N>,
    pub part: BodyPart<N>,
}

pub struct BodyPart<N: Real> {
    pub geometry: ShapeHandle<N>,
    pub children: Vec<BodyPartChild<N>>,
}

impl<N: Real> fmt::Debug for BodyPart<N> {
    fn fmt<'a>(&self, f: &mut fmt::Formatter<'a>) -> fmt::Result {
        write!(f, "geometry: ");
        let s = &self.geometry;
        if let Some(ref cube) = s.as_shape::<Cuboid<N>>() {
            write!(f, "{:?}", cube);
        } else {
            write!(f, "unknown shape");
        }
        Ok(())
    }
}

#[derive(Debug)]
pub struct Body<N: Real> {
    pub tree: BodyPart<N>,
}

pub trait Actor<N: Real> {
    fn body(&self) -> &Body<N>;

    fn step(&mut self, time: N);
}

#[derive(Debug)]
pub struct BlockBody<N: Real> {
    body_: Body<N>,
}

impl<N: Real> BlockBody<N> {
    pub fn new(size: Vector3<N>) -> Self {
        BlockBody {
            body_: Body {
                tree: BodyPart {
                    geometry: ShapeHandle::new(Cuboid::new(size / (N::one() + N::one()))),
                    children: vec![],
                },
            },
        }
    }
}

impl<N: Real> Actor<N> for BlockBody<N> {
    fn body(&self) -> &Body<N> {
        &self.body_
    }

    fn step(&mut self, _time: N) {}
}
