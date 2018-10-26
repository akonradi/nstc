use nalgebra::{Real, Unit, Vector3};
use ncollide3d::shape::{Cuboid, ShapeHandle};
use nphysics3d::math::AngularVector;
use std::fmt;

/// An axial joint used to connect two body parts.
#[derive(Debug)]
pub struct Joint<N: Real> {
    /// The maximum angle allowed by the joint, in radians.
    max_angle_rad: N,
    /// The minimum angle allowed by the joint, in radians.
    min_angle_rad: N,
    /// The angle of the axis of the joint.
    axis: Unit<AngularVector<N>>,
    /// The position on the parent at which the joint attaches.
    ///
    /// The attachment point on the parent goemetry is the furthest-most projection of this vector
    /// from the center of the body.
    parent_attachment: Unit<AngularVector<N>>,

    /// The requested velocity of the joint, or None to allow free rotation.
    angular_velocity: Option<N>,
}

#[derive(Debug)]
pub struct BodyPartChild<N: Real> {
    /// The joint conecting this body part to its parent.
    pub joint: Joint<N>,
    pub part: BodyPart<N>,
}

pub struct BodyPart<N: Real> {
    /// The physical shape of this body part.
    pub geometry: ShapeHandle<N>,
    /// The child body parts of this one.
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

/// The body of an `Actor` impl.
#[derive(Debug)]
pub struct Body<N: Real> {
    /// The parts of the body.
    ///
    /// The body is represented as a tree of parts from parent to children.
    pub tree: BodyPart<N>,
}

pub trait Actor<N: Real> {
    /// Returns the representation of this actor's body.
    ///
    /// The body can change as the actor updates joint velocities. This provides a read-only view
    /// for observers.
    fn body(&self) -> &Body<N>;

    /// Updates the joint velocities for the given timestep.
    ///
    /// This should be called by the simulation environment with strictly monotonically increasing
    /// values of `time`.
    fn step(&mut self, time: N);
}

/// A dead simple Actor impl that doesn't do anything.
#[derive(Debug)]
pub struct BlockBody<N: Real> {
    body_: Body<N>,
}

impl<N: Real> BlockBody<N> {
    /// Creates a new `BlockBody` with the given dimensions.
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
