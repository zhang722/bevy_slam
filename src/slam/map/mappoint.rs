
use opencv::{
    prelude::*,
    core,
};
use nalgebra as na;

use super::keyframe::*;

pub type MapPointId = usize;

pub struct MapPointReference {
    pub id: KeyFrameId,
    pub keypoint: core::KeyPoint,
    pub descriptor: Mat,
}

pub struct MapPoint {
    pub id: MapPointId,
    pub position: na::Vector3<f64>,
    pub desctriptor: Mat,
    pub references: Vec<MapPointReference>,
}

impl MapPoint {
    pub fn new(
        position: na::Vector3<f64>,
        desctriptor: Mat,
    ) -> Self {
        let id = super::generate_id();
        MapPoint {
            id,
            position,
            desctriptor,
            references: Vec::new(),
        }
    }

    pub fn new_with_id(
        id: MapPointId,
        position: na::Vector3<f64>,
        desctriptor: Mat,
    ) -> Self {
        MapPoint {
            id,
            position,
            desctriptor,
            references: Vec::new(),
        }
    }

    pub fn add_reference(&mut self, reference: MapPointReference) {
        self.references.push(reference);
    }

    pub fn add_references(&mut self, references: Vec<MapPointReference>) {
        self.references.extend(references);
    }
}