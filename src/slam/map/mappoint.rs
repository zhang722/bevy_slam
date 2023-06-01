
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

impl MapPointReference {
    pub fn new(
        id: KeyFrameId,
        keypoint: &core::KeyPoint,
        descriptor: &Mat,
    ) -> Self {
        MapPointReference {
            id,
            keypoint: keypoint.clone(),
            descriptor: descriptor.clone(),
        }
    }
    pub fn new_with_kf(
        keyframe: &KeyFrame,
        keypoint: &core::KeyPoint,
        descriptor: &Mat,
    ) -> Self {
        MapPointReference::new(
            keyframe.id,
            keypoint,
            descriptor,
        )
    }
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

    pub fn from_point(
        position: na::Point3<f64>,
        desctriptor: &Mat,
    ) -> Self {
        Self::new(na::Vector3::<f64>::new(position.x, position.y, position.z), desctriptor.clone())
    }

    pub fn add_reference(&mut self, reference: MapPointReference) {
        self.references.push(reference);
    }

    pub fn add_references(&mut self, references: Vec<MapPointReference>) {
        self.references.extend(references);
    }
}