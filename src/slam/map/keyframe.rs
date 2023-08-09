use std::time;
use std::cell::RefCell;
use std::rc::Rc;

use opencv::{
    prelude::*,
    core,
};
use nalgebra as na;

use super::super::{
    frame,
    camera,
};
use super::mappoint::*;

pub type KeyFrameId = usize;

#[derive(Clone)]
pub struct KeyFrame {
    pub id: KeyFrameId,
    pub timestamp: time::Duration,
    pub img: Mat,
    pub keypoints: core::Vector<core::KeyPoint>,
    pub descriptors: Mat,
    pub intrinsics: camera::CameraIntrinsics,
    pub pose: na::Isometry3<f64>, 
    pub observations: Vec<Rc<RefCell<MapPoint>>>,
    pub connections: Vec<KeyFrameId>,
}

impl KeyFrame {
    // generator without given id, instead, use generate_id()
    pub fn new(
        timestamp: time::Duration,
        img: Mat,
        keypoints: core::Vector<core::KeyPoint>,
        descriptors: Mat,
        intrinsics: camera::CameraIntrinsics,
        pose: na::Isometry3<f64>,
    ) -> Self {
        let id = super::generate_id();
        KeyFrame {
            id,
            timestamp,
            img,
            keypoints,
            descriptors,
            intrinsics,
            pose,
            observations: Vec::new(),
            connections: Vec::new(),
        }
    }

    pub fn new_with_id(
        id: KeyFrameId,
        timestamp: time::Duration,
        img: Mat,
        keypoints: core::Vector<core::KeyPoint>,
        descriptors: Mat,
        intrinsics: camera::CameraIntrinsics,
        pose: na::Isometry3<f64>,
    ) -> Self {
        KeyFrame {
            id,
            timestamp,
            img,
            keypoints,
            descriptors,
            intrinsics,
            pose,
            observations: Vec::new(),
            connections: Vec::new(),
        }
    }

    pub fn from_frame(
        frame: &frame::Frame,
        intrinsics: &camera::CameraIntrinsics,
    ) -> Self {
        KeyFrame::new(
            frame.timestamp.clone(),
            frame.img.clone(),
            frame.keypoints.clone(),
            frame.descriptors.clone(),
            intrinsics.clone(),
            frame.pose.clone(),
        )
    }

    pub fn add_observation(&mut self, observation: Rc<RefCell<MapPoint>>) {
        self.observations.push(observation);
    }

    pub fn add_observations(&mut self, observations: Vec<Rc<RefCell<MapPoint>>>) {
        self.observations.extend(observations);
    }

    pub fn add_connection(&mut self, id: KeyFrameId) {
        self.connections.push(id);
    }

    pub fn add_connections(&mut self, ids: Vec<KeyFrameId>) {
        self.connections.extend(ids);
    }

    pub fn observation(&self, id: MapPointId) -> Option<Rc<RefCell<MapPoint>>> {
        self.observations.iter().find(|x| x.borrow().id == id).cloned()
    }
}