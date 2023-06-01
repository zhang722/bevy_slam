use std::{
    time,
    error::Error,
};

use opencv::{
    core,
    features2d,
    calib3d, prelude::{DescriptorMatcherTrait, KeyPointTraitConst},
};
use nalgebra as na;

#[derive(Clone)]
pub struct Frame {
    pub timestamp: time::Duration,
    pub img: core::Mat,
    pub keypoints: core::Vector<core::KeyPoint>,
    pub descriptors: core::Mat,
    pub pose: na::Isometry3<f64>, 
}

impl Frame {
    pub fn new(
        timestamp: time::Duration,
        img: core::Mat,
        keypoints: core::Vector<core::KeyPoint>,
        descriptors: core::Mat,
        pose: na::Isometry3<f64>,
    ) -> Self {
        Frame {
            timestamp,
            img,
            keypoints,
            descriptors,
            pose,
        }
    }

    pub fn default() -> Self {
        Frame {
            timestamp: time::Duration::from_secs(0),
            img: core::Mat::default(),
            keypoints: core::Vector::default(),
            descriptors: core::Mat::default(),
            pose: na::Isometry3::identity(),
        }
    }

    pub fn match_other(&self, other: &Self) -> Result<Vec<core::DMatch>, Box<dyn Error>> {
        // 创建 BFMatcher
        let mut bf_matcher = match features2d::BFMatcher::create(core::NORM_HAMMING, false) {
            Ok(bf_matcher) => bf_matcher,
            Err(e) => {
                println!("Create BFMatcher failed: {}", e);
                return Err(Box::new(e));
            },
        };

        bf_matcher.add(&self.descriptors)?;
        let mut matches = core::Vector::<core::DMatch>::default();

        bf_matcher.match_(&other.descriptors, &mut matches, &core::Mat::default())?;

        let matches = matches.into_iter().filter(|m| m.distance < 30.0).collect::<Vec<_>>();

        let (points1, points2) = matches2points(&matches, &self.keypoints, &other.keypoints)?;

        let mut mask: core::Vector<u8> = core::Vector::default();
        let _matrix = calib3d::find_fundamental_mat(
            &points1, 
            &points2, 
            calib3d::FM_RANSAC, 1.0, 0.99, &mut mask)?;
        
        let matches = matches.into_iter().enumerate().filter(|m| mask.get(m.0).unwrap() != 0).map(|m| m.1).collect::<Vec<_>>();

        Ok(matches)
    }

    pub fn parallax_other(&self, other: &Self, matches: &Vec<core::DMatch>) -> Result<f64, Box<dyn Error>> {
        let (points1, points2) = matches2points(&matches, &self.keypoints, &other.keypoints)?;
        let parallax: f64 = points1.iter().zip(points2.iter())
            .fold(0.0, |acc, (x, y)| acc + ((x.x - y.x).powi(2) + (x.y - y.y).powi(2)) as f64);

        Ok(parallax / matches.len() as f64)
    }
}

pub fn matches2points(
    matches: &Vec<core::DMatch>, 
    keypoints1: &core::Vector<core::KeyPoint>,
    keypoints2: &core::Vector<core::KeyPoint>
) -> Result<(core::Vector<core::Point2f>, core::Vector<core::Point2f>), Box<dyn Error>> {
    let mut points1 = core::Vector::<core::Point2f>::default();
    let mut points2 = core::Vector::<core::Point2f>::default();
    for m in matches {
        let pt1 = keypoints1.get(m.train_idx as usize)?.pt();
        let pt2 = keypoints2.get(m.query_idx as usize)?.pt();
        points1.push(pt1);
        points2.push(pt2);
    }
    Ok((points1, points2))
}

pub fn matches2indices(matches: &Vec<core::DMatch>) -> (Vec<usize>, Vec<usize>) {
    (matches.iter().map(|m| m.train_idx as usize).collect::<Vec<_>>(),
    matches.iter().map(|m| m.query_idx as usize).collect::<Vec<_>>())
}