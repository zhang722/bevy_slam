use std::{time, error::Error};
use opencv::{
    prelude::*,
    core,
    features2d,
    imgcodecs,
    imgproc,
    calib3d,
};
use nalgebra as na;

use super::load_data;
use super::camera;
use super::recover_pose;

const GX: usize = 15;
const GY: usize = 10;
const FEATURE_NUM: usize = GX * GY * 6;

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
}

pub struct Tracker {
    pub pose: na::Isometry3<f64>,
    pub init_flag: bool,
    pub last_frame: Frame,
    pub curr_frame: Frame,
    pub camera: camera::CameraIntrinsics,
    pub orb_detector: core::Ptr<features2d::ORB>,
    pub bf_matcher: core::Ptr<features2d::BFMatcher>,
}

impl Tracker {
    pub fn new(
        camera: camera::CameraIntrinsics,
    ) -> Result<Self, Box<dyn Error>> {
        let orb_detector = features2d::ORB::create(
            FEATURE_NUM as i32,
            1.2,
            8,
            31,
            0,
            2,
            features2d::ORB_ScoreType::HARRIS_SCORE, 
            31,
            20,
        ).unwrap();

        // 创建 BFMatcher
        let bf_matcher = match features2d::BFMatcher::create(core::NORM_HAMMING, false) {
            Ok(bf_matcher) => bf_matcher,
            Err(e) => {
                println!("Create BFMatcher failed: {}", e);
                return Err(Box::new(e));
            },
        };

        Ok(Self {
            pose: na::Isometry3::identity(),
            init_flag: false,
            last_frame: Frame::default(),
            curr_frame: Frame::default(),
            camera,
            orb_detector,
            bf_matcher,
        })
    }


    pub fn match_frame(
        &mut self,
    ) -> Result<core::Vector<core::DMatch>, Box<dyn std::error::Error>> {
        opencv::prelude::DescriptorMatcherTrait::clear(&mut self.bf_matcher)?; 

        self.bf_matcher.add(&self.last_frame.descriptors)?;
        let mut matches = core::Vector::<core::DMatch>::default();

        self.bf_matcher.match_(&self.curr_frame.descriptors, &mut matches, &core::Mat::default())?;

        Ok(matches)
    }

    pub fn process_frame(
        &mut self,
    ) -> Result<na::Isometry3<f64>, Box<dyn Error>> {
        let matches = match self.match_frame() {
            Ok(m) => m,
            Err(e) => {
                println!("Match frame failed: {}", e);
                return Err(e);
            },
        };
        let out_img = self.curr_frame.img.clone();
        let mut out_img_color = core::Mat::default();
        imgproc::cvt_color(&out_img, &mut out_img_color, imgproc::COLOR_GRAY2BGR, 0)?;

        let matches = matches.into_iter().filter(|m| m.distance < 30.0).collect::<Vec<_>>();
        let mut points1: core::Vector<core::Point2f> = core::Vector::default();
        let mut points2: core::Vector<core::Point2f> = core::Vector::default();
        for m in &matches {
            let last_pt = self.last_frame.keypoints.get(m.train_idx as usize)?.pt();
            let curr_pt = self.curr_frame.keypoints.get(m.query_idx as usize)?.pt();

            points1.push(last_pt);
            points2.push(curr_pt);
        }
        let mut mask: core::Vector<u8> = core::Vector::default();
        let _matrix = calib3d::find_fundamental_mat(
            &points1, 
            &points2, 
            calib3d::FM_RANSAC, 1.0, 0.99, &mut mask)?;

        let mut inliers1 = core::Vector::<core::Point2f>::default();
        let mut inliers2 = core::Vector::<core::Point2f>::default();
        for (idx, m) in matches.iter().enumerate() {
            if mask.get(idx)? == 0 {
                continue;
            }
            let last_pt = self.last_frame.keypoints.get(m.train_idx as usize)?.pt();
            let curr_pt = self.curr_frame.keypoints.get(m.query_idx as usize)?.pt();

            inliers1.push(last_pt);
            inliers2.push(curr_pt);
        }

        let mut mask: core::Vector<u8> = core::Vector::default();
        // in opencv, find_essential_mat returns Essential Matrix E, 
        // which subjects to x2^T * E * x1 = 0
        // It is important!
        // So E = R_{21} * [t_{21}]_x, not R_{12} * [t_{12}]_x !!!
        // where R_{21} is the rotation matrix from frame 1 to frame 2
        // and [t_{21}]_x is the skew-symmetric matrix of translation vector t_{21}
        let essential_mat = calib3d::find_essential_mat(
            &inliers1, 
            &inliers2, 
            self.camera.fx, 
            core::Point_ { x: self.camera.cx, y: self.camera.cy }, 
            calib3d::RANSAC, 
            0.999, 
            1.0, 
            &mut mask)?;

        // draw matches in one img
        let mut inliers_essential1 = core::Vector::<core::Point2f>::default();
        let mut inliers_essential2 = core::Vector::<core::Point2f>::default();
        for (idx, status) in mask.iter().enumerate() {
            if status == 0 {
                continue;
            }
            let last_pt = inliers1.get(idx)?;
            let curr_pt = inliers2.get(idx)?;
            inliers_essential1.push(last_pt);
            inliers_essential2.push(curr_pt);
            let last_pt = core::Point2i::new(last_pt.x as i32, last_pt.y as i32);
            let curr_pt = core::Point2i::new(curr_pt.x as i32, curr_pt.y as i32);
            imgproc::line(
                &mut out_img_color, 
                last_pt,
                curr_pt,
                core::Scalar::new(255.0, 0.0, 255.0, 255.0),
                2, 
                8, 
                0).unwrap();
        }

        opencv::highgui::imshow("matches", &out_img_color).unwrap();
        opencv::highgui::wait_key(0).unwrap();

        // NOTE: type of mask is Vec<bool>, not opencv::core::Vector<u8>
        // and type of points3d is Vec<na::Point3<f64>>, not opencv::core::Vector<core::Point3f>
        // since we implement the function by ourselves
        let (pose, mask, points3d) = recover_pose::from_essential(
            &essential_mat, 
            &inliers_essential1, 
            &inliers_essential2, 
            &self.camera)?;

        Ok(pose)
    }

    pub fn track(
        &mut self, 
        data: load_data::EurocData
    ) -> Result<na::Isometry3<f64>, Box<dyn Error>> {
        let timestamp = data.timestamp.as_secs_f64();
        let img_name = data.img_name;
        let img = match imgcodecs::imread(&img_name, imgcodecs::IMREAD_GRAYSCALE) {
            Ok(img) => img,
            Err(e) => {
                println!("Read img failed: {}.\n Error: {}", img_name, e);
                return Err(Box::new(e));
            },
        };
        let mut corners = core::Vector::<core::Point2f>::default();
        imgproc::good_features_to_track(
            &img, 
            &mut corners, 
            1000, 
            0.01, 
            10.0, 
            &opencv::core::Mat::default(), 
            3, 
            false, 
            0.04).unwrap(); 
        

        let mut orb_keypoints: core::Vector<core::KeyPoint> = corners.into_iter()
            .map(|f|  
                core::KeyPoint::new_point(
                                    f,
                                    1.0, 
                                    -1.0, 
                                    0.0, 
                                    0, 
                                    -1,).unwrap()
            ).collect();
        let mut orb_desc = core::Mat::default();
        if let Err(e) = self.orb_detector.compute(&img, &mut orb_keypoints, &mut orb_desc) {
            println!("Detect and compute failed: {}", e);
            return Err(Box::new(e));
        }

        self.curr_frame = Frame::new(
            data.timestamp,
            img,
            orb_keypoints,
            orb_desc,
            na::Isometry3::identity(),
        );

        if !self.init_flag {
            self.last_frame = std::mem::replace(&mut self.curr_frame, Frame::default());
            self.init_flag = true;

            return Ok(na::Isometry3::identity());
        }

        println!("img_name: {}", img_name);
        let pose = match self.process_frame() {
            Ok(pose) => pose,
            Err(e) => {
                println!("Process frame failed: {}", e);
                return Err(e);
            },
        };

        self.last_frame = std::mem::replace(&mut self.curr_frame, Frame::default());

        self.pose *= pose;

        Ok(self.pose)
    }
}


pub fn pose_from_essential_mat(
    essential_mat: &core::Mat,
    points1: &core::Vector<core::Point2f>,
    points2: &core::Vector<core::Point2f>,
    intrinsics: &camera::CameraIntrinsics,
) -> Result<na::Isometry3<f64>, Box<dyn Error>> {
    let mut r = core::Mat::default();
    let mut t = core::Mat::default();
    let mut mask: core::Vector<u8> = core::Vector::default();
    let n = calib3d::recover_pose(
        essential_mat, 
        points1, 
        points2, 
        &mut r, 
        &mut t, 
        intrinsics.fx, 
        core::Point2d::new(intrinsics.cx, intrinsics.cy), 
        &mut mask).unwrap();
    println!("len: {}", points1.len());
    println!("inliers: {}", n);

    recover_pose::from_essential(essential_mat, points1, points2, intrinsics)?;

    if (n as f64) / (points1.len() as f64) < 0.5 {
        return Err(Box::new(std::io::Error::new(
            std::io::ErrorKind::Other,
            "Not enough inliers",
        )));
    }
    
    // cv::Mat to na::Isometry3<f64>
    let r = na::Rotation3::<f64>::from_matrix_unchecked(na::Matrix3::new(
        *r.at_2d(0, 0)?, *r.at_2d(0, 1)?, *r.at_2d(0, 2)?,
        *r.at_2d(1, 0)?, *r.at_2d(1, 1)?, *r.at_2d(1, 2)?,
        *r.at_2d(2, 0)?, *r.at_2d(2, 1)?, *r.at_2d(2, 2)?,
    ));
    let pose = na::Isometry3::<f64>::from_parts(
        na::Translation3::new(*t.at_2d(0, 0)?, *t.at_2d(1, 0)?, *t.at_2d(2, 0)?),
        na::UnitQuaternion::from_rotation_matrix(&r), 
    );
    println!("step_pose: {}", pose.to_matrix());
    println!("t.norm: {}", pose.translation.vector.norm());

    Ok(pose)
}

mod test {
    use opencv::prelude::{MatTraitConst, MatTraitConstManual, Feature2DTrait, DescriptorMatcherTrait};

    #[test]
    fn test_track() -> Result<(), Box<dyn std::error::Error>> {
        let path = "/home/zhang/Downloads/MH_01_easy/mav0/cam0";
        let data_set = super::super::load_data::load_euroc_data(path)?;
        let camera = super::super::camera::CameraIntrinsics::new_euroc();
        let mut tracker = super::Tracker::new(camera).unwrap();
        let mut pose = nalgebra::Isometry3::<f64>::identity();
        for data in data_set {
            pose = match tracker.track(data) {
                Ok(pose) => pose,
                Err(e) => {
                    pose
                },
            };
            println!("pose: {}", pose.to_matrix());
        }

        Err("end".into())
    }

    fn detect(img: &opencv::core::Mat) -> Result<(opencv::core::Vector<opencv::core::KeyPoint>, opencv::core::Mat), Box<dyn std::error::Error>> {
        let mut corners = opencv::core::Mat::default();
        opencv::imgproc::good_features_to_track(
            &img, 
            &mut corners, 
            1000, 
            0.01, 
            10.0, 
            &opencv::core::Mat::default(), 
            3, 
            false, 
            0.04).unwrap(); 
        
        // compute orb
        let mut orb_keypoints = opencv::core::Vector::default();
        let mut orb_desc = opencv::core::Mat::default();
        let mut orb = opencv::features2d::ORB::create(
            1000, 
            1.2, 
            8, 
            31, 
            0, 
            2, 
            opencv::features2d::ORB_ScoreType::HARRIS_SCORE, 
            31, 
            20,).unwrap();
        
        // map corner to keypoint
        for i in 0..corners.rows() {
            let pt = corners.at_row::<opencv::core::Point2f>(i).unwrap();
            let pt = opencv::core::Point2f::new(pt[0].x, pt[0].y);
            let kp = opencv::core::KeyPoint::new_point(
                pt,
                1.0, 
                -1.0, 
                0.0, 
                0, 
                -1,).unwrap();
            orb_keypoints.push(kp);
        }
        orb.compute(&img, &mut orb_keypoints,  &mut orb_desc).unwrap();

        Ok((orb_keypoints, orb_desc))
    }

    #[test]
    fn test_extract_feature() -> Result<(), Box<dyn std::error::Error>> {
        let img_name1 = "/home/zhang/Downloads/MH_01_easy/mav0/cam0/data/1403636634913555456.png";
        let img_name2 = "/home/zhang/Downloads/MH_01_easy/mav0/cam0/data/1403636634963555584.png";
        let img1 = opencv::imgcodecs::imread(&img_name1, opencv::imgcodecs::IMREAD_GRAYSCALE).unwrap();
        let img2 = opencv::imgcodecs::imread(&img_name2, opencv::imgcodecs::IMREAD_GRAYSCALE).unwrap();

        let (kp1, desc1) = detect(&img1)?;
        let (kp2, desc2) = detect(&img2)?;
        // match
        let mut matcher = opencv::features2d::BFMatcher::create(
            opencv::core::NORM_HAMMING, 
            false).unwrap();
        matcher.add(&desc1).unwrap();
        let mut matches = opencv::core::Vector::<opencv::core::DMatch>::default();
        matcher.match_( &desc2, &mut matches, &opencv::core::Mat::default()).unwrap();

        // draw matches
        let mut out_img = opencv::core::Mat::default();
        opencv::features2d::draw_matches(
            &img2, 
            &kp2, 
            &img1, 
            &kp1, 
            &matches, 
            &mut out_img, 
            opencv::core::Scalar::all(-1.0), 
            opencv::core::Scalar::all(-1.0), 
            &opencv::core::Vector::<i8>::default(), 
            opencv::features2d::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS).unwrap();

        opencv::highgui::imshow("matches", &out_img).unwrap();
        opencv::highgui::wait_key(0).unwrap();

        Err("end".into())
    }

    #[test]
    fn test_bug_img() {
        let img_name = "/home/zhang/Downloads/MH_01_easy/mav0/cam0/data/1403636634863555584.png";
        let img = match opencv::imgcodecs::imread(img_name, opencv::imgcodecs::IMREAD_GRAYSCALE) {
            Ok(img) => img,
            Err(e) => {
                println!("Read img failed: {}.\n Error: {}", img_name, e);
                return;
            },
        };
        let out_img = img;
        let mut out_img_color = opencv::core::Mat::default();
        opencv::imgproc::cvt_color(&out_img, &mut out_img_color, opencv::imgproc::COLOR_GRAY2BGR, 0).unwrap();

        // draw
        opencv::highgui::imshow("matches", &out_img_color).unwrap();
        opencv::highgui::wait_key(0).unwrap();
    }

    #[test]
    fn test_bug_track() -> Result<(), Box<dyn std::error::Error>> {
        let path1 = "/home/zhang/Downloads/MH_01_easy/mav0/cam0/data/1403636580263555584.png";
        let path2 = "/home/zhang/Downloads/MH_01_easy/mav0/cam0/data/1403636580313555456.png";
        let data1 = super::super::load_data::EurocData {
            timestamp: std::time::Duration::default(),
            img_name: path1.into(),
        }; 
        let data2 = super::super::load_data::EurocData {
            timestamp: std::time::Duration::default(),
            img_name: path2.into(),
        }; 

        let data_set = vec![data1, data2];
        let camera = super::super::camera::CameraIntrinsics::new_euroc();
        let mut tracker = super::Tracker::new(camera).unwrap();
        for data in data_set {
            let pose = tracker.track(data).unwrap();
            println!("pose: {}", pose.to_matrix());
        }

        Err("end".into())
    }
}