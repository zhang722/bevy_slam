use std::error::Error;
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
use super::map;
use super::frame::Frame;
use super::init;

const GX: usize = 15;
const GY: usize = 10;
const FEATURE_NUM: usize = GX * GY * 6;


pub struct Tracker {
    pub initializer: init::Init,
    pub pose: na::Isometry3<f64>,
    pub last_frame: Frame,
    pub curr_frame: Frame,
    pub camera: camera::CameraIntrinsics,
    pub orb_detector: core::Ptr<features2d::ORB>,
    pub bf_matcher: core::Ptr<features2d::BFMatcher>,
    pub map: map::Map,
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
            initializer: init::Init::new(&camera),
            pose: na::Isometry3::identity(),
            last_frame: Frame::default(),
            curr_frame: Frame::default(),
            camera,
            orb_detector,
            bf_matcher,
            map: map::Map::new(),
        })
    }

    pub fn track(
        &mut self, 
        data: load_data::EurocData
    ) -> Result<na::Isometry3<f64>, Box<dyn Error>> {
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

        let inframe = Frame::new(
            data.timestamp,
            img,
            orb_keypoints,
            orb_desc,
            na::Isometry3::identity(),
        );

        if !self.initializer.done() {
            println!("initlializing...");
            if self.initializer.run(inframe) {
                self.map = self.initializer.map.clone();
                println!("map size: {}", self.map.mappoints.len());
            }
            return Ok(na::Isometry3::identity());
        }

        Ok(self.pose)
    }
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
            if tracker.initializer.done() {
                break;
            }
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