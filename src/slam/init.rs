use std::{
    rc::Rc,
    cell::RefCell,
    error::Error
};

use opencv::{
    core,
    calib3d,
    imgproc, prelude::MatTraitConst,
};

use super::{
    frame::{self, Frame},
    map::{ Map, mappoint::*, keyframe::* },
    camera,
    recover_pose,
};

pub struct Init {
    first_frame: Option<Frame>,
    second_frame: Option<Frame>,
    matches: Vec<core::DMatch>,
    intrinsics: camera::CameraIntrinsics, 
    pub map: Map,
    done: bool,
}

impl Init {
    pub fn new(
        intrinsics: &camera::CameraIntrinsics,
    ) -> Self {
        Self {
            first_frame: None,
            second_frame: None,
            matches: Vec::new(),
            intrinsics: intrinsics.clone(),
            map: Map::new(),
            done: false,
        }
    }

    pub fn done(&self) -> bool {
        self.done
    }

    pub fn run(&mut self, inframe: Frame) -> bool {
        if self.first_frame.is_none() {
            self.first_frame = Some(inframe);
            return false;
        }
        let first_frame = self.first_frame.clone().unwrap();
        let matches = match first_frame.match_other(&inframe) {
            Ok(matches) => matches,
            Err(e) => {
                println!("match failed: {}", e);
                return false;
            }
        };

        // if first_frame.parallax_other(&inframe, &matches).expect("compute parallax falied") < 1.0 {
        //     return false;
        // }

        self.second_frame = Some(inframe);
        self.matches = matches;

        self.initialize().is_ok()
    }

    pub fn initialize(&mut self) -> Result<(), Box<dyn Error>> {
        let first_frame = self.first_frame.clone().unwrap();
        let mut second_frame = self.second_frame.as_mut().unwrap();
        let (inliers1, inliers2) = frame::matches2points(
            &self.matches, 
            &first_frame.keypoints, 
            &second_frame.keypoints
        )?;
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
            self.intrinsics.fx, 
            core::Point_ { x: self.intrinsics.cx, y: self.intrinsics.cy }, 
            calib3d::RANSAC, 
            0.999, 
            1.0, 
            &mut mask)?;

        let matches = self.matches.iter().zip(mask.iter()).filter(|(_, status)| *status != 0).map(|(m, _)| *m).collect::<Vec<_>>();
        let (inliers1, inliers2) = frame::matches2points(
            &matches, 
            &first_frame.keypoints, 
            &second_frame.keypoints
        )?;

        let out_img = second_frame.img.clone();
        let mut out_img_color = core::Mat::default();
        imgproc::cvt_color(&out_img, &mut out_img_color, imgproc::COLOR_GRAY2BGR, 0)?;

        for (last_pt, curr_pt) in inliers1.iter().zip(inliers2.iter()) {
            let last_pt = core::Point2i::new(last_pt.x as i32, last_pt.y as i32);
            let curr_pt = core::Point2i::new(curr_pt.x as i32, curr_pt.y as i32);
            imgproc::line(
                &mut out_img_color, 
                last_pt,
                curr_pt,
                core::Scalar::new(255.0, 0.0, 255.0, 255.0),
                2, 
                8, 
                0)?;
        }

        opencv::highgui::imshow("matches", &out_img_color)?;
        opencv::highgui::wait_key(0)?;

        // NOTE: type of mask is Vec<bool>, not opencv::core::Vector<u8>
        // and type of points3d is Vec<na::Point3<f64>>, not opencv::core::Vector<core::Point3f>
        // since we implement the function by ourselves
        let (pose, mask, points3d) = recover_pose::from_essential(
            &essential_mat, 
            &inliers1, 
            &inliers2, 
            &self.intrinsics)?;
        second_frame.pose = pose;

        let matches = matches.into_iter().zip(mask.into_iter()).filter(|(_, status)| *status).map(|(m, _)| m.clone()).collect::<Vec<_>>();
        let (idx1, idx2) = frame::matches2indices(&matches);

        let mut kf1 = KeyFrame::from_frame(&first_frame, &self.intrinsics);
        let mut kf2 = KeyFrame::from_frame(&second_frame, &self.intrinsics);
        let mut map = Map::new();

        println!("p3d len: {}", points3d.len());
        println!("idx1 len: {}", idx1.len());
        println!("idx2 len: {}", idx2.len());

        for (idx, point) in points3d.iter().enumerate() {
            let kp1 = first_frame.keypoints.get(idx1[idx]).unwrap();
            let kp2 = second_frame.keypoints.get(idx2[idx]).unwrap();
            let des1 = first_frame.descriptors.row(idx1[idx] as i32).unwrap();
            let des2 = second_frame.descriptors.row(idx2[idx] as i32).unwrap();
            let mp = Rc::new(RefCell::new(MapPoint::from_point(*point, &des2)));
            kf1.add_observation(mp.clone());
            kf2.add_observation(mp.clone());
            mp.borrow_mut().add_reference(MapPointReference::new_with_kf(&kf1, &kp1, &des1));
            mp.borrow_mut().add_reference(MapPointReference::new_with_kf(&kf2, &kp2, &des2));
            map.insert_mappoint(mp.clone());
        }

        map.insert_keyframe(kf1);
        map.insert_keyframe(kf2);
        self.map = map;
        self.done = true;

        Ok(())
    }
}
