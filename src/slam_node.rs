use std::io::Write;

use nalgebra as na;

use opencv::prelude::KeyPointTraitConst;
use rosrust_msg::*;
use rosrust_msg::sensor_msgs::PointField;

mod slam;

fn main() {
    // Initialize node
    rosrust::init("talker");

    // Create publisher
    let path_pub: rosrust::Publisher<nav_msgs::Path> = rosrust::publish("chatter", 100).unwrap();
    let pose_pub: rosrust::Publisher<geometry_msgs::PoseStamped> = rosrust::publish("pose", 100).unwrap();
    let point_cloud_pub: rosrust::Publisher<sensor_msgs::PointCloud> = rosrust::publish("slam_point_cloud", 10).unwrap();

    let mut count = 0;

    // Create object that maintains 20Hz between sleep requests
    let rate = rosrust::rate(20.0);
    let path = "/media/zhang/data/ubuntu_files/downloads/MH_01_easy/mav0/cam0";
    let data_set = slam::load_data::load_euroc_data(path).unwrap();
    let camera = slam::camera::CameraIntrinsics::new_euroc();
    let mut tracker = slam::process_image::Tracker::new(camera).unwrap();
    let mut data_iter = data_set.into_iter();
    let mut pose = na::Isometry3::<f64>::identity();
    let mut path_msg = nav_msgs::Path::default();
    let mut point_cloud_msg = sensor_msgs::PointCloud::default();

    // Breaks when a shutdown signal is sent
    while rosrust::is_ok() {
        if tracker.initializer.done() {
            scene_save(&tracker.map, "scene.json");
            let keyframes = 
                tracker.map.keyframes.values().into_iter().map(|x| x.id).collect::<Vec<_>>();
            let mappoints = 
                tracker.map.mappoints.values().into_iter().map(|x| x.borrow().id).collect::<Vec<_>>();
            
            slam::optimize::optimize(
                &mut tracker.map, &keyframes, &mappoints);
            scene_save(&tracker.map, "scene_opt.json");
            break;
        }
        // Create string message
        pose = match data_iter.next() {
            Some(data) => {
                tracker.track(data).unwrap_or(pose)
            },
            None => {
                break;
            },
        };

        let pose = geometry_msgs::Pose{
            position: geometry_msgs::Point {
                x: pose.translation.x / 10.0,
                y: pose.translation.y / 10.0,
                z: pose.translation.z / 10.0,
            },
            orientation: geometry_msgs::Quaternion {
                x: pose.rotation.i,
                y: pose.rotation.j,
                z: pose.rotation.k,
                w: pose.rotation.w,
            },
        };

        let header = std_msgs::Header {
            seq: count,
            stamp: rosrust::now(),
            frame_id: "odom".to_string(),
        };

        let msg = geometry_msgs::PoseStamped {
            header : header.clone(),
            pose: pose.clone(),
        };
        pose_pub.send(msg.clone()).unwrap();

        path_msg.header = header.clone();
        path_msg.poses.push(msg);

        // Send string message to topic via publisher
        path_pub.send(path_msg.clone()).unwrap();

        // pointcloud
        point_cloud_msg.header = header.clone();
        if tracker.initializer.done() {
            let points = tracker.map.points();
            for point in points {
                let point = geometry_msgs::Point32 {
                    x: point.as_ref().borrow().position.x as f32,
                    y: point.as_ref().borrow().position.y as f32,
                    z: point.as_ref().borrow().position.z as f32,
                };
                point_cloud_msg.points.push(point);
            }
        }
        point_cloud_pub.send(point_cloud_msg.clone()).unwrap();

        // Sleep to maintain 10Hz rate
        rate.sleep();

        count += 1;
    }
}

fn scene_save(map: &slam::map::Map, filename: &str) {
    let mut file = std::fs::File::create(filename).unwrap();
    let points = map.points();
    let keyframes = &map.keyframes;
    // for point in points.iter() {
    //     let point = point.as_ref().borrow();
    //     for reference in point.references.iter() {
    //             if reference.id != 0 {
    //                 continue;
    //             }
    //             let mut ps = na::Vector3::<f64>::new(point.position.x, point.position.y, point.position.z);
    //             ps /= ps[2];
    //             let pp = slam::camera::CameraIntrinsics::new_euroc().k_mat * ps;
    //             println!("res: ({},{})", pp[0] as f32 - reference.keypoint.pt().x, pp[1] as f32 - reference.keypoint.pt().y);
    //     }
     
    // }

    // output to json
    let mut json = String::new();
    json.push_str("{\n");
    json.push_str("\t\"points\": [\n");
    for point in points.iter() {
        let point = point.as_ref().borrow();
        json.push_str(&format!("\t\t[{},{},{}],\n", point.position.x, point.position.y, point.position.z));
    }
    json.push_str("\t],\n");
    json.push_str("\t\"keyframes\": [\n");
    for kf in keyframes {
        let kf = kf.1;
        json.push_str("\t\t{\n");
        json.push_str(&format!("\t\t\t\"pose\":{{\"x\": {}, \"y\": {}, \"z\": {}, \"q\": [{}, {}, {}, {}]}},\n", kf.pose.translation.x, kf.pose.translation.y, kf.pose.translation.z, kf.pose.rotation.i, kf.pose.rotation.j, kf.pose.rotation.k, kf.pose.rotation.w));

        json.push_str("\t\t\t\"points\": [");
        for point in points.iter() {
            let point = point.as_ref().borrow();
            for reference in point.references.iter() {
                if reference.id != kf.id {
                    continue;
                }
                json.push_str(&format!("[{},{}],", reference.keypoint.pt().x, reference.keypoint.pt().y));
            }
        }
        json.push_str("]\n");
        json.push_str("\t\t},\n");
    }
    json.push_str("\t]\n");
    json.push_str("}\n");
    file.write_all(json.as_bytes()).unwrap();
}