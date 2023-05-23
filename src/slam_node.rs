use nalgebra as na;

use rosrust_msg::*;
mod slam;

fn main() {
    // Initialize node
    rosrust::init("talker");

    // Create publisher
    let path_pub: rosrust::Publisher<nav_msgs::Path> = rosrust::publish("chatter", 100).unwrap();
    let pose_pub: rosrust::Publisher<geometry_msgs::PoseStamped> = rosrust::publish("pose", 100).unwrap();

    let mut count = 0;

    // Create object that maintains 20Hz between sleep requests
    let rate = rosrust::rate(20.0);
    let path = "/home/zhang/Downloads/MH_01_easy/mav0/cam0";
    let data_set = slam::load_data::load_euroc_data(path).unwrap();
    let camera = slam::camera::CameraIntrinsics::new_euroc();
    let mut tracker = slam::process_image::Tracker::new(camera).unwrap();
    let mut data_iter = data_set.into_iter();
    let mut pose = na::Isometry3::<f64>::identity();
    let mut path_msg = nav_msgs::Path::default();

    // Breaks when a shutdown signal is sent
    while rosrust::is_ok() {
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

        // Sleep to maintain 10Hz rate
        rate.sleep();

        count += 1;
    }
}