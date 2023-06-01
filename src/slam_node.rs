use nalgebra as na;

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
    let path = "/home/zhang/Downloads/MH_01_easy/mav0/cam0";
    let data_set = slam::load_data::load_euroc_data(path).unwrap();
    let camera = slam::camera::CameraIntrinsics::new_euroc();
    let mut tracker = slam::process_image::Tracker::new(camera).unwrap();
    let mut data_iter = data_set.into_iter();
    let mut pose = na::Isometry3::<f64>::identity();
    let mut path_msg = nav_msgs::Path::default();
    let mut point_cloud_msg = sensor_msgs::PointCloud::default();

    // 设置点云数据
    let point1 = geometry_msgs::Point32 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    let point2 = geometry_msgs::Point32 {
        x: 4.0,
        y: 5.0,
        z: 6.0,
    };
    let point3 = geometry_msgs::Point32 {
        x: 7.0,
        y: 8.0,
        z: 9.0,
    };

    // Breaks when a shutdown signal is sent
    while rosrust::is_ok() {
        if tracker.initializer.done() {
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
                    x: point.borrow().position.x as f32,
                    y: point.borrow().position.y as f32,
                    z: point.borrow().position.z as f32,
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