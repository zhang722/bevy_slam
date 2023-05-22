use rosrust_msg::*;
mod slam;

fn main() {
    // Initialize node
    rosrust::init("talker");

    // Create publisher
    let chatter_pub: rosrust::Publisher<geometry_msgs::PoseStamped> = rosrust::publish("chatter", 100).unwrap();

    let mut count = 0;

    // Create object that maintains 20Hz between sleep requests
    let rate = rosrust::rate(20.0);
    let path = "/home/zhang/Downloads/MH_01_easy/mav0/cam0";
    let data_set = slam::load_data::load_euroc_data(path).unwrap();
    let camera = slam::camera::CameraIntrinsics::new_euroc();
    let mut tracker = slam::process_image::Tracker::new(camera).unwrap();
    let mut data_iter = data_set.into_iter();

    // Breaks when a shutdown signal is sent
    while rosrust::is_ok() {
        // Create string message
        let pose = match data_iter.next() {
            Some(data) => {
                tracker.track(data).unwrap()
            },
            None => {
                break;
            },
        };

        let msg = geometry_msgs::PoseStamped {
            header : std_msgs::Header {
                seq: count,
                stamp: rosrust::now(),
                frame_id: "odom".to_string(),
            },
            pose : geometry_msgs::Pose{
                position: geometry_msgs::Point {
                    x: pose.translation.x,
                    y: pose.translation.y,
                    z: pose.translation.z,
                },
                orientation: geometry_msgs::Quaternion {
                    x: pose.rotation.i,
                    y: pose.rotation.j,
                    z: pose.rotation.k,
                    w: pose.rotation.w,
                },
            }
        };

        // Send string message to topic via publisher
        chatter_pub.send(msg).unwrap();

        // Sleep to maintain 10Hz rate
        rate.sleep();

        count += 1;
    }
}