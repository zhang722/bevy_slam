use bevy::prelude::*;

mod visualizer;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    App::new()
        .insert_resource(ClearColor(Color::rgb(1.0, 1.0, 1.0)))
        .add_plugins(DefaultPlugins)
        // pan orbit camera
        .add_plugin(visualizer::pan_orbit_camera::PanOrbitCameraPlugin)
        // server
        .add_plugin(visualizer::server::ServerPlugin)
        // message
        .add_plugin(visualizer::message::MessagePlugin)
        // process msg
        .add_plugin(visualizer::process_msg::ProcessMsgPlugin)
        .run();

    Ok(())
}


