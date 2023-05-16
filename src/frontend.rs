use bevy::prelude::*;

mod visualizer;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    App::new()
        .add_plugins(DefaultPlugins)
        // pan orbit camera
        .add_plugin(visualizer::pan_orbit_camera::PanOrbitCameraPlugin)

        // server
        .add_plugin(visualizer::server::ServerPlugin)
        .run();

    Ok(())
}


