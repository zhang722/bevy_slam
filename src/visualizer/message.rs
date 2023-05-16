use std::error::Error;

use bevy::prelude::*;

pub struct BodyTransformMessage {
    pub id: usize,
    pub transform: Transform,
}

pub struct PointMessage {
    pub id: usize,
    pub position: Vec3,
}

pub fn parse_transform_message(msg: &str) -> Result<BodyTransformMessage, Box<dyn Error>> {
    let mut parts = msg.split_whitespace();
    let id = parts.next().unwrap().parse::<usize>()?;
    let qx = parts.next().unwrap().parse::<f32>()?;
    let qy = parts.next().unwrap().parse::<f32>()?;
    let qz = parts.next().unwrap().parse::<f32>()?;
    let qw = parts.next().unwrap().parse::<f32>()?;

    let x = parts.next().unwrap().parse::<f32>()?;
    let y = parts.next().unwrap().parse::<f32>()?;
    let z = parts.next().unwrap().parse::<f32>()?;
    let transform = Transform {
        translation: Vec3::new(x, y, z),
        rotation: Quat::from_xyzw(qx, qy, qz, qw),
        scale: Vec3::ONE,
    };
    Ok(BodyTransformMessage { id, transform })
}

pub fn parse_point_message(msg: &str) -> Result<PointMessage, Box<dyn Error>> {
    let mut parts = msg.split_whitespace();
    let id = parts.next().unwrap().parse::<usize>()?;
    let x = parts.next().unwrap().parse::<f32>()?;
    let y = parts.next().unwrap().parse::<f32>()?;
    let z = parts.next().unwrap().parse::<f32>()?;
    let position = Vec3::new(x, y, z);
    Ok(PointMessage { id, position })
}
