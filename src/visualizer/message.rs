use std::{error::Error};

use bevy::{prelude::*};

use super::server;

#[derive(Debug)]
pub struct BodyTransformMessage {
    pub id: usize,
    pub transform: Transform,
}

#[derive(Debug)]
pub struct PointMessage {
    pub id: usize,
    pub position: Vec3,
}

pub struct BodyTransformEvent(pub BodyTransformMessage);
pub struct PointEvent(pub PointMessage);

pub struct MessagePlugin;
impl Plugin for MessagePlugin {
    fn build(&self, app: &mut App) {
        app
            .add_event::<BodyTransformEvent>()
            .add_event::<PointEvent>()
            .add_system(response_stream_event);
    }
}

fn response_stream_event(
    mut events: EventReader<server::StreamEvent>,
    mut body_transform_event: EventWriter<BodyTransformEvent>,
    mut point_event: EventWriter<PointEvent>
) {
    for event in events.iter() {
        let msg = &event.0;
        for str in msg.split('{') {
            if !str.ends_with('}') {
                continue;
            }
            let str = str.trim_end_matches('}');

            if str.starts_with("body") {
                let body = parse_transform_message(str).unwrap();
                body_transform_event.send(BodyTransformEvent(body));
            } else if str.starts_with("point") {
                let point = parse_point_message(str).unwrap();
                point_event.send(PointEvent(point));
            }
        }
    }
}



pub fn parse_transform_message(msg: &str) -> Result<BodyTransformMessage, Box<dyn Error>> {
    let mut parts = msg.split_whitespace();
    parts.next();
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
    parts.next();
    let id = parts.next().unwrap().parse::<usize>()?;
    let x = parts.next().unwrap().parse::<f32>()?;
    let y = parts.next().unwrap().parse::<f32>()?;
    let z = parts.next().unwrap().parse::<f32>()?;
    let position = Vec3::new(x, y, z);
    Ok(PointMessage { id, position })
}
