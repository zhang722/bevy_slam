use std::{collections::HashMap, sync::Arc};

use bevy::{prelude::*};
use bevy_obj::*;
use bevy_prototype_debug_lines::*;

use super::message;

#[derive(Component)]
struct Body;
#[derive(Component)]
struct Point;

#[derive(Resource)]
struct IdMap(HashMap<usize, Entity>);

pub struct ProcessMsgPlugin;
impl Plugin for ProcessMsgPlugin {
    fn build(&self, app: &mut App) {
         app
            .insert_resource(IdMap(HashMap::new()))
            .add_plugin(ObjPlugin)
            .add_plugin(DebugLinesPlugin::default())
            .add_startup_system(startup)
            .add_system(show_frame)
            .add_system(receive_body_event)
            .add_system(receive_point_event);
    }
}

fn startup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mesh_handle: Handle<Mesh> = asset_server.load("obj/iphonex.obj");
    commands.spawn(PbrBundle {
        mesh: mesh_handle,
        material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        transform: Transform {scale: Vec3::splat(0.001), ..Default::default()},
        ..Default::default()
    }).insert(Body);

    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cylinder {radius: 0.01, height: 1.0, resolution: 10, segments: 1})),
        material: materials.add(Color::rgb(1.0, 0.0, 0.0).into()),
        transform: Transform {
            translation: Vec3::new(0.5, 0.0, 0.0), 
            rotation: Quat::from_axis_angle(Vec3::Z, std::f32::consts::FRAC_PI_2),
            ..Default::default()
        },
        ..Default::default()
    });
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cylinder {radius: 0.01, height: 1.0, resolution: 10, segments: 1})),
        material: materials.add(Color::rgb(0.0, 1.0, 0.0).into()),
        transform: Transform::from_xyz(0.0, 0.5, 0.0),
        ..Default::default()
    });
    commands.spawn(PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cylinder {radius: 0.01, height: 1.0, resolution: 10, segments: 1})),
        material: materials.add(Color::rgb(0.0, 0.0, 1.0).into()),
        transform: Transform {
            translation: Vec3::new(0.0, 0.0, 0.5), 
            rotation: Quat::from_axis_angle(Vec3::X, std::f32::consts::FRAC_PI_2),
            ..Default::default()
        },
        ..Default::default()
    });
    commands.spawn(PointLightBundle {
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        point_light: PointLight {
            intensity: 1000.0,
            range: 100.0,
            ..Default::default()
        },
        ..Default::default()
    });
}

fn show_frame(
    mut lines: ResMut<DebugLines>,
) {
    lines.line_colored(
        Vec3::ZERO,
        Vec3::X,
        0.0,
        Color::RED,
    );

    // 创建 Y 轴的线段
    lines.line_colored(
        Vec3::ZERO,
        Vec3::Y,
        0.0,
        Color::GREEN,
    );

    // 创建 Z 轴的线段
    lines.line_colored(
        Vec3::ZERO,
        Vec3::Z,
        0.0,
        Color::BLUE,
    );
}



fn receive_body_event(
    time: Res<Time>,
    mut body_transform_event: EventReader<message::BodyTransformEvent>,
    mut query: Query<&mut Transform, With<Body>>,
) {
    for event in body_transform_event.iter() {
        println!("body event: {:?}", event.0);
        let mut t_wc = query.single_mut();
        let t_oc_nc = event.0.transform;
        let temp = t_wc.rotation * t_oc_nc.translation;
        t_wc.translation += temp;
        t_wc.rotate(t_oc_nc.rotation);
    }
}

fn receive_point_event(
    mut commands: Commands,
    mut point_event: EventReader<message::PointEvent>,
    mut id_map: ResMut<IdMap>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut query: Query<&mut Transform, With<Point>>,
) {
    for event in point_event.iter() {
        let id = event.0.id;
        if id_map.0.contains_key(&id) {
            let entity = id_map.0.get(&id).unwrap();
            if let Ok(mut t) = query.get_mut(*entity) {
                t.translation = event.0.position;
            } else {
                println!("get mut transform failed");
            }
        } else {
            let id: Entity = commands.spawn(PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Cube { size: 0.01 })),
                material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
                transform: Transform::from_translation(event.0.position),
                ..Default::default()
            }).insert(Point).id();
            id_map.0.insert(event.0.id, id);
        }
        println!("point event: {:?}", event.0);
    }
}