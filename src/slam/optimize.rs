use std::{ 
    rc::Rc, 
    cell::RefCell,
};

use nalgebra as na;

use lm::*;
use opencv::prelude::KeyPointTraitConst;

use super::map::{ Map, mappoint::*, keyframe::* };

/// Converts a 6-Vector Lie Algebra representation of a rigid body
/// transform to an NAlgebra Isometry (quaternion+translation pair)
///
/// This is largely taken from this paper:
/// https://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
fn exp_map(param_vector: &na::Vector6<f64>) -> na::Isometry3<f64> {
    let t = param_vector.fixed_view::<3, 1>(0, 0);
    let omega = param_vector.fixed_view::<3, 1>(3, 0);
    let theta = omega.norm();
    let half_theta = 0.5 * theta;
    let quat_axis = omega * half_theta.sin() / theta;
    let quat = if theta > 1e-6 {
        na::UnitQuaternion::from_quaternion(na::Quaternion::new(
            half_theta.cos(),
            quat_axis.x,
            quat_axis.y,
            quat_axis.z,
        ))
    } else {
        na::UnitQuaternion::identity()
    };

    let mut v = na::Matrix3::<f64>::identity();
    if theta > 1e-6 {
        let ssym_omega = skew_sym(omega.clone_owned());
        v += ssym_omega * (1.0 - theta.cos()) / (theta.powi(2))
            + (ssym_omega * ssym_omega) * ((theta - theta.sin()) / (theta.powi(3)));
    }

    let trans = na::Translation::from(v * t);

    na::Isometry3::from_parts(trans, quat)
}


/// Produces a skew-symmetric or "cross-product matrix" from
/// a 3-vector. This is needed for the `exp_map` and `log_map`
/// functions
fn skew_sym(v: na::Vector3<f64>) -> na::Matrix3<f64> {
    let mut ss = na::Matrix3::zeros();
    ss[(0, 1)] = -v[2];
    ss[(0, 2)] = v[1];
    ss[(1, 0)] = v[2];
    ss[(1, 2)] = -v[0];
    ss[(2, 0)] = -v[1];
    ss[(2, 1)] = v[0];
    ss
}

/// Converts an NAlgebra Isometry to a 6-Vector Lie Algebra representation
/// of a rigid body transform.
///
/// This is largely taken from this paper:
/// https://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
fn log_map(input: &na::Isometry3<f64>) -> na::DVector<f64> {
    let t: na::Vector3<f64> = input.translation.vector;

    let quat = input.rotation;
    let theta: f64 = 2.0 * (quat.scalar()).acos();
    let half_theta = 0.5 * theta;
    let mut omega = na::Vector3::<f64>::zeros();

    let mut v_inv = na::Matrix3::<f64>::identity();
    if theta > 1e-6 {
        omega = quat.vector() * theta / (half_theta.sin());
        let ssym_omega = skew_sym(omega);
        v_inv -= ssym_omega * 0.5;
        v_inv += ssym_omega * ssym_omega * (1.0 - half_theta * half_theta.cos() / half_theta.sin())
            / (theta * theta);
    }

    let mut ret = na::DVector::<f64>::zeros(6);
    ret.fixed_view_mut::<3, 1>(0, 0).copy_from(&(v_inv * t));
    ret.fixed_view_mut::<3, 1>(3, 0).copy_from(&omega);

    ret
}

pub fn optimize(map: &mut Map, keyframes: &[KeyFrameId], mappoints: &[MapPointId])
{
    let mut graph = Graph::default();
    let mut id = 0;
    let camera_vertices = keyframes.iter().enumerate().map(|(idx, id_kf)| {
        let pose = map.keyframe(*id_kf).expect("keyframe id not correct!").pose;
        
        let ret = Rc::new(RefCell::new(CameraVertex {
            id,
            params: log_map(&pose),
            edges: Vec::new(),
            fixed: idx == 0,
            hessian_index: 0,
        })) as VertexBase;
        id += 1;
        ret
    }).collect::<Vec<_>>();

    let mut id_edge = 0;
    let point_vertices = mappoints.iter().map(|id_mp| {
        let mp = map.mappoint(*id_mp).expect("mappoint id not correct!");
        let mp = mp.borrow();
        let ret = Rc::new(RefCell::new(PointVertex {
            id,
            params: na::dvector![mp.position.x, mp.position.y, mp.position.z],
            edges: Vec::new(),
            fixed: false,
            hessian_index: 0,
        })) as VertexBase;
        id += 1;

        // add edges into graph
        for mp_reference in mp.references.iter() {
            if let Some(idx_kf) = keyframes.iter().position(|x| *x == mp_reference.id) {
                let obs = mp_reference.keypoint.pt();
                if let Some(keyframe) = map.keyframe(mp_reference.id) {
                    let edge = Rc::new(RefCell::new( Point3dProjectWithIntrinsicEdge {
                        id: id_edge,
                        vertices: Vec::new(),
                        sigma: na::DMatrix::<f64>::identity(2, 2),
                        measurement: na::dvector![obs.x as f64, obs.y as f64],
                        intrinsic: keyframe.intrinsics.vector(),
                    }
                    )) as EdgeBase;
                    id_edge += 1;
                    edge.borrow_mut().add_vertex(camera_vertices[idx_kf].clone());
                    edge.borrow_mut().add_vertex(ret.clone());
                    graph.add_edge(&edge);
                } 
            }
        }
        ret
    }).collect::<Vec<_>>();

    graph.add_vertex_set(camera_vertices);
    graph.add_vertex_set(point_vertices);
    // for vertex in graph.vertex_set(0) {
    //     println!("is fixed: {}", vertex.borrow().is_fixed());
    // }
    graph.optimize();

    // update keyframes
    for (idx, id_kf) in keyframes.iter().enumerate() {
        let kf = map.keyframe_mut(*id_kf).expect("keyframe id not correct!");
        let camera_vertex = graph.vertex(idx).expect("camera vertex id not correct!");
        let camera_vertex = camera_vertex.borrow();
        let pose = exp_map(&camera_vertex.params().fixed_view::<6, 1>(0, 0).clone_owned());
        kf.pose = pose;
    }

    // update mappoints
    for (idx, id_mp) in mappoints.iter().enumerate() {
        let mp = map.mappoint(*id_mp).expect("mappoint id not correct!");
        let mut mp = mp.borrow_mut();
        let point_vertex = graph.vertex(idx + keyframes.len()).expect("point vertex id not correct!");
        let point_vertex = point_vertex.borrow();
        mp.position = na::Vector3::<f64>::new(point_vertex.params()[0], point_vertex.params()[1], point_vertex.params()[2]);
    }
}