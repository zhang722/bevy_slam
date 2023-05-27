use std::error::Error;
use std::fmt::format;
use std::time;

use opencv::{
    core, prelude::*,
};
use nalgebra as na;

use super::camera;
use super::cv_convert;


// t1= U(:,3) and R1=U * W * VT
// t2=−U(:,3) and R2=U * WT * VT
// if det(Ri)<0 then R=−R and t=−t
// then we have 4 possible solutions
// (R1,t1),(R1,t2),(R2,t1),(R2,t2)
pub fn from_essential(
    essential_mat: &core::Mat,
    points1: &core::Vector<core::Point2f>,
    points2: &core::Vector<core::Point2f>,
    intrinsics: &camera::CameraIntrinsics,
) -> Result<(na::Isometry3<f64>, Vec<bool>, Vec<na::Point3<f64>>), Box<dyn Error>> {
    // use nalgebra to decompose essential matrix
    let e = cv_convert::cv_mat_to_na_mat(essential_mat)?;
    let svd = e.svd(true, true);
    let mut u = svd.u.unwrap();
    // let sigma = svd.singular_values;
    let mut vt = svd.v_t.unwrap();
    let w = na::Matrix3::<f64>::new(
        0.0, -1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 1.0,
    );
    if u.determinant() < 0.0 {
        u = -u;
    }
    if vt.determinant() < 0.0 {
        vt = -vt;
    }

    let r1 = u * w * vt;
    let r2 = u * w.transpose() * vt;
    let t1 = na::Vector3::<f64>::new(
        u.m13, u.m23, u.m33
    );
    let t2 = -t1;
    
    let (inliers1, mask1, points3d1) = check_cheirality(&r1, &t1, points1, points2, intrinsics)?;
    let (inliers2, mask2, points3d2) = check_cheirality(&r1, &t2, points1, points2, intrinsics)?;
    let (inliers3, mask3, points3d3) = check_cheirality(&r2, &t1, points1, points2, intrinsics)?;
    let (inliers4, mask4, points3d4) = check_cheirality(&r2, &t2, points1, points2, intrinsics)?;

    // get max inliers
    let max_inliers = inliers1.max(inliers2).max(inliers3).max(inliers4);
    if (max_inliers as f64 / points1.len() as f64) < 0.5 {
        return Err("Not enough inliers".into());
    }
    // get corresponding r and t
    let (r, t, mask, points3d) = if max_inliers == inliers1 {
        (r1, t1, mask1, points3d1)
    } else if max_inliers == inliers2 {
        (r1, t2, mask2, points3d2)
    } else if max_inliers == inliers3 {
        (r2, t1, mask3, points3d3)
    } else {
        (r2, t2, mask4, points3d4)
    };
    // construct pose from r and t
    let pose = na::Isometry3::<f64>::from_parts(
        na::Translation3::<f64>::from(t),
        na::UnitQuaternion::<f64>::from_rotation_matrix(&na::Rotation3::<f64>::from_matrix_unchecked(r)),
    );

    Ok((pose, mask, points3d))
}

pub fn check_cheirality(
    r: &na::Matrix3<f64>,
    t: &na::Vector3<f64>,
    points1: &core::Vector<core::Point2f>,
    points2: &core::Vector<core::Point2f>,
    intrinsics: &camera::CameraIntrinsics,
) -> Result<(usize, Vec<bool>, Vec<na::Point3<f64>>), Box<dyn Error>> {
    let mut mask = Vec::new();
    let mut points3d = Vec::new();
    for (pt1, pt2) in points1.iter().zip(points2.iter()) {
        let x1 = cv_convert::cv_point2f_to_na_point2f(&pt1);
        let x2 = cv_convert::cv_point2f_to_na_point2f(&pt2);
        let p1 = intrinsics.k_mat * na::Matrix3x4::<f64>::from_rows(&[
            na::RowVector4::<f64>::new(1.0, 0.0, 0.0, 0.0),
            na::RowVector4::<f64>::new(0.0, 1.0, 0.0, 0.0),
            na::RowVector4::<f64>::new(0.0, 0.0, 1.0, 0.0),
        ]);
        let p2 = intrinsics.k_mat * na::Matrix3x4::<f64>::from_columns(&[
            r.column(0), r.column(1), r.column(2), t.into()
        ]);

        let p1 = trangulate_point_linear(&x1, &x2, &p1, &p2)?;
        let p2 = r * p1 + t;

        points3d.push(p1);
        mask.push(p1[2] > 0.0 && p2[2] > 0.0 && p1[2] < 50.0 && p2[2] < 50.0);
    }

    let inlier = mask.iter().filter(|&n| *n).count();

    Ok((inlier, mask, points3d))
}

pub fn trangulate_point_linear(
    xl: &na::Point2<f64>,
    xr: &na::Point2<f64>,
    pl: &na::Matrix3x4<f64>,
    pr: &na::Matrix3x4<f64>,
) -> Result<na::Point3<f64>, Box<dyn Error>> { 
    let design = na::Matrix4::<f64>::from_rows(&[
        xl.x * pl.row(2) - pl.row(0),
        xl.y * pl.row(2) - pl.row(1),
        xr.x * pr.row(2) - pr.row(0),
        xr.y * pr.row(2) - pr.row(1),
    ]);
    let svd = design.svd(true, true);
    let vt = svd.v_t.expect("SVD failed");
    let p = vt.row(vt.nrows() - 1).transpose();
    let p = p / p[3];
    Ok(na::Point3::<f64>::new(p[0], p[1], p[2]))
}
