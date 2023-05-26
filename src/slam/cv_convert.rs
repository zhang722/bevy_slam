use std::error::Error;
use opencv::{prelude::*, core::Point2f};
use nalgebra as na;

pub fn cv_mat_to_na_mat(mat: &Mat) -> Result<na::Matrix3<f64>, Box<dyn Error>> {
    Ok(na::Matrix3::<f64>::from_row_slice(&[
        *mat.at_2d::<f64>(0, 0)?, *mat.at_2d::<f64>(0, 1)?, *mat.at_2d::<f64>(0, 2)?,
        *mat.at_2d::<f64>(1, 0)?, *mat.at_2d::<f64>(1, 1)?, *mat.at_2d::<f64>(1, 2)?,
        *mat.at_2d::<f64>(2, 0)?, *mat.at_2d::<f64>(2, 1)?, *mat.at_2d::<f64>(2, 2)?,
    ]))
}

pub fn cv_point2f_to_na_point2f(pt: &Point2f) -> na::Point2<f64> {
    na::Point2::<f64>::new(pt.x as f64, pt.y as f64)
}