use nalgebra as na;

/*fx, fy, cx, cy, k1, k2, p1, p2*/
type Vector8<T> = na::Matrix<T, na::U8, na::U1, na::ArrayStorage<T, 8, 1>>;

#[derive(Clone, Copy)]
pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,

    pub k1: f64,
    pub k2: f64,
    pub p1: f64,
    pub p2: f64,
    pub k_mat : na::Matrix3<f64>,
}

impl CameraIntrinsics {
    pub fn new_euroc() -> Self {
        Self { fx: 458.654, fy: 457.296, cx: 367.215, cy: 248.375, k1: -0.28340811, k2: 0.07395907, p1: 0.00019359, p2: 1.76187114e-05 ,
            k_mat: na::Matrix3::<f64>::new(
                458.654, 0.0, 367.215,
                0.0, 457.296, 248.375,
                0.0, 0.0, 1.0,
            )
        }
    }

    pub fn inv_projection(&self, pt: &na::Point2<f64>) -> na::Point2<f64> {
        let x = (pt.x - self.cx) / self.fx;
        let y = (pt.y - self.cy) / self.fy;
        na::Point2::<f64>::new(x, y)
    }

    pub fn vector(&self) -> Vector8<f64> {
        Vector8::<f64>::from_vec(vec![self.fx, self.fy, self.cx, self.cy, self.k1, self.k2, self.p1, self.p2])
    }
}