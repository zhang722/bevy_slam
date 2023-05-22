pub struct CameraIntrinsics {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,

    pub k1: f64,
    pub k2: f64,
    pub p1: f64,
    pub p2: f64,
}

impl CameraIntrinsics {
    pub fn new_euroc() -> Self {
        Self { fx: 458.654, fy: 457.296, cx: 367.215, cy: 248.375, k1: -0.28340811, k2: 0.07395907, p1: 0.00019359, p2: 1.76187114e-05 }
    }
}