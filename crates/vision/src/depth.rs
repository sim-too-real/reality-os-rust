//! Depth, occupancy, and a tiny visual-odometry step. Not ORB-SLAM.

use crate::Camera;

#[derive(Debug, Clone)]
pub struct DepthFrame {
    pub width: u32,
    pub height: u32,
    pub depth_m: Vec<f64>,
}

impl DepthFrame {
    pub fn new(width: u32, height: u32, depth_m: Vec<f64>) -> Result<Self, &'static str> {
        let n = (width as usize) * (height as usize);
        if depth_m.len() != n {
            return Err("depth_len_mismatch");
        }
        Ok(Self {
            width,
            height,
            depth_m,
        })
    }

    pub fn planar(width: u32, height: u32, z_m: f64) -> Self {
        Self {
            width,
            height,
            depth_m: vec![z_m; (width as usize) * (height as usize)],
        }
    }

    pub fn at(&self, x: u32, y: u32) -> Option<f64> {
        if x >= self.width || y >= self.height {
            return None;
        }
        let z = self.depth_m[(y * self.width + x) as usize];
        z.is_finite().then_some(z).filter(|z| *z > 0.0)
    }
}

pub fn unproject_depth(camera: &Camera, u: f64, v: f64, z_m: f64) -> Option<[f64; 3]> {
    if !camera.fx.is_finite()
        || !camera.fy.is_finite()
        || camera.fx <= 0.0
        || camera.fy <= 0.0
        || !z_m.is_finite()
        || z_m <= 0.0
    {
        return None;
    }
    let x = (u - camera.cx) / camera.fx * z_m;
    let y = (v - camera.cy) / camera.fy * z_m;
    Some([x, y, z_m])
}

#[derive(Debug, Clone)]
pub struct OccupancyGrid {
    pub origin_xy: [f64; 2],
    pub resolution_m: f64,
    pub width: usize,
    pub height: usize,
    pub occupied: Vec<bool>,
}

impl OccupancyGrid {
    pub fn empty(origin_xy: [f64; 2], resolution_m: f64, width: usize, height: usize) -> Self {
        Self {
            origin_xy,
            resolution_m: resolution_m.max(1e-3),
            width,
            height,
            occupied: vec![false; width * height],
        }
    }

    pub fn occupied_count(&self) -> usize {
        self.occupied.iter().filter(|c| **c).count()
    }
}

pub fn integrate_depth(
    camera: &Camera,
    depth: &DepthFrame,
    stride: u32,
) -> OccupancyGrid {
    let mut grid = OccupancyGrid::empty([-1.0, -1.0], 0.05, 40, 40);
    let step = stride.max(1);
    for y in (0..depth.height).step_by(step as usize) {
        for x in (0..depth.width).step_by(step as usize) {
            let Some(z) = depth.at(x, y) else {
                continue;
            };
            let Some(p) = unproject_depth(camera, f64::from(x), f64::from(y), z) else {
                continue;
            };
            let ix = ((p[0] - grid.origin_xy[0]) / grid.resolution_m).floor() as i32;
            let iy = ((p[1] - grid.origin_xy[1]) / grid.resolution_m).floor() as i32;
            if ix >= 0 && iy >= 0 && (ix as usize) < grid.width && (iy as usize) < grid.height {
                grid.occupied[iy as usize * grid.width + ix as usize] = true;
            }
        }
    }
    grid
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

/// Consecutive-centroid odometry. Not a loop-closing SLAM.
#[derive(Debug, Clone, Default)]
pub struct VisualOdometry {
    last_xy: Option<(f64, f64)>,
    pose: Pose2,
}

impl VisualOdometry {
    pub fn new() -> Self {
        Self {
            last_xy: None,
            pose: Pose2 {
                x: 0.0,
                y: 0.0,
                yaw: 0.0,
            },
        }
    }

    pub fn pose(&self) -> Pose2 {
        self.pose
    }

    pub fn step(&mut self, seen_xy_m: Option<(f64, f64)>) -> Pose2 {
        if let (Some((x, y)), Some((px, py))) = (seen_xy_m, self.last_xy) {
            if x.is_finite() && y.is_finite() {
                self.pose.x += x - px;
                self.pose.y += y - py;
            }
        }
        self.last_xy = seen_xy_m;
        self.pose
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Camera;

    #[test]
    fn depth_occupancy_and_vo_are_not_slam() {
        let cam = Camera::default_workcell(8, 8);
        let depth = DepthFrame::planar(8, 8, 0.75);
        let grid = integrate_depth(&cam, &depth, 2);
        assert!(grid.occupied_count() > 0);
        let mut vo = VisualOdometry::new();
        vo.step(Some((0.0, 0.0)));
        let p = vo.step(Some((0.1, 0.0)));
        assert!((p.x - 0.1).abs() < 1e-12);
        assert!(unproject_depth(&cam, cam.cx, cam.cy, 0.75).is_some());
        assert!(unproject_depth(&cam, cam.cx, cam.cy, -1.0).is_none());
    }
}
