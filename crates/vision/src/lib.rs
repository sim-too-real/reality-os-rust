//! Pixel see. Lookat/hint is search, never pose. Missing camera / no pixels refuse.

mod depth;
pub use depth::{
    integrate_depth, unproject_depth, DepthFrame, OccupancyGrid, Pose2, VisualOdometry,
};

use realityos_kernel::{DecisionStatus, KernelResult, ObservationEvidence};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Debug, Clone)]
pub struct Frame {
    pub width: u32,
    pub height: u32,
    pub rgb: Vec<u8>,
}

impl Frame {
    pub fn new(width: u32, height: u32, rgb: Vec<u8>) -> Result<Self, &'static str> {
        let n = (width as usize) * (height as usize) * 3;
        if rgb.len() != n {
            return Err("frame_len_mismatch");
        }
        Ok(Self { width, height, rgb })
    }

    pub fn black(width: u32, height: u32) -> Self {
        Self {
            width,
            height,
            rgb: vec![0; (width as usize) * (height as usize) * 3],
        }
    }

    /// Bright blob at (cx, cy) for tests.
    pub fn with_blob(width: u32, height: u32, cx: u32, cy: u32, radius: u32) -> Self {
        let mut f = Self::black(width, height);
        for y in 0..height {
            for x in 0..width {
                let dx = x as i32 - cx as i32;
                let dy = y as i32 - cy as i32;
                if dx * dx + dy * dy <= (radius as i32) * (radius as i32) {
                    let i = ((y * width + x) * 3) as usize;
                    f.rgb[i] = 220;
                    f.rgb[i + 1] = 40;
                    f.rgb[i + 2] = 30;
                }
            }
        }
        f
    }

    pub fn content_hash(&self) -> String {
        hex::encode(Sha256::digest(&self.rgb))
    }

    pub fn n_pixels(&self) -> usize {
        (self.width as usize) * (self.height as usize)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Camera {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub table_z_m: f64,
}

impl Camera {
    pub fn default_workcell(width: u32, height: u32) -> Self {
        Self {
            fx: width as f64,
            fy: height as f64,
            cx: width as f64 / 2.0,
            cy: height as f64 / 2.0,
            table_z_m: 0.75,
        }
    }

    /// Pinhole ray onto z = table_z (camera at origin looking +z). SIM geometry.
    pub fn unproject(&self, u: f64, v: f64) -> (f64, f64) {
        let x = (u - self.cx) / self.fx * self.table_z_m;
        let y = (v - self.cy) / self.fy * self.table_z_m;
        (x, y)
    }

    pub fn validate(&self, width: u32, height: u32) -> Result<(), &'static str> {
        if !self.fx.is_finite() || !self.fy.is_finite() || self.fx <= 0.0 || self.fy <= 0.0 {
            return Err("camera_focal_invalid");
        }
        if !self.cx.is_finite() || !self.cy.is_finite() || !self.table_z_m.is_finite() {
            return Err("camera_principal_non_finite");
        }
        if self.table_z_m <= 0.0 {
            return Err("camera_table_z_non_positive");
        }
        if self.cx < 0.0 || self.cy < 0.0 || self.cx > f64::from(width) || self.cy > f64::from(height)
        {
            return Err("camera_principal_outside_frame");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FrameStats {
    pub fill: f64,
    pub mean: f64,
    pub contrast: f64,
    pub edge_mass: f64,
}

pub fn frame_stats(frame: &Frame) -> FrameStats {
    let n = frame.n_pixels().max(1) as f64;
    let mut sum = 0.0;
    let mut sum2 = 0.0;
    let mut lit = 0.0;
    let mut edge = 0.0;
    let w = frame.width as usize;
    let h = frame.height as usize;
    for y in 0..h {
        for x in 0..w {
            let i = (y * w + x) * 3;
            let yv = 0.299 * f64::from(frame.rgb[i])
                + 0.587 * f64::from(frame.rgb[i + 1])
                + 0.114 * f64::from(frame.rgb[i + 2]);
            sum += yv;
            sum2 += yv * yv;
            if frame.rgb[i] > 130 {
                lit += 1.0;
                if x < 2 || y < 2 || x + 2 >= w || y + 2 >= h {
                    edge += 1.0;
                }
            }
        }
    }
    let mean = sum / n;
    let var = (sum2 / n - mean * mean).max(0.0);
    FrameStats {
        fill: lit / n,
        mean: mean / 255.0,
        contrast: (var.sqrt() / 255.0).clamp(0.0, 1.0),
        edge_mass: if lit > 0.0 { edge / lit } else { 1.0 },
    }
}

pub fn quality_from_stats(stats: &FrameStats, has_blob: bool) -> (f64, f64) {
    if !has_blob {
        return (0.12, 0.92);
    }
    let quality = (0.35 + 0.4 * stats.contrast + 0.25 * (1.0 - stats.edge_mass)).clamp(0.0, 1.0);
    let ood = (stats.edge_mass * 0.6 + (stats.fill - 0.25).abs() * 0.8).clamp(0.0, 1.0);
    (quality, ood)
}

pub fn centroid_uv(frame: &Frame, r_min: u8) -> Option<(f64, f64)> {
    let mut su = 0.0;
    let mut sv = 0.0;
    let mut n = 0.0;
    let w = frame.width as usize;
    for y in 0..frame.height as usize {
        for x in 0..w {
            let i = (y * w + x) * 3;
            let r = frame.rgb[i];
            let g = frame.rgb[i + 1];
            let b = frame.rgb[i + 2];
            if r > r_min && r > g.saturating_add(30) && r > b.saturating_add(30) {
                su += x as f64;
                sv += y as f64;
                n += 1.0;
            }
        }
    }
    if n < 8.0 {
        None
    } else {
        Some((su / n, sv / n))
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SeeResult {
    pub ok: bool,
    pub status: DecisionStatus,
    pub seen_xy_m: Option<(f64, f64)>,
    pub pose_std_m: f64,
    pub n_lit: u32,
    pub source: String,
    pub reason: String,
    pub pixel_hash: String,
    pub metal: bool,
}

pub fn see_from_pixels(frame: &Frame, camera: Option<&Camera>) -> SeeResult {
    let hash = frame.content_hash();
    let Some(cam) = camera else {
        return SeeResult {
            ok: false,
            status: DecisionStatus::Refuse,
            seen_xy_m: None,
            pose_std_m: 0.08,
            n_lit: 0,
            source: "missing_camera".into(),
            reason: "missing_camera".into(),
            pixel_hash: hash,
            metal: false,
        };
    };
    if let Err(reason) = cam.validate(frame.width, frame.height) {
        return SeeResult {
            ok: false,
            status: DecisionStatus::Refuse,
            seen_xy_m: None,
            pose_std_m: 0.08,
            n_lit: 0,
            source: "invalid_camera".into(),
            reason: reason.into(),
            pixel_hash: hash,
            metal: false,
        };
    }
    match centroid_uv(frame, 130) {
        None => SeeResult {
            ok: false,
            status: DecisionStatus::Probe,
            seen_xy_m: None,
            pose_std_m: 0.08,
            n_lit: 0,
            source: "no_blob".into(),
            reason: "pixels_present_but_no_object".into(),
            pixel_hash: hash,
            metal: false,
        },
        Some((u, v)) => {
            let xy = cam.unproject(u, v);
            let stats = frame_stats(frame);
            let n_lit = (stats.fill * frame.n_pixels() as f64) as u32;
            SeeResult {
                ok: true,
                status: DecisionStatus::Allow,
                seen_xy_m: Some(xy),
                pose_std_m: (0.008 + 0.04 * stats.edge_mass).min(0.08),
                n_lit,
                source: "pinhole_centroid".into(),
                reason: "compiled_from_pixels".into(),
                pixel_hash: hash,
                metal: false,
            }
        }
    }
}

pub fn compile_observation(
    frame: &Frame,
    camera: &Camera,
    sensor_id: &str,
    now_s: f64,
    ttl_s: f64,
) -> KernelResult<ObservationEvidence> {
    camera
        .validate(frame.width, frame.height)
        .map_err(|e| realityos_kernel::KernelError::validation("camera", e))?;
    let has_blob = centroid_uv(frame, 130).is_some();
    let (quality, ood) = quality_from_stats(&frame_stats(frame), has_blob);
    ObservationEvidence::new(
        sensor_id,
        format!("cam-{:.3}-{:.3}", camera.fx, camera.fy),
        now_s,
        now_s,
        frame.content_hash(),
        "vision/optical".to_string(),
        quality,
        ood,
        now_s + ttl_s,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blob_unprojects_near_center() {
        let f = Frame::with_blob(64, 64, 32, 32, 4);
        let cam = Camera::default_workcell(64, 64);
        let s = see_from_pixels(&f, Some(&cam));
        assert!(s.ok);
        let (x, y) = s.seen_xy_m.unwrap();
        assert!(x.abs() < 0.05 && y.abs() < 0.05);
        assert!(!s.metal);
    }

    #[test]
    fn missing_camera_refuses() {
        let f = Frame::with_blob(16, 16, 8, 8, 2);
        let s = see_from_pixels(&f, None);
        assert!(!s.ok);
        assert_eq!(s.status, DecisionStatus::Refuse);
    }

    #[test]
    fn black_frame_probes() {
        let f = Frame::black(16, 16);
        let cam = Camera::default_workcell(16, 16);
        let s = see_from_pixels(&f, Some(&cam));
        assert_eq!(s.status, DecisionStatus::Probe);
    }

    #[test]
    fn compile_observation_is_not_a_boolean() {
        let f = Frame::with_blob(16, 16, 8, 8, 2);
        let cam = Camera::default_workcell(16, 16);
        let ev = compile_observation(&f, &cam, "cam0", 1.0, 5.0).unwrap();
        assert!(!ev.digest().is_empty());
        assert!(!ev.is_expired(2.0));
    }

    #[test]
    fn invalid_camera_refuses() {
        let f = Frame::with_blob(16, 16, 8, 8, 2);
        let cam = Camera {
            fx: f64::NAN,
            fy: 16.0,
            cx: 8.0,
            cy: 8.0,
            table_z_m: 0.75,
        };
        let s = see_from_pixels(&f, Some(&cam));
        assert!(!s.ok);
        assert!(compile_observation(&f, &cam, "cam0", 1.0, 2.0).is_err());
    }
}
