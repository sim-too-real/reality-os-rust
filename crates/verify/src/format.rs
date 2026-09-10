//! Model format detection. Do not silently accept unsupported models.

use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ModelFormat {
    Mjcf,
    Urdf,
    Usd,
    Sdf,
    Step,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum FormatDisposition {
    Supported,
    ExperimentalUnsupportedInVerifyV1,
    UnsupportedRequiresConversion,
    NotARobotDefinition,
    Unsupported,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FormatDiagnosis {
    pub format: ModelFormat,
    pub disposition: FormatDisposition,
    pub path: String,
    pub detail: String,
    pub lost_or_unreliable: Vec<String>,
}

impl FormatDiagnosis {
    pub fn is_loadable(&self) -> bool {
        matches!(self.disposition, FormatDisposition::Supported)
    }
}

pub fn usd_unsupported(path: impl Into<String>) -> FormatDiagnosis {
    FormatDiagnosis {
        format: ModelFormat::Usd,
        disposition: FormatDisposition::ExperimentalUnsupportedInVerifyV1,
        path: path.into(),
        detail: "EXPERIMENTAL_UNSUPPORTED_IN_VERIFY_V1: USD/USDA/USDC/USDZ is not a verified importer in this crate. MjModel.from_xml_path is not a USD loader.".into(),
        lost_or_unreliable: vec!["usd_not_implemented_in_verify_v1".into()],
    }
}

/// Detect from path without decoding binary USD packages as text.
pub fn detect_format_path(path: &Path) -> FormatDiagnosis {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    if matches!(ext.as_str(), "usd" | "usda" | "usdc" | "usdz") {
        return usd_unsupported(path.display().to_string());
    }
    let contents = std::fs::read_to_string(path).unwrap_or_default();
    detect_format(path, &contents)
}

pub fn parse_declared_format(declared: &str) -> Option<ModelFormat> {
    match declared.trim().to_ascii_lowercase().as_str() {
        "mjcf" | "xml" | "mjb" => Some(ModelFormat::Mjcf),
        "urdf" => Some(ModelFormat::Urdf),
        "usd" | "usda" | "usdc" | "usdz" => Some(ModelFormat::Usd),
        "sdf" => Some(ModelFormat::Sdf),
        "step" | "stp" | "iges" | "igs" => Some(ModelFormat::Step),
        _ => None,
    }
}

pub fn detect_format(path: &Path, contents: &str) -> FormatDiagnosis {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    let trimmed = contents.trim_start();
    let p = path.display().to_string();

    match ext.as_str() {
        "usd" | "usda" | "usdc" | "usdz" => usd_unsupported(p),
        "sdf" => FormatDiagnosis {
            format: ModelFormat::Sdf,
            disposition: FormatDisposition::UnsupportedRequiresConversion,
            path: p,
            detail: "UNSUPPORTED_REQUIRES_CONVERSION: SDF has no validated converter in this crate"
                .into(),
            lost_or_unreliable: vec!["sdf_requires_validated_conversion".into()],
        },
        "step" | "stp" | "iges" | "igs" => FormatDiagnosis {
            format: ModelFormat::Step,
            disposition: FormatDisposition::NotARobotDefinition,
            path: p,
            detail: step_diagnostic(),
            lost_or_unreliable: vec![
                "cad_geometry_is_not_a_robot".into(),
                "missing_links_joints_inertials_limits_actuators".into(),
            ],
        },
        "urdf" => urdf_diagnosis(p, contents),
        "xml" => {
            if trimmed.contains("<robot") && !trimmed.contains("<mujoco") {
                urdf_diagnosis(p, contents)
            } else if trimmed.contains("<mujoco") {
                mjcf_diagnosis(p, contents)
            } else if trimmed.contains("<sdf") {
                FormatDiagnosis {
                    format: ModelFormat::Sdf,
                    disposition: FormatDisposition::UnsupportedRequiresConversion,
                    path: p,
                    detail: "UNSUPPORTED_REQUIRES_CONVERSION".into(),
                    lost_or_unreliable: vec!["sdf_requires_validated_conversion".into()],
                }
            } else {
                FormatDiagnosis {
                    format: ModelFormat::Unknown,
                    disposition: FormatDisposition::Unsupported,
                    path: p,
                    detail: "xml root is neither mujoco nor robot".into(),
                    lost_or_unreliable: vec!["unknown_xml_root".into()],
                }
            }
        }
        "obj" | "stl" | "msh" => FormatDiagnosis {
            format: ModelFormat::Unknown,
            disposition: FormatDisposition::NotARobotDefinition,
            path: p,
            detail: "mesh asset is geometry only; a robot model file is required".into(),
            lost_or_unreliable: vec!["mesh_is_not_a_robot".into()],
        },
        _ if trimmed.contains("<mujoco") => mjcf_diagnosis(p, contents),
        _ => FormatDiagnosis {
            format: ModelFormat::Unknown,
            disposition: FormatDisposition::Unsupported,
            path: p,
            detail: format!("unsupported extension .{ext}"),
            lost_or_unreliable: vec!["unsupported_extension".into()],
        },
    }
}

fn mjcf_diagnosis(path: String, contents: &str) -> FormatDiagnosis {
    let mut lost = Vec::new();
    if contents.contains("compiler") && contents.contains("autolimits") {
        lost.push("autolimits_depend_on_compiler_flags".into());
    }
    FormatDiagnosis {
        format: ModelFormat::Mjcf,
        disposition: FormatDisposition::Supported,
        path,
        detail: "MJCF is the preferred canonical simulation format".into(),
        lost_or_unreliable: lost,
    }
}

fn urdf_diagnosis(path: String, contents: &str) -> FormatDiagnosis {
    let mut lost = Vec::new();
    let low = contents.to_ascii_lowercase();
    if low.contains("<mimic") {
        lost.push("urdf_mimic_not_compiled_as_constraint".into());
    }
    if low.contains("<gazebo") {
        lost.push("gazebo_extensions_ignored".into());
    }
    if low.contains("<transmission") {
        lost.push("urdf_transmission_reduced_to_mujoco_actuator".into());
    }
    if low.contains("type=\"planar\"") {
        lost.push("planar_joint_unsupported_or_reduced".into());
    }
    if low.contains("<calibration") {
        lost.push("urdf_calibration_ignored".into());
    }
    FormatDiagnosis {
        format: ModelFormat::Urdf,
        disposition: FormatDisposition::Supported,
        path,
        detail: "URDF may load where MuJoCo supports it; lost features are recorded".into(),
        lost_or_unreliable: lost,
    }
}

pub fn step_diagnostic() -> String {
    "CAD/STEP/STP geometry is not a robot. Dynamics require links, joints, inertials, joint limits, and actuators. Returning structured diagnostic rather than inventing a kinematic tree.".into()
}

pub fn sdf_unsupported() -> &'static str {
    "UNSUPPORTED_REQUIRES_CONVERSION"
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn mjcf_and_urdf_xml_roots() {
        let mj = detect_format(Path::new("a.xml"), "<mujoco></mujoco>");
        assert_eq!(mj.format, ModelFormat::Mjcf);
        assert!(mj.is_loadable());
        let ur = detect_format(Path::new("b.xml"), "<robot name='x'></robot>");
        assert_eq!(ur.format, ModelFormat::Urdf);
        let ur2 = detect_format(Path::new("c.urdf"), "<robot name='x'><mimic/></robot>");
        assert!(ur2.lost_or_unreliable.iter().any(|s| s.contains("mimic")));
    }

    #[test]
    fn sdf_and_step_are_not_silently_accepted() {
        let sdf = detect_format(&PathBuf::from("r.sdf"), "<sdf></sdf>");
        assert_eq!(
            sdf.disposition,
            FormatDisposition::UnsupportedRequiresConversion
        );
        assert!(sdf.detail.contains(sdf_unsupported()));
        let step = detect_format(Path::new("part.step"), "ISO-10303-21;");
        assert_eq!(step.disposition, FormatDisposition::NotARobotDefinition);
        assert!(step.detail.contains("not a robot"));
    }

    #[test]
    fn usd_is_experimental_unsupported_in_verify_v1() {
        for name in ["r.usd", "r.usda", "r.usdc", "r.usdz"] {
            let d = detect_format(Path::new(name), "#usda 1.0");
            assert_eq!(d.format, ModelFormat::Usd);
            assert_eq!(
                d.disposition,
                FormatDisposition::ExperimentalUnsupportedInVerifyV1
            );
            assert!(d.detail.contains("EXPERIMENTAL_UNSUPPORTED_IN_VERIFY_V1"));
            assert!(!d.is_loadable());
        }
        let bin = detect_format_path(Path::new("pkg.usdz"));
        assert_eq!(
            bin.disposition,
            FormatDisposition::ExperimentalUnsupportedInVerifyV1
        );
    }
}
