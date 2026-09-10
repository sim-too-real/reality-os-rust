//! Static model validation before a task runs. No silent physical auto-repair.

use crate::bundle::RobotBundle;
use crate::normalize::RobotManifest;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum ValidationStatus {
    Valid,
    ValidWithWarnings,
    Invalid,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ValidationReport {
    pub status: ValidationStatus,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
    pub modifications: Vec<String>,
}

impl ValidationReport {
    pub fn ok(&self) -> bool {
        self.status != ValidationStatus::Invalid
    }
}

pub fn validate_bundle(bundle: &RobotBundle, manifest: Option<&RobotManifest>) -> ValidationReport {
    let mut errors = Vec::new();
    let mut warnings = Vec::new();
    for rel in bundle.referenced_assets() {
        let p = Path::new(&rel);
        let candidates = [
            bundle.root.join(&rel),
            bundle
                .model_path
                .parent()
                .unwrap_or(&bundle.root)
                .join(&rel),
            bundle
                .root
                .join("assets")
                .join(p.file_name().unwrap_or_default()),
        ];
        if !candidates.iter().any(|c| c.exists()) {
            errors.push(format!("missing_asset:{rel}"));
        }
    }
    if !bundle.format.is_loadable() {
        errors.push(bundle.format.detail.clone());
    }
    warnings.extend(bundle.format.lost_or_unreliable.iter().cloned());

    if let Some(m) = manifest {
        validate_compiled(m, &mut errors, &mut warnings);
    }
    let status = if !errors.is_empty() {
        ValidationStatus::Invalid
    } else if !warnings.is_empty() {
        ValidationStatus::ValidWithWarnings
    } else {
        ValidationStatus::Valid
    };
    ValidationReport {
        status,
        errors,
        warnings,
        modifications: Vec::new(),
    }
}

fn validate_compiled(m: &RobotManifest, errors: &mut Vec<String>, warnings: &mut Vec<String>) {
    if !(m.timestep.is_finite() && m.timestep > 0.0 && m.timestep < 0.1) {
        errors.push(format!("bad_timestep:{}", m.timestep));
    }
    let mut names = BTreeSet::new();
    for j in &m.joints {
        if !names.insert(format!("j:{}", j.name)) {
            errors.push(format!("duplicate_name:{}", j.name));
        }
        if j.limited
            && !(j.range[0].is_finite() && j.range[1].is_finite() && j.range[0] <= j.range[1])
        {
            errors.push(format!("invalid_joint_limits:{}", j.name));
        }
    }
    for a in &m.actuators {
        if !names.insert(format!("a:{}", a.name)) {
            errors.push(format!("duplicate_name:{}", a.name));
        }
        if a.transmission_target.is_empty() {
            errors.push(format!("actuator_missing_transmission:{}", a.name));
        } else if !m.joints.iter().any(|j| j.name == a.transmission_target)
            && !m.bodies.iter().any(|b| b.name == a.transmission_target)
        {
            errors.push(format!("actuator_missing_transmission_target:{}", a.name));
        }
        if !a.ctrllimited {
            warnings.push(format!("unbounded_actuator_control:{}", a.name));
        }
        if !a.ctrlrange[0].is_finite() || !a.ctrlrange[1].is_finite() {
            errors.push(format!("nan_inf_actuator:{}", a.name));
        }
    }
    for b in &m.bodies {
        if b.name == "world" {
            continue;
        }
        if !b.mass.is_finite() || b.mass < 0.0 {
            errors.push(format!("invalid_mass:{}", b.name));
        } else if b.mass == 0.0 {
            warnings.push(format!("zero_mass:{}", b.name));
        }
        if b.inertia.iter().any(|x| !x.is_finite() || *x < 0.0) {
            errors.push(format!("invalid_inertia:{}", b.name));
        }
    }
    for c in &m.cameras {
        if !m.bodies.iter().any(|b| b.name == c.parent_body) {
            errors.push(format!("invalid_camera_parent:{}", c.name));
        }
    }
    if m.nu == 0 {
        errors.push("no_actuators".into());
    }
    warnings.extend(m.lost_features.iter().cloned());
}

/// Runtime checks after compile + short passive rollout. Does not repair parameters.
pub fn validate_runtime(
    report: &mut ValidationReport,
    inspect: &serde_json::Value,
    reset_state: &serde_json::Value,
    rollout: Option<&serde_json::Value>,
) {
    if let Some(ws) = inspect.get("warnings").and_then(|v| v.as_array()) {
        for w in ws {
            if let Some(s) = w.as_str() {
                report.warnings.push(format!("compile_warning:{s}"));
            }
        }
    }
    if reset_state["nan"].as_bool() == Some(true) {
        report.errors.push("nan_state_at_reset".into());
    }
    if let Some(cons) = reset_state.get("contacts").and_then(|v| v.as_array()) {
        for c in cons {
            let dist = c["dist"].as_f64().unwrap_or(0.0);
            let b1 = c["body1"].as_str().unwrap_or("");
            let b2 = c["body2"].as_str().unwrap_or("");
            if dist < -1e-4
                && b1 != "world"
                && b2 != "world"
                && !b1.contains("floor")
                && !b2.contains("floor")
            {
                report
                    .warnings
                    .push(format!("initial_penetration:{b1}-{b2}:{dist}"));
            }
        }
    }
    if let Some(r) = rollout {
        if r["nan"].as_bool() == Some(true) {
            report
                .errors
                .push("exploding_dynamics_passive_rollout".into());
        }
    }
    report.status = if !report.errors.is_empty() {
        ValidationStatus::Invalid
    } else if !report.warnings.is_empty() {
        ValidationStatus::ValidWithWarnings
    } else {
        ValidationStatus::Valid
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bundle::RobotBundle;
    use std::fs;
    use std::path::PathBuf;

    fn temp_bundle(id: &str, xml: &str, extra_yaml: &str) -> PathBuf {
        let root = std::env::temp_dir().join(format!("ros-verify-{id}-{}", std::process::id()));
        let _ = fs::create_dir_all(&root);
        fs::write(
            root.join("robot.yaml"),
            format!(
                "robot_id: {id}\nmodel_format: mjcf\nmodel_file: model.xml\nexpected_base_type: fixed\n{extra_yaml}"
            ),
        )
        .unwrap();
        fs::write(root.join("model.xml"), xml).unwrap();
        root
    }

    #[test]
    fn missing_mesh_is_invalid() {
        let xml = r#"<mujoco><asset><mesh name="m" file="missing_mesh.stl"/></asset><worldbody><body><geom mesh="m"/></body></worldbody></mujoco>"#;
        let root = temp_bundle("miss", xml, "");
        let b = RobotBundle::load(&root).unwrap();
        let r = validate_bundle(&b, None);
        assert_eq!(r.status, ValidationStatus::Invalid);
        assert!(r.errors.iter().any(|e| e.contains("missing_asset")));
    }
}
