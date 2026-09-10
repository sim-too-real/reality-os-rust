//! Canonical RobotBundle. Manifest supplies semantics MuJoCo cannot infer.

use crate::format::{
    detect_format, detect_format_path, parse_declared_format, FormatDiagnosis, ModelFormat,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs;
use std::path::{Path, PathBuf};
use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum BundleError {
    #[error("bundle path missing: {0}")]
    Missing(String),
    #[error("parse: {0}")]
    Parse(String),
    #[error("unsupported model: {0}")]
    Unsupported(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BaseType {
    Fixed,
    Floating,
    Mobile,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct NamedRef {
    pub name: String,
    #[serde(default)]
    pub site: Option<String>,
    #[serde(default)]
    pub body: Option<String>,
    #[serde(default)]
    pub joint: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct SourceLicense {
    #[serde(default)]
    pub license: String,
    #[serde(default)]
    pub attribution: String,
    #[serde(default)]
    pub redistributable: bool,
    #[serde(default)]
    pub notes: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RobotYaml {
    pub robot_id: String,
    pub model_format: String,
    pub model_file: String,
    #[serde(default)]
    pub asset_roots: Vec<String>,
    pub expected_base_type: BaseType,
    #[serde(default)]
    pub joint_aliases: std::collections::BTreeMap<String, String>,
    #[serde(default)]
    pub actuator_aliases: std::collections::BTreeMap<String, String>,
    #[serde(default)]
    pub end_effectors: Vec<NamedRef>,
    #[serde(default)]
    pub grippers: Vec<NamedRef>,
    #[serde(default)]
    pub feet: Vec<NamedRef>,
    #[serde(default)]
    pub cameras: Vec<NamedRef>,
    #[serde(default)]
    pub task_frames: Vec<NamedRef>,
    #[serde(default)]
    pub collision_groups: std::collections::BTreeMap<String, Vec<String>>,
    #[serde(default)]
    pub default_controller_profile: Option<String>,
    #[serde(default)]
    pub effort_limit: Option<f64>,
    #[serde(default)]
    pub effort_units: Option<String>,
    #[serde(default)]
    pub source: Option<SourceLicense>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotBundle {
    pub root: PathBuf,
    pub manifest: RobotYaml,
    pub model_path: PathBuf,
    pub model_text: String,
    pub model_bytes: Vec<u8>,
    pub format: FormatDiagnosis,
    pub asset_files: Vec<PathBuf>,
    pub source_hash: String,
}

impl RobotBundle {
    pub fn load(root: impl AsRef<Path>) -> Result<Self, BundleError> {
        let root = dunce_abs(root.as_ref());
        let yaml_path = root.join("robot.yaml");
        if !yaml_path.exists() {
            return Err(BundleError::Missing(yaml_path.display().to_string()));
        }
        let yaml_text =
            fs::read_to_string(&yaml_path).map_err(|e| BundleError::Parse(e.to_string()))?;
        let manifest: RobotYaml =
            serde_yaml::from_str(&yaml_text).map_err(|e| BundleError::Parse(e.to_string()))?;
        if manifest.robot_id.trim().is_empty() {
            return Err(BundleError::Parse("robot_id empty".into()));
        }
        let model_path = dunce_abs(&root.join(&manifest.model_file));
        if !model_path.exists() {
            return Err(BundleError::Missing(model_path.display().to_string()));
        }
        let ext = model_path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        if matches!(ext.as_str(), "usdc" | "usdz") {
            return Err(BundleError::Unsupported(
                crate::format::usd_unsupported(model_path.display().to_string()).detail,
            ));
        }
        let format = if matches!(ext.as_str(), "usd" | "usda") {
            detect_format_path(&model_path)
        } else {
            let text =
                fs::read_to_string(&model_path).map_err(|e| BundleError::Parse(e.to_string()))?;
            detect_format(&model_path, &text)
        };
        if let Some(declared) = parse_declared_format(&manifest.model_format) {
            if declared != format.format {
                return Err(BundleError::Parse(format!(
                    "model_format_mismatch:declared={} detected={:?}",
                    manifest.model_format, format.format
                )));
            }
        }
        if matches!(format.format, ModelFormat::Usd) {
            return Err(BundleError::Unsupported(format.detail.clone()));
        }
        if matches!(
            format.format,
            ModelFormat::Sdf | ModelFormat::Step | ModelFormat::Unknown
        ) && !format.is_loadable()
        {
            return Err(BundleError::Unsupported(format.detail.clone()));
        }
        let model_bytes = fs::read(&model_path).map_err(|e| BundleError::Parse(e.to_string()))?;
        let model_text = if matches!(ext.as_str(), "usdc" | "usdz") {
            String::new()
        } else {
            String::from_utf8_lossy(&model_bytes).into_owned()
        };
        let mut asset_files = Vec::new();
        for rel in &manifest.asset_roots {
            let dir = root.join(rel);
            if dir.is_dir() {
                collect_files(&dir, &mut asset_files);
            }
        }
        asset_files.sort();
        let source_hash = hash_bundle(&root, &yaml_text, &model_bytes, &asset_files);
        Ok(Self {
            root,
            manifest,
            model_path,
            model_text,
            model_bytes,
            format,
            asset_files,
            source_hash,
        })
    }

    pub fn asset_roots_abs(&self) -> Vec<PathBuf> {
        let mut out = vec![self.root.clone()];
        if let Some(parent) = self.model_path.parent() {
            out.push(parent.to_path_buf());
        }
        for rel in &self.manifest.asset_roots {
            out.push(self.root.join(rel));
        }
        out.sort();
        out.dedup();
        out
    }

    pub fn referenced_assets(&self) -> Vec<String> {
        let mut out = Vec::new();
        for line in self.model_text.lines() {
            let l = line.trim();
            for key in ["file=", "meshdir=", "texturefile="] {
                if let Some(idx) = l.find(key) {
                    let rest = &l[idx + key.len()..];
                    let token = rest
                        .trim_start_matches(['"', '\''])
                        .split(['"', '\'', ' ', '>'])
                        .next()
                        .unwrap_or("");
                    if !token.is_empty() && token.contains('.') {
                        out.push(token.to_string());
                    }
                }
            }
        }
        out.sort();
        out.dedup();
        out
    }
}

fn dunce_abs(path: &Path) -> PathBuf {
    fs::canonicalize(path).unwrap_or_else(|_| {
        if path.is_absolute() {
            path.to_path_buf()
        } else {
            std::env::current_dir()
                .map(|cwd| cwd.join(path))
                .unwrap_or_else(|_| path.to_path_buf())
        }
    })
}

fn collect_files(dir: &Path, out: &mut Vec<PathBuf>) {
    if let Ok(rd) = fs::read_dir(dir) {
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                collect_files(&p, out);
            } else {
                out.push(p);
            }
        }
    }
}

fn rel_from(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .replace('\\', "/")
}

fn hash_bundle(root: &Path, yaml: &str, model_bytes: &[u8], assets: &[PathBuf]) -> String {
    let mut h = Sha256::new();
    h.update(b"realityos.robot_bundle/2\0");
    h.update(yaml.as_bytes());
    h.update(b"\0model\0");
    h.update(model_bytes);
    for a in assets {
        h.update(b"\0asset\0");
        h.update(rel_from(root, a).as_bytes());
        h.update(b"\0");
        if let Ok(bytes) = fs::read(a) {
            h.update(&bytes);
        }
    }
    hex::encode(h.finalize())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::FormatDisposition;

    #[test]
    fn binary_usd_is_not_read_as_text() {
        let root = std::env::temp_dir().join(format!("ros-usd-{}", std::process::id()));
        let _ = fs::create_dir_all(&root);
        fs::write(
            root.join("robot.yaml"),
            "robot_id: usd_bot\nmodel_format: usdz\nmodel_file: pkg.usdz\nexpected_base_type: fixed\n",
        )
        .unwrap();
        fs::write(root.join("pkg.usdz"), [0u8, 1, 2, 3, 255]).unwrap();
        let err = RobotBundle::load(&root).unwrap_err();
        match err {
            BundleError::Unsupported(d) => {
                assert!(d.contains("EXPERIMENTAL_UNSUPPORTED_IN_VERIFY_V1"));
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn yaml_format_must_match_detected() {
        let root = std::env::temp_dir().join(format!("ros-fmt-{}", std::process::id()));
        let _ = fs::create_dir_all(&root);
        fs::write(
            root.join("robot.yaml"),
            "robot_id: mismatch\nmodel_format: urdf\nmodel_file: model.xml\nexpected_base_type: fixed\n",
        )
        .unwrap();
        fs::write(root.join("model.xml"), "<mujoco></mujoco>").unwrap();
        let err = RobotBundle::load(&root).unwrap_err();
        assert!(err.to_string().contains("model_format_mismatch"));
        let _ = FormatDisposition::Supported;
    }
}
