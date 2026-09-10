//! Canonical RobotBundle. Manifest supplies semantics MuJoCo cannot infer.

use crate::format::{detect_format, FormatDiagnosis, ModelFormat};
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
    pub source: Option<SourceLicense>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobotBundle {
    pub root: PathBuf,
    pub manifest: RobotYaml,
    pub model_path: PathBuf,
    pub model_text: String,
    pub format: FormatDiagnosis,
    pub asset_files: Vec<PathBuf>,
    pub source_hash: String,
}

impl RobotBundle {
    pub fn load(root: impl AsRef<Path>) -> Result<Self, BundleError> {
        let root = root.as_ref().to_path_buf();
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
        let model_path = root.join(&manifest.model_file);
        if !model_path.exists() {
            return Err(BundleError::Missing(model_path.display().to_string()));
        }
        let model_text =
            fs::read_to_string(&model_path).map_err(|e| BundleError::Parse(e.to_string()))?;
        let format = detect_format(&model_path, &model_text);
        if matches!(
            format.format,
            ModelFormat::Sdf | ModelFormat::Step | ModelFormat::Unknown
        ) && !format.is_loadable()
        {
            return Err(BundleError::Unsupported(format.detail.clone()));
        }
        let mut asset_files = Vec::new();
        for rel in &manifest.asset_roots {
            let dir = root.join(rel);
            if dir.is_dir() {
                collect_files(&dir, &mut asset_files);
            }
        }
        asset_files.sort();
        let source_hash = hash_bundle(&yaml_text, &model_text, &asset_files);
        Ok(Self {
            root,
            manifest,
            model_path,
            model_text,
            format,
            asset_files,
            source_hash,
        })
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

fn hash_bundle(yaml: &str, model: &str, assets: &[PathBuf]) -> String {
    let mut h = Sha256::new();
    h.update(b"realityos.robot_bundle/1\0");
    h.update(yaml.as_bytes());
    h.update(b"\0");
    h.update(model.as_bytes());
    for a in assets {
        h.update(a.to_string_lossy().as_bytes());
        if let Ok(bytes) = fs::read(a) {
            h.update(&bytes);
        }
    }
    hex::encode(h.finalize())
}
