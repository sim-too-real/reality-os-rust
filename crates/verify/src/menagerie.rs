//! Pinned official Menagerie fetch/cache. Fixture registry only.

use serde_json::Value;
use std::path::{Path, PathBuf};
use std::process::Command;

pub const MENAGERIE_REPO: &str = "https://github.com/google-deepmind/mujoco_menagerie";
pub const MENAGERIE_SHA: &str = "8161bba264d7fa7c99ca301e91e7fb44737676ad";
pub const DEVELOPMENT_RELPATH: &str = "universal_robots_ur5e";

pub fn external_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/external")
}

pub fn development_bundle_dir() -> PathBuf {
    external_root().join("ur5e")
}

pub fn development_model_dir() -> PathBuf {
    development_bundle_dir().join("model")
}

pub fn ensure_development_model() -> Result<Value, String> {
    let dest = development_model_dir();
    let xml = dest.join("ur5e.xml");
    if xml.exists() {
        let prov = dest.join("PROVENANCE.json");
        if prov.exists() {
            let t = std::fs::read_to_string(prov).map_err(|e| e.to_string())?;
            return serde_json::from_str(&t).map_err(|e| e.to_string());
        }
    }
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts/fetch_menagerie.py");
    dest.parent().map(std::fs::create_dir_all);
    let out = Command::new("python")
        .arg(&script)
        .arg(DEVELOPMENT_RELPATH)
        .arg(&dest)
        .output()
        .map_err(|e| format!("fetch menagerie: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "fetch menagerie failed: {} {}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ));
    }
    let prov = dest.join("PROVENANCE.json");
    let t = std::fs::read_to_string(prov).map_err(|e| e.to_string())?;
    serde_json::from_str(&t).map_err(|e| e.to_string())
}

pub fn development_bundle_ready() -> bool {
    Path::new(&development_bundle_dir().join("robot.yaml")).exists()
        && development_model_dir().join("ur5e.xml").exists()
}
