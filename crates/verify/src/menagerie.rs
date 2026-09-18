//! Pinned official Menagerie fetch/cache. Fixture registry only.

use serde_json::Value;
use std::path::{Path, PathBuf};
use std::process::Command;

pub const MENAGERIE_REPO: &str = "https://github.com/google-deepmind/mujoco_menagerie";
pub const MENAGERIE_SHA: &str = "8161bba264d7fa7c99ca301e91e7fb44737676ad";
pub const DEVELOPMENT_RELPATH: &str = "universal_robots_ur5e";
pub const HOLDOUT_RELPATH: &str = "franka_emika_panda";
pub const V2_HOLDOUT_RELPATH: &str = "kuka_iiwa_14";
pub const MANIPULATION_HOLDOUT_RELPATH: &str = "trossen_wx250s";

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
    fetch_rel(DEVELOPMENT_RELPATH, &development_model_dir())
}

pub fn development_bundle_ready() -> bool {
    Path::new(&development_bundle_dir().join("robot.yaml")).exists()
        && development_model_dir().join("ur5e.xml").exists()
}

pub fn holdout_bundle_dir() -> PathBuf {
    external_root().join("panda")
}

pub fn holdout_model_dir() -> PathBuf {
    holdout_bundle_dir().join("model")
}

pub fn ensure_holdout_model() -> Result<Value, String> {
    fetch_rel(HOLDOUT_RELPATH, &holdout_model_dir())
}

pub fn v2_holdout_bundle_dir() -> PathBuf {
    external_root().join("iiwa14")
}

pub fn v2_holdout_model_dir() -> PathBuf {
    v2_holdout_bundle_dir().join("model")
}

pub fn ensure_v2_holdout_model() -> Result<Value, String> {
    fetch_rel(V2_HOLDOUT_RELPATH, &v2_holdout_model_dir())
}

pub fn manipulation_holdout_bundle_dir() -> PathBuf {
    external_root().join("wx250s")
}

pub fn manipulation_holdout_model_dir() -> PathBuf {
    manipulation_holdout_bundle_dir().join("model")
}

pub fn ensure_manipulation_holdout_model() -> Result<Value, String> {
    fetch_rel(
        MANIPULATION_HOLDOUT_RELPATH,
        &manipulation_holdout_model_dir(),
    )
}

fn fetch_rel(rel: &str, dest: &Path) -> Result<Value, String> {
    let xmls = std::fs::read_dir(dest).ok().map(|it| {
        it.filter_map(|e| e.ok())
            .any(|e| e.path().extension().and_then(|x| x.to_str()) == Some("xml"))
    });
    let prov = dest.join("PROVENANCE.json");
    if xmls == Some(true) && prov.exists() {
        let t = std::fs::read_to_string(prov).map_err(|e| e.to_string())?;
        return serde_json::from_str(&t).map_err(|e| e.to_string());
    }
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("scripts/fetch_menagerie.py");
    if let Some(parent) = dest.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let out = Command::new("python")
        .arg(&script)
        .arg(rel)
        .arg(dest)
        .output()
        .map_err(|e| format!("fetch menagerie: {e}"))?;
    if !out.status.success() {
        return Err(format!(
            "fetch menagerie failed: {} {}",
            String::from_utf8_lossy(&out.stdout),
            String::from_utf8_lossy(&out.stderr)
        ));
    }
    let t = std::fs::read_to_string(dest.join("PROVENANCE.json")).map_err(|e| e.to_string())?;
    serde_json::from_str(&t).map_err(|e| e.to_string())
}
