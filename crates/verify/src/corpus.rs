//! Small legally redistributable original corpus. No third-party meshes.

use std::path::{Path, PathBuf};

pub fn bundled_robots_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../robots/bundles")
}

pub fn milestone_robots() -> Vec<PathBuf> {
    let root = bundled_robots_root();
    ["planar_arm", "arm_gripper", "cartpole"]
        .into_iter()
        .map(|id| root.join(id))
        .collect()
}

pub fn corpus_ids() -> &'static [&'static str] {
    &["planar_arm", "arm_gripper", "cartpole"]
}

pub fn robot_dir(id: &str) -> PathBuf {
    bundled_robots_root().join(id)
}

pub fn exists_all(root: impl AsRef<Path>) -> bool {
    milestone_robots().iter().all(|p| {
        let _ = root;
        p.join("robot.yaml").exists() && p.join("model.xml").exists()
    })
}
