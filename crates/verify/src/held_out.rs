//! Held-out robot bundle loader. Not part of the milestone corpus.

use std::path::PathBuf;

pub fn held_out_bundle() -> PathBuf {
    crate::corpus::bundled_robots_root().join("wrist_offset_arm")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    #[test]
    fn held_out_bundle_exists_and_semantics_crate_does_not_name_it() {
        let p = held_out_bundle();
        assert!(p.join("robot.yaml").exists());
        let sem = Path::new(env!("CARGO_MANIFEST_DIR")).join("../semantics/src");
        for ent in std::fs::read_dir(sem).unwrap() {
            let path = ent.unwrap().path();
            if path.extension().and_then(|e| e.to_str()) == Some("rs") {
                let t = std::fs::read_to_string(&path).unwrap();
                assert!(!t.contains("wrist_offset_arm"), "{}", path.display());
            }
        }
    }
}
