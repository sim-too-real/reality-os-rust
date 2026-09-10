//! Physical-intelligence schemas. No plant I/O. SIM ≠ METAL.

pub mod adapter;
pub mod capability;
pub mod skill;
pub mod embodiment;
pub mod observation;
pub mod provenance;
pub mod reach;
pub mod sensor;
pub mod transform;
pub mod world;

pub const SCHEMA_FAMILY: &str = "realityos.semantics/1";

#[cfg(test)]
mod quarantine {
    #[test]
    fn semantics_sources_do_not_name_held_out_robot() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        for ent in std::fs::read_dir(root).unwrap() {
            let p = ent.unwrap().path();
            if p.extension().and_then(|e| e.to_str()) == Some("rs") {
                if p.file_name().and_then(|n| n.to_str()) == Some("lib.rs") {
                    continue;
                }
                let t = std::fs::read_to_string(&p).unwrap();
                assert!(!t.contains("wrist_offset_arm"), "{}", p.display());
            }
        }
    }
}
