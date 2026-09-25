//! Physical-intelligence schemas. No plant I/O. SIM ≠ METAL.

pub mod adapter;
pub mod allowed_contact;
pub mod capability;
pub mod command;
pub mod command_domain;
pub mod contact;
pub mod contact_collision;
pub mod contact_jacobian;
pub mod contact_maneuver;
pub mod contact_manifold;
pub mod discrepancy;
pub mod effect_feasibility;
pub mod effort;
pub mod embodiment;
pub mod execution_envelope;
pub mod failure;
pub mod geometry;
pub mod geometry_fk;
pub mod geometry_query;
pub mod goal_loop;
pub mod grasp;
pub mod grasp_hold;
pub mod gripper_state;
pub mod interaction;
pub mod kinematics;
pub mod maneuver_witness;
pub mod mechanics_regime;
pub mod object;
pub mod observation;
pub mod pair_friction;
pub mod physical_belief;
pub mod physical_experience;
pub mod physical_interaction;
pub mod physical_quantity;
pub mod plan;
pub mod planar_goal;
pub mod probe_selection;
pub mod provenance;
pub mod push;
pub mod reach;
pub mod recoverability;
pub mod release;
pub mod resource;
pub mod runtime_interaction;
pub mod self_load;
pub mod sensor;
pub mod skill;
pub mod transform;
pub mod transition_validity;
pub mod workspace;
pub mod world;

pub const SCHEMA_FAMILY: &str = "realityos.semantics/1";

#[cfg(test)]
mod quarantine {
    #[test]
    fn semantics_sources_do_not_name_held_out_robot() {
        let forbidden = [
            ["wrist", "offset", "arm"].join("_"),
            ["ur", "5e"].join(""),
            ["ur", "5"].join(""),
            ["pan", "da"].join(""),
            ["fran", "ka"].join(""),
            ["ku", "ka"].join(""),
            ["ii", "wa"].join(""),
        ];
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        for ent in std::fs::read_dir(root).unwrap() {
            let p = ent.unwrap().path();
            if p.extension().and_then(|e| e.to_str()) == Some("rs") {
                let t = std::fs::read_to_string(&p).unwrap();
                let lower = t.to_ascii_lowercase();
                for needle in &forbidden {
                    assert!(
                        !lower.contains(needle),
                        "{} contains forbidden identifier {needle}",
                        p.display()
                    );
                }
            }
        }
    }
}
