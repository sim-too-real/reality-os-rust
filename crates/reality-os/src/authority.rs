//! Learned systems never hold final actuation authority.

use serde::{Deserialize, Serialize};

pub const LEARNED_SOURCE_MARKERS: &[&str] = &[
    "vla",
    "vlm",
    "rl",
    "policy",
    "world_model",
    "learned",
    "neural",
    "gemini",
    "embodied_reasoner",
    "embodied-reasoner",
    "er2",
    "er_2",
];

const FORBIDDEN_TOOLS: &[&str] = &[
    "move_actuator",
    "write_motors",
    "set_torque",
    "plant_act",
    "bypass_governor",
];

pub fn is_learned_source(source: &str) -> bool {
    let s = source.to_ascii_lowercase();
    LEARNED_SOURCE_MARKERS.iter().any(|m| s.contains(m))
}

pub fn is_forbidden_tool(name: &str) -> bool {
    let n = name.to_ascii_lowercase();
    FORBIDDEN_TOOLS.iter().any(|t| n == *t)
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorityScreen {
    pub executable: bool,
    pub learned_actuator_authority: bool,
    pub reason: String,
}

pub fn screen_external_proposal(source: &str) -> AuthorityScreen {
    if is_learned_source(source) {
        AuthorityScreen {
            executable: false,
            learned_actuator_authority: false,
            reason: "learned_source_is_proposal_only".into(),
        }
    } else {
        AuthorityScreen {
            executable: false,
            learned_actuator_authority: false,
            reason: "external_proposal_must_still_certify".into(),
        }
    }
}

pub fn assert_no_learned_actuator_authority(
    source: &str,
    has_certificate: bool,
    acknowledged: bool,
) -> Result<(), String> {
    if !is_learned_source(source) {
        return Ok(());
    }
    if has_certificate && acknowledged {
        return Err("learned_source_cannot_hold_governor_ack".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vla_is_not_executable() {
        let s = screen_external_proposal("openvla");
        assert!(!s.executable);
        assert!(!s.learned_actuator_authority);
        assert!(is_forbidden_tool("move_actuator"));
        assert!(!is_forbidden_tool("plan_certify_decide"));
    }
}
