//! Learned systems never hold final actuation authority.
//!
//! Source strings are diagnostic. Typed [`ProposalClass`] is the authority
//! class. No proposal class can mint a certificate, acknowledge, sign, or
//! execute.

use serde::{Deserialize, Serialize};

/// Diagnostic labels only. Not a security boundary.
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
    "grok",
    "xai",
];

const FORBIDDEN_TOOLS: &[&str] = &[
    "move_actuator",
    "write_motors",
    "set_torque",
    "plant_act",
    "bypass_governor",
];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum ProposalClass {
    UntrustedLearned,
    #[default]
    ExternalDeterministic,
    Operator,
}

impl ProposalClass {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::UntrustedLearned => "untrusted_learned",
            Self::ExternalDeterministic => "external_deterministic",
            Self::Operator => "operator",
        }
    }

    pub const fn can_construct_certificate(self) -> bool {
        false
    }
    pub const fn can_acknowledge(self) -> bool {
        false
    }
    pub const fn can_sign(self) -> bool {
        false
    }
    pub const fn can_execute(self) -> bool {
        false
    }

    /// Mapping from a leftover source note. Telemetry only.
    pub fn from_source_note(source: &str) -> Self {
        if is_learned_source(source) {
            Self::UntrustedLearned
        } else if source.eq_ignore_ascii_case("operator") || source.eq_ignore_ascii_case("language")
        {
            Self::Operator
        } else {
            Self::ExternalDeterministic
        }
    }
}

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
    pub class: ProposalClass,
    pub reason: String,
}

pub fn screen_proposal(class: ProposalClass) -> AuthorityScreen {
    AuthorityScreen {
        executable: false,
        learned_actuator_authority: false,
        class,
        reason: match class {
            ProposalClass::UntrustedLearned => "learned_source_is_proposal_only".into(),
            ProposalClass::ExternalDeterministic => "external_proposal_must_still_certify".into(),
            ProposalClass::Operator => "operator_proposal_must_still_certify".into(),
        },
    }
}

/// Deprecated string entry point. Classifies then screens. Never grants execution.
pub fn screen_external_proposal(source: &str) -> AuthorityScreen {
    screen_proposal(ProposalClass::from_source_note(source))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_proposal_class_holds_authority() {
        for c in [
            ProposalClass::UntrustedLearned,
            ProposalClass::ExternalDeterministic,
            ProposalClass::Operator,
        ] {
            assert!(!c.can_construct_certificate());
            assert!(!c.can_acknowledge());
            assert!(!c.can_sign());
            assert!(!c.can_execute());
            assert!(!screen_proposal(c).executable);
        }
        assert!(is_forbidden_tool("move_actuator"));
        assert!(!is_forbidden_tool("plan_certify_decide"));
    }
}
