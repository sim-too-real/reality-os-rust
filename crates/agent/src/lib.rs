//! Grok proposes skill programs. Reality OS + Governor certify and write.
//! Port of `theworld.integrations.grok_skill_agent` + `llm_admission`.

use realityos_core::{is_forbidden_tool, is_learned_source, Intent, PolicyProposal, SkillIR};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use thiserror::Error;

pub const EVIDENCE_STATUS: &str = "SIM_LLM_PROPOSED_NOT_MEASURED";
pub const GROK_BASE: &str = "https://api.x.ai/v1";
pub const GROK_MODEL: &str = "grok-4.5";

pub const FORBIDDEN_KEYS: &[&str] = &[
    "measured",
    "metal",
    "metal_claim",
    "evidence_status",
    "invent_authority",
    "shop_release",
    "signature",
    "hmac",
    "certified",
    "provenance",
];

pub const ALLOWED_VERBS: &[&str] = &[
    "grasp", "pick", "lift", "hold", "place", "slide", "push", "insert", "press", "carry", "walk",
    "stop", "reach",
];

pub fn skill_registry() -> Vec<SkillIR> {
    ALLOWED_VERBS
        .iter()
        .map(|verb| {
            let mut s = SkillIR::hold();
            s.id = format!("skill.{verb}");
            s.verb = (*verb).into();
            s
        })
        .collect()
}

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum AgentError {
    #[error("llm payload refused: {0}")]
    Refused(String),
    #[error("no api key (set XAI_API_KEY); use offline proposer in tests")]
    NoKey,
    #[error("http: {0}")]
    Http(String),
    #[error("parse: {0}")]
    Parse(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SkillPhase {
    pub name: String,
    pub verb: String,
    #[serde(default)]
    pub target_object: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SkillProgram {
    pub program_name: String,
    #[serde(default)]
    pub notes: String,
    pub phases: Vec<SkillPhase>,
    #[serde(default)]
    pub evidence_status: String,
    #[serde(default)]
    pub metal: bool,
    #[serde(default)]
    pub learned_actuator_authority: bool,
}

impl SkillProgram {
    pub fn intents(&self) -> Vec<Intent> {
        self.phases
            .iter()
            .map(|p| Intent::language(&p.name, &p.verb))
            .collect()
    }
}

pub fn strip_forbidden(v: &mut Value) -> Vec<String> {
    let mut stripped = Vec::new();
    if let Value::Object(map) = v {
        strip_map(map, &mut stripped);
    }
    stripped
}

fn strip_map(map: &mut Map<String, Value>, stripped: &mut Vec<String>) {
    let keys: Vec<String> = map.keys().cloned().collect();
    for k in keys {
        if FORBIDDEN_KEYS.iter().any(|f| k.eq_ignore_ascii_case(f)) {
            map.remove(&k);
            stripped.push(k);
            continue;
        }
        if let Some(Value::Object(child)) = map.get_mut(&k) {
            strip_map(child, stripped);
        }
    }
}

pub fn admit_program(mut raw: Value) -> Result<SkillProgram, AgentError> {
    let stripped = strip_forbidden(&mut raw);
    let mut prog: SkillProgram =
        serde_json::from_value(raw).map_err(|e| AgentError::Parse(e.to_string()))?;
    prog.metal = false;
    prog.learned_actuator_authority = false;
    prog.evidence_status = EVIDENCE_STATUS.into();
    if prog.phases.is_empty() {
        return Err(AgentError::Refused("empty_phases".into()));
    }
    for p in &prog.phases {
        if is_forbidden_tool(&p.verb) {
            return Err(AgentError::Refused(format!("forbidden_verb:{}", p.verb)));
        }
        if !skill_registry().iter().any(|s| s.verb == p.verb) {
            return Err(AgentError::Refused(format!("verb_not_admitted:{}", p.verb)));
        }
    }
    let _ = stripped;
    Ok(prog)
}

/// Deterministic proposer for tests / no-network CI.
pub fn offline_propose(prompt: &str) -> SkillProgram {
    let verb = if prompt.contains("place") {
        "place"
    } else if prompt.contains("walk") {
        "walk"
    } else if prompt.contains("stop") {
        "stop"
    } else {
        "hold"
    };
    SkillProgram {
        program_name: "offline_skill".into(),
        notes: prompt.chars().take(80).collect(),
        phases: vec![SkillPhase {
            name: "phase0".into(),
            verb: verb.into(),
            target_object: String::new(),
        }],
        evidence_status: EVIDENCE_STATUS.into(),
        metal: false,
        learned_actuator_authority: false,
    }
}

pub fn proposal_from_program(prog: &SkillProgram, action: Vec<f64>) -> PolicyProposal {
    let mut p = PolicyProposal::learned(action, "grok_offline");
    p.policy_id = prog.program_name.clone();
    p
}

/// Live Grok chat. Never used on the 1 kHz path. Feature `live-grok`.
#[cfg(feature = "live-grok")]
pub fn grok_propose(prompt: &str) -> Result<SkillProgram, AgentError> {
    let key = std::env::var("XAI_API_KEY").map_err(|_| AgentError::NoKey)?;
    if key.trim().is_empty() {
        return Err(AgentError::NoKey);
    }
    let body = serde_json::json!({
        "model": GROK_MODEL,
        "messages": [
            {"role": "system", "content": "You are a robot skill programmer. Return ONLY JSON {program_name, notes, phases:[{name,verb,target_object}]}. verb in grasp,pick,hold,place,slide,push,insert,press,carry,walk,stop,reach. Never torques, never metal, never MEASURED."},
            {"role": "user", "content": prompt}
        ]
    });
    let resp: Value = ureq::post(&format!("{GROK_BASE}/chat/completions"))
        .set("Authorization", &format!("Bearer {key}"))
        .set("Content-Type", "application/json")
        .send_json(body)
        .map_err(|e| AgentError::Http(e.to_string()))?
        .into_json()
        .map_err(|e| AgentError::Parse(e.to_string()))?;
    let content = resp
        .pointer("/choices/0/message/content")
        .and_then(Value::as_str)
        .ok_or_else(|| AgentError::Parse("no content".into()))?;
    let json = extract_json(content)?;
    admit_program(json)
}

#[cfg(feature = "live-grok")]
fn extract_json(s: &str) -> Result<Value, AgentError> {
    let t = s.trim();
    let t = t
        .strip_prefix("```json")
        .or_else(|| t.strip_prefix("```"))
        .unwrap_or(t)
        .trim_end_matches('`')
        .trim();
    serde_json::from_str(t).map_err(|e| AgentError::Parse(e.to_string()))
}

pub fn learned_source_is_not_executable(source: &str) -> bool {
    is_learned_source(source)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strips_metal_and_forbids_move_actuator() {
        let raw = serde_json::json!({
            "program_name": "x",
            "metal": true,
            "evidence_status": "MEASURED",
            "phases": [{"name": "a", "verb": "move_actuator"}]
        });
        assert!(admit_program(raw).is_err());
    }

    #[test]
    fn admits_hold_and_stamps_sim() {
        let raw = serde_json::json!({
            "program_name": "hold_still",
            "phases": [{"name": "h", "verb": "hold"}]
        });
        let p = admit_program(raw).unwrap();
        assert!(!p.metal);
        assert_eq!(p.evidence_status, EVIDENCE_STATUS);
        assert!(learned_source_is_not_executable("grok"));
    }

    #[test]
    fn offline_never_emits_torques() {
        let p = offline_propose("pick and place the cube");
        assert_eq!(p.phases[0].verb, "place");
        assert!(!p.learned_actuator_authority);
    }
}
