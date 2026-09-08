use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicalPlan {
    pub action: Vec<f64>,
    pub kind: String,
    #[serde(default)]
    pub rationale: String,
}

impl PhysicalPlan {
    pub fn new(kind: impl Into<String>, action: Vec<f64>) -> Self {
        Self {
            action,
            kind: kind.into(),
            rationale: String::new(),
        }
    }

    pub fn finite(&self) -> bool {
        self.action.iter().all(|x| x.is_finite())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Intent {
    pub text: String,
    pub verb: String,
    pub target_object: String,
    pub source: String,
    #[serde(default)]
    pub require_scene: bool,
}

impl Intent {
    pub fn language(text: impl Into<String>, verb: impl Into<String>) -> Self {
        let verb = verb.into();
        let require_scene = matches!(
            verb.as_str(),
            "precision_place" | "place" | "pick" | "grasp" | "insert"
        );
        Self {
            text: text.into(),
            verb,
            target_object: String::new(),
            source: "language".into(),
            require_scene,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PolicyProposal {
    pub action: Vec<f64>,
    pub source: String,
    #[serde(default)]
    pub policy_id: String,
}

impl PolicyProposal {
    pub fn is_learned(&self) -> bool {
        crate::authority::is_learned_source(&self.source)
    }
}
