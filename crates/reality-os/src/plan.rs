use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicalPlan {
    pub action: Vec<f64>,
    pub kind: String,
    #[serde(default)]
    pub rationale: String,
    #[serde(default)]
    pub frame_id: String,
    #[serde(default)]
    pub units: String,
    #[serde(default)]
    pub mode: String,
    #[serde(default)]
    pub duration_s: Option<f64>,
    #[serde(default)]
    pub provenance: String,
    #[serde(default)]
    pub completion: String,
}

impl PhysicalPlan {
    pub fn new(kind: impl Into<String>, action: Vec<f64>) -> Self {
        Self {
            action,
            kind: kind.into(),
            rationale: String::new(),
            frame_id: String::new(),
            units: String::new(),
            mode: String::new(),
            duration_s: None,
            provenance: String::new(),
            completion: String::new(),
        }
    }

    pub fn hold(kind: impl Into<String>, dof: usize) -> Self {
        let mut p = Self::new(kind, vec![0.0; dof.max(1)]);
        p.mode = "hold".into();
        p.units = "effort".into();
        p.provenance = "explicit_hold".into();
        p.completion = "hold_zero_effort".into();
        p
    }

    pub fn finite(&self) -> bool {
        self.action.iter().all(|x| x.is_finite())
            && self
                .duration_s
                .map(|d| d.is_finite() && d >= 0.0)
                .unwrap_or(true)
    }

    pub fn lacks_explicit_target(&self) -> bool {
        self.action.is_empty()
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
