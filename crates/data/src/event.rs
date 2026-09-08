use realityos_kernel::{Layer, Violation};
use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventKind {
    SensorIngest,
    Decide,
    Gate,
    Write,
    Refuse,
    Estop,
    Recover,
    SessionStart,
    NamedHole,
    PhysicsScreen,
}

impl EventKind {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SensorIngest => "sensor_ingest",
            Self::Decide => "decide",
            Self::Gate => "gate",
            Self::Write => "write",
            Self::Refuse => "refuse",
            Self::Estop => "estop",
            Self::Recover => "recover",
            Self::SessionStart => "session_start",
            Self::NamedHole => "named_hole",
            Self::PhysicsScreen => "physics_screen",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebugEvent {
    pub seq: u64,
    pub t_s: f64,
    pub correlation_id: String,
    pub layer: Layer,
    pub kind: EventKind,
    pub ok: bool,
    #[serde(default)]
    pub violations: Vec<Violation>,
    #[serde(default)]
    pub payload: Value,
    pub metal: bool,
}

impl DebugEvent {
    pub fn new(
        layer: Layer,
        kind: EventKind,
        ok: bool,
        t_s: f64,
        correlation_id: impl Into<String>,
    ) -> Self {
        Self {
            seq: 0,
            t_s,
            correlation_id: correlation_id.into(),
            layer,
            kind,
            ok,
            violations: Vec::new(),
            payload: Value::Null,
            metal: false,
        }
    }

    pub fn with_payload(mut self, payload: Value) -> Self {
        self.payload = payload;
        self
    }

    pub fn with_violations(mut self, v: Vec<Violation>) -> Self {
        self.violations = v;
        self
    }
}

#[derive(Debug, Clone, Default)]
pub struct EventFilter {
    pub layer: Option<Layer>,
    pub kind: Option<EventKind>,
    pub correlation_id: Option<String>,
    pub ok: Option<bool>,
}

impl EventFilter {
    pub fn matches(&self, e: &DebugEvent) -> bool {
        if self.layer.is_some_and(|l| l != e.layer) {
            return false;
        }
        if self.kind.is_some_and(|k| k != e.kind) {
            return false;
        }
        if let Some(id) = &self.correlation_id {
            if id != &e.correlation_id {
                return false;
            }
        }
        if self.ok.is_some_and(|ok| ok != e.ok) {
            return false;
        }
        true
    }
}
