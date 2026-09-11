use crate::transform::{Se3, TransformEdge, TransformError, TransformGraph};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InteractionFrameKind {
    Object,
    Grasp,
    PushContact,
    Approach,
    Surface,
}

impl InteractionFrameKind {
    pub fn frame_id(self, key: &str) -> String {
        match self {
            Self::Object => format!("object:{key}"),
            Self::Grasp => format!("grasp:{key}"),
            Self::PushContact => format!("push:{key}"),
            Self::Approach => format!("approach:{key}"),
            Self::Surface => format!("surface:{key}"),
        }
    }
}

pub fn insert_interaction_frame(
    graph: &mut TransformGraph,
    parent: &str,
    kind: InteractionFrameKind,
    key: &str,
    pose_in_parent: Se3,
    epoch: &str,
    now_s: f64,
    source: &str,
) -> Result<String, TransformError> {
    let child = kind.frame_id(key);
    graph.insert(TransformEdge::from_se3(
        parent,
        &child,
        pose_in_parent,
        epoch,
        now_s,
        source,
    ))?;
    Ok(child)
}

pub fn world_pose_of(
    graph: &TransformGraph,
    frame: &str,
    now_s: f64,
    freshness_s: f64,
) -> Result<Se3, TransformError> {
    graph.resolve("world", frame, now_s, Some(freshness_s))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_resolve_through_the_graph() {
        let mut g = TransformGraph::new("e0");
        let obj = Se3::try_new([0.4, 0.0, 0.1], [1.0, 0.0, 0.0, 0.0]).unwrap();
        let id = insert_interaction_frame(
            &mut g,
            "world",
            InteractionFrameKind::Object,
            "box",
            obj,
            "e0",
            1.0,
            "scenario",
        )
        .unwrap();
        let approach = Se3::try_new([0.0, 0.0, 0.08], [1.0, 0.0, 0.0, 0.0]).unwrap();
        let aid = insert_interaction_frame(
            &mut g,
            &id,
            InteractionFrameKind::Approach,
            "box",
            approach,
            "e0",
            1.0,
            "scenario",
        )
        .unwrap();
        let w = world_pose_of(&g, &aid, 1.0, 1.0).unwrap();
        assert!((w.xyz[2] - 0.18).abs() < 1e-9);
        assert!((w.xyz[0] - 0.4).abs() < 1e-9);
    }
}
