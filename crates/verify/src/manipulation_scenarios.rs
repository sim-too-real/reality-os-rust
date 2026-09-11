//! Deterministic mixed positive/ambiguous/negative manipulation scenarios.

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Polarity {
    Positive,
    Ambiguous,
    Negative,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NegKind {
    Unreachable,
    StaleObject,
    ObjectMoved,
    GripperUnavailable,
    UnsupportedCoupling,
    BlockedGrasp,
    EmptyClose,
    Slip,
    UnexpectedContact,
    ForceBoundUnavailable,
    ActuatorSaturation,
    Replay,
    RestartReplay,
    ControllerFailure,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ManipulationScenario {
    pub seed: u64,
    pub polarity: Polarity,
    pub skill: String,
    pub objects: Vec<Value>,
    pub planar: bool,
    pub object_id: String,
    pub expected_refusal: Option<String>,
    pub neg: Option<NegKind>,
    pub stale: bool,
    pub move_object_after_obs: bool,
    pub replay: bool,
    pub restart_replay: bool,
    pub crash_controller: bool,
    pub immovable: bool,
    pub push_dir: [f64; 3],
    pub push_dist: f64,
    pub required_opening: f64,
}

pub fn is_planar_model(joint_axes: &[[f64; 3]]) -> bool {
    !joint_axes.is_empty()
        && joint_axes.iter().all(|a| a[0].abs() < 0.2 && a[1].abs() < 0.2 && a[2].abs() > 0.8)
}

impl NegKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Unreachable => "UNREACHABLE",
            Self::StaleObject => "STALE_OBJECT",
            Self::ObjectMoved => "OBJECT_MOVED",
            Self::GripperUnavailable => "RESOURCE_UNSUPPORTED",
            Self::UnsupportedCoupling => "COUPLING_UNSUPPORTED",
            Self::BlockedGrasp => "BLOCKED_APPROACH",
            Self::EmptyClose => "GRIPPER_EMPTY_CLOSE",
            Self::Slip => "SLIP",
            Self::UnexpectedContact => "UNEXPECTED_CONTACT",
            Self::ForceBoundUnavailable => "FORCE_BOUND_UNAVAILABLE",
            Self::ActuatorSaturation => "ACTUATOR_SATURATION",
            Self::Replay => "REPLAY",
            Self::RestartReplay => "REPLAY",
            Self::ControllerFailure => "CONTROLLER_FAILURE",
        }
    }
}

pub fn release_scenario(seed: u64, n: usize) -> ManipulationScenario {
    let mut rng = StdRng::seed_from_u64(seed.wrapping_add(11));
    let opening = rng.gen_range(0.0..0.4);
    let stale = n % 10 == 0;
    let replay = n % 17 == 0;
    let restart = n % 19 == 0;
    let crash = n % 23 == 0;
    let polarity = if stale || crash {
        Polarity::Negative
    } else if replay || restart {
        Polarity::Ambiguous
    } else {
        Polarity::Positive
    };
    ManipulationScenario {
        seed,
        polarity,
        skill: "RELEASE".into(),
        objects: vec![],
        planar: false,
        object_id: "obj0".into(),
        expected_refusal: if stale {
            Some("STALE_GRIPPER_STATE".into())
        } else if crash {
            Some("CONTROLLER_FAILURE".into())
        } else {
            None
        },
        neg: if stale {
            Some(NegKind::StaleObject)
        } else if crash {
            Some(NegKind::ControllerFailure)
        } else if replay {
            Some(NegKind::Replay)
        } else if restart {
            Some(NegKind::RestartReplay)
        } else {
            None
        },
        stale,
        move_object_after_obs: false,
        replay,
        restart_replay: restart,
        crash_controller: crash,
        immovable: false,
        push_dir: [0.0, 0.0, 0.0],
        push_dist: 0.0,
        required_opening: 1.0 - opening * 0.1,
    }
}

pub fn grasp_scenario(seed: u64, planar: bool, idx: usize) -> ManipulationScenario {
    let mut rng = StdRng::seed_from_u64(seed.wrapping_add(idx as u64 * 17 + 3));
    let required = required_negative_kinds();
    let (polarity, neg) = if idx < required.len() {
        (Polarity::Negative, Some(required[idx]))
    } else {
        match idx % 5 {
            0 | 1 => (Polarity::Positive, None),
            2 => (
                Polarity::Ambiguous,
                if idx % 11 == 2 {
                    Some(NegKind::Slip)
                } else {
                    None
                },
            ),
            _ => (Polarity::Negative, Some(required[idx % required.len()])),
        }
    };
    let geom = match idx % 3 {
        0 => "box",
        1 => "cylinder",
        _ => "box",
    };
    let size = if planar {
        rng.gen_range(0.010..0.016)
    } else {
        rng.gen_range(0.018..0.032)
    };
    let mass = rng.gen_range(0.02..0.25);
    let friction = if matches!(neg, Some(NegKind::Slip)) {
        0.02
    } else {
        rng.gen_range(0.3..1.2)
    };
    let (x, y, z) = if planar {
        (
            rng.gen_range(0.20..0.30),
            rng.gen_range(-0.03..0.03),
            0.12 + size,
        )
    } else {
        (
            rng.gen_range(0.38..0.52),
            rng.gen_range(-0.12..0.12),
            0.42 + size,
        )
    };
    if matches!(neg, Some(NegKind::Unreachable)) {
        let (x, y, z) = if planar {
            (1.6, 0.0, 0.05)
        } else {
            (1.8, 0.0, 0.2)
        };
        return scenario_with_object(
            seed,
            polarity,
            "GRASP",
            planar,
            geom,
            [x, y, z],
            size,
            mass,
            friction,
            neg,
            idx,
        );
    }
    let mut objects = vec![object_json("obj0", geom, [x, y, z], size, mass, friction, true)];
    if planar {
        objects.insert(
            0,
            json!({"name":"table","type":"box","pos":[0.24,0.0,0.105],"size":[0.40,0.22,0.01],"mass":10.0,"movable":false,"rgba":[0.4,0.4,0.4,1]}),
        );
    } else {
        objects.insert(
            0,
            json!({"name":"table","type":"box","pos":[0.45,0.0,0.40],"size":[0.25,0.25,0.02],"mass":20.0,"movable":false,"rgba":[0.45,0.4,0.35,1]}),
        );
    }
    if matches!(neg, Some(NegKind::UnexpectedContact) | Some(NegKind::BlockedGrasp)) {
        objects.push(json!({
            "name":"obstacle",
            "type":"box",
            "pos": if planar { json!([x - 0.04, y, z + 0.04]) } else { json!([x, y, z + 0.08]) },
            "size":[0.03,0.03,0.03],
            "mass":1.0,
            "movable":false,
            "rgba":[0.1,0.1,0.1,1]
        }));
    }
    let empty = matches!(neg, Some(NegKind::EmptyClose));
    if empty {
        objects.retain(|o| o["name"] != "obj0");
        objects.push(object_json(
            "obj0",
            geom,
            if planar {
                [0.55, 0.2, 0.03]
            } else {
                [0.85, 0.35, 0.45]
            },
            size,
            mass,
            friction,
            true,
        ));
    }
    ManipulationScenario {
        seed,
        polarity,
        skill: "GRASP".into(),
        objects,
        planar,
        object_id: "obj0".into(),
        expected_refusal: expected_of(neg),
        neg,
        stale: matches!(neg, Some(NegKind::StaleObject)),
        move_object_after_obs: matches!(neg, Some(NegKind::ObjectMoved)),
        replay: matches!(neg, Some(NegKind::Replay)),
        restart_replay: matches!(neg, Some(NegKind::RestartReplay)),
        crash_controller: matches!(neg, Some(NegKind::ControllerFailure)),
        immovable: false,
        push_dir: [0.0, 0.0, 0.0],
        push_dist: 0.0,
        required_opening: if empty { 0.2 } else { 0.85 },
    }
}

pub fn push_scenario(seed: u64, planar: bool, idx: usize) -> ManipulationScenario {
    let mut rng = StdRng::seed_from_u64(seed.wrapping_add(idx as u64 * 31 + 9));
    let push_negs = [
        NegKind::Unreachable,
        NegKind::StaleObject,
        NegKind::ObjectMoved,
        NegKind::UnexpectedContact,
        NegKind::Slip,
        NegKind::Replay,
        NegKind::RestartReplay,
        NegKind::ControllerFailure,
        NegKind::ForceBoundUnavailable,
    ];
    let (polarity, neg) = if idx < push_negs.len() {
        (Polarity::Negative, Some(push_negs[idx]))
    } else {
        match idx % 5 {
            0 | 1 => (Polarity::Positive, None),
            2 => (Polarity::Ambiguous, None),
            _ => (Polarity::Negative, Some(push_negs[idx % push_negs.len()])),
        }
    };
    let mass = if idx % 21 == 0 {
        80.0
    } else {
        rng.gen_range(0.04..0.4)
    };
    let immovable = mass > 20.0;
    let friction = if matches!(neg, Some(NegKind::Slip)) {
        0.01
    } else {
        rng.gen_range(0.2..1.0)
    };
    let size = rng.gen_range(0.02..0.04);
    let (x, y, z) = if planar {
        (rng.gen_range(0.20..0.30), rng.gen_range(-0.03..0.03), 0.12 + size)
    } else {
        (rng.gen_range(0.40..0.55), rng.gen_range(-0.10..0.10), 0.42 + size)
    };
    let dir = if planar {
        [1.0, 0.0, 0.0]
    } else {
        [1.0, rng.gen_range(-0.2..0.2), 0.0]
    };
    let mut s = scenario_with_object(
        seed,
        polarity,
        "PUSH",
        planar,
        "box",
        [x, y, z],
        size,
        mass,
        friction,
        neg,
        idx,
    );
    s.immovable = immovable;
    s.push_dir = dir;
    s.push_dist = rng.gen_range(0.04..0.10);
    if matches!(neg, Some(NegKind::UnexpectedContact)) {
        s.objects.push(json!({
            "name":"obstacle",
            "type":"box",
            "pos": if planar { json!([x + 0.08, y, z]) } else { json!([x + 0.08, y, z]) },
            "size":[0.03,0.03,0.03],
            "mass":2.0,
            "movable":false
        }));
    }
    s
}

#[allow(clippy::too_many_arguments)]
fn scenario_with_object(
    seed: u64,
    polarity: Polarity,
    skill: &str,
    planar: bool,
    geom: &str,
    pos: [f64; 3],
    size: f64,
    mass: f64,
    friction: f64,
    neg: Option<NegKind>,
    _idx: usize,
) -> ManipulationScenario {
    let mut objects = vec![object_json("obj0", geom, pos, size, mass, friction, mass < 20.0)];
    objects.insert(
        0,
        if planar {
            json!({"name":"table","type":"box","pos":[0.24,0.0,0.105],"size":[0.40,0.22,0.01],"mass":10.0,"movable":false})
        } else {
            json!({"name":"table","type":"box","pos":[0.45,0.0,0.40],"size":[0.25,0.25,0.02],"mass":20.0,"movable":false})
        },
    );
    ManipulationScenario {
        seed,
        polarity,
        skill: skill.into(),
        objects,
        planar,
        object_id: "obj0".into(),
        expected_refusal: expected_of(neg),
        neg,
        stale: matches!(neg, Some(NegKind::StaleObject)),
        move_object_after_obs: matches!(neg, Some(NegKind::ObjectMoved)),
        replay: matches!(neg, Some(NegKind::Replay)),
        restart_replay: matches!(neg, Some(NegKind::RestartReplay)),
        crash_controller: matches!(neg, Some(NegKind::ControllerFailure)),
        immovable: mass > 20.0,
        push_dir: [1.0, 0.0, 0.0],
        push_dist: 0.06,
        required_opening: 0.8,
    }
}

fn expected_of(neg: Option<NegKind>) -> Option<String> {
    neg.map(|n| n.as_str().to_string())
}

fn object_json(
    name: &str,
    geom: &str,
    pos: [f64; 3],
    size: f64,
    mass: f64,
    friction: f64,
    movable: bool,
) -> Value {
    json!({
        "name": name,
        "type": geom,
        "pos": pos,
        "size": if geom == "cylinder" { json!([size * 0.7, size]) } else { json!([size, size, size]) },
        "mass": mass,
        "friction": friction,
        "movable": movable,
        "rgba": [0.85, 0.25, 0.2, 1.0]
    })
}

pub fn required_negative_kinds() -> &'static [NegKind] {
    &[
        NegKind::Unreachable,
        NegKind::StaleObject,
        NegKind::ObjectMoved,
        NegKind::GripperUnavailable,
        NegKind::UnsupportedCoupling,
        NegKind::BlockedGrasp,
        NegKind::EmptyClose,
        NegKind::Slip,
        NegKind::UnexpectedContact,
        NegKind::ForceBoundUnavailable,
        NegKind::ActuatorSaturation,
        NegKind::Replay,
        NegKind::RestartReplay,
        NegKind::ControllerFailure,
    ]
}
