//! Reusable embodiment-agnostic scenario families.

use crate::observation::VisionMode;
use crate::scenario::{Dist, EnvelopeSpec, Region, ScenarioSpec};
use crate::task::TaskSpec;
use serde_json::json;

pub const MILESTONE_FAMILIES: &[&str] = &[
    "JOINT_TRACKING",
    "REACH_TARGET",
    "KEEP_OUT_ZONE",
    "ACTUATOR_SATURATION",
    "EXTERNAL_PUSH",
    "COMMAND_REPLAY",
    "DUPLICATE_COMMAND",
    "WRONG_ROBOT_IDENTITY",
    "WRONG_TASK_AUTHORITY",
    "POLICY_CRASH",
];

pub const ALL_FAMILIES: &[&str] = &[
    "JOINT_TRACKING",
    "REACH_TARGET",
    "PICK_OBJECT",
    "PLACE_OBJECT",
    "PUSH_OBJECT",
    "CONTACT_TARGET",
    "AVOID_OBSTACLE",
    "KEEP_OUT_ZONE",
    "PAYLOAD_CHANGE",
    "FRICTION_CHANGE",
    "OBJECT_MOVED_AFTER_OBSERVATION",
    "SENSOR_DELAY",
    "SENSOR_DROPOUT",
    "STALE_OBSERVATION",
    "ACTUATOR_SATURATION",
    "EXTERNAL_PUSH",
    "UNEXPECTED_CONTACT",
    "COMMAND_REPLAY",
    "DUPLICATE_COMMAND",
    "WRONG_ROBOT_IDENTITY",
    "WRONG_TASK_AUTHORITY",
    "POLICY_CRASH",
    "AUTHORITY_RESTART",
];

pub fn family_spec(family: &str, nu: usize, ee: &str) -> ScenarioSpec {
    let mut p = std::collections::BTreeMap::new();
    let mut envelope = EnvelopeSpec::default();
    let mut task = TaskSpec::Hold { duration_s: 0.2 };
    let mut objects = Vec::new();
    let mut spec = ScenarioSpec {
        id: family.to_ascii_lowercase(),
        family: family.into(),
        scene: "default".into(),
        task: task.clone(),
        parameters: p.clone(),
        objects: objects.clone(),
        envelope: envelope.clone(),
        vision_mode: VisionMode::State,
        control_hz: 50.0,
        duration_s: 0.40,
        sensor_delay_s: 0.0,
        sensor_dropout: false,
        stale_observation: false,
        replay_command: false,
        duplicate_command: false,
        wrong_robot: false,
        wrong_task_authority: false,
        policy_crash: false,
        authority_restart: false,
        external_push: None,
        push_body: None,
    };
    match family {
        "JOINT_TRACKING" => {
            for i in 0..nu.clamp(1, 6) {
                p.insert(
                    format!("q{i}"),
                    Dist::Uniform {
                        low: -0.15,
                        high: 0.15,
                    },
                );
            }
            task = TaskSpec::JointTrack {
                target: vec![0.0; nu.max(1)],
                tolerance: 0.35,
            };
        }
        "REACH_TARGET" => {
            p.insert(
                "target.x".into(),
                Dist::Uniform {
                    low: 0.15,
                    high: 0.35,
                },
            );
            p.insert(
                "target.y".into(),
                Dist::Uniform {
                    low: -0.1,
                    high: 0.1,
                },
            );
            p.insert("target.z".into(), Dist::Constant { value: 0.15 });
            spec.duration_s = 0.50;
            task = TaskSpec::Reach {
                end_effector: ee.into(),
                target: [0.25, 0.0, 0.15],
                radius: 0.20,
            };
        }
        "KEEP_OUT_ZONE" | "AVOID_OBSTACLE" => {
            p.insert(
                "zone.x".into(),
                Dist::Uniform {
                    low: 0.2,
                    high: 0.45,
                },
            );
            envelope.keep_out.push(Region {
                name: "keep_out".into(),
                center: [0.35, 0.0, 0.1],
                half: [0.04, 0.04, 0.04],
            });
            task = TaskSpec::KeepOut {
                name: "keep_out".into(),
            };
        }
        "PICK_OBJECT" => {
            objects.push(json!({"name":"cube_1","type":"box","size":[0.03,0.03,0.03],"pos":[0.25,0.0,0.03],"mass":0.05}));
            p.insert(
                "cube_1.x".into(),
                Dist::Uniform {
                    low: 0.2,
                    high: 0.3,
                },
            );
            task = TaskSpec::Pick {
                object: "cube_1".into(),
            };
        }
        "PLACE_OBJECT" => {
            objects.push(json!({"name":"cube_1","type":"box","size":[0.03,0.03,0.03],"pos":[0.25,0.0,0.03],"mass":0.05}));
            task = TaskSpec::Place {
                object: "cube_1".into(),
                target_region: "tray_A".into(),
            };
        }
        "PUSH_OBJECT" | "CONTACT_TARGET" | "UNEXPECTED_CONTACT" => {
            objects.push(json!({"name":"cube_1","type":"box","size":[0.04,0.04,0.04],"pos":[0.28,0.0,0.04],"mass":0.08}));
            if family == "UNEXPECTED_CONTACT" {
                envelope
                    .forbidden_contact_pairs
                    .push(("link2".into(), "cube_1".into()));
            }
            task = TaskSpec::Hold { duration_s: 0.2 };
        }
        "PAYLOAD_CHANGE" => {
            objects.push(json!({"name":"payload","type":"box","size":[0.02,0.02,0.02],"pos":[0.0,0.0,0.4],"mass":0.2,"movable":false}));
        }
        "FRICTION_CHANGE" => {
            objects.push(json!({"name":"cube_1","type":"box","size":[0.03,0.03,0.03],"pos":[0.2,0.0,0.03],"friction":0.4}));
            p.insert(
                "cube_1.friction".into(),
                Dist::Uniform {
                    low: 0.05,
                    high: 0.8,
                },
            );
        }
        "OBJECT_MOVED_AFTER_OBSERVATION" => {
            objects.push(
                json!({"name":"cube_1","type":"box","size":[0.03,0.03,0.03],"pos":[0.22,0.0,0.03]}),
            );
            spec.vision_mode = VisionMode::PerfectPerception;
        }
        "SENSOR_DELAY" => {
            spec.sensor_delay_s = 0.05;
            p.insert(
                "camera_latency_s".into(),
                Dist::Uniform {
                    low: 0.02,
                    high: 0.18,
                },
            );
        }
        "SENSOR_DROPOUT" => spec.sensor_dropout = true,
        "STALE_OBSERVATION" => spec.stale_observation = true,
        "ACTUATOR_SATURATION" => {
            task = TaskSpec::JointTrack {
                target: vec![0.4; nu.max(1)],
                tolerance: 0.8,
            };
        }
        "EXTERNAL_PUSH" => {
            spec.external_push = Some([2.0, 0.0, 0.0]);
            spec.push_body = Some("link1".into());
            task = TaskSpec::Hold { duration_s: 0.2 };
        }
        "COMMAND_REPLAY" => spec.replay_command = true,
        "DUPLICATE_COMMAND" => spec.duplicate_command = true,
        "WRONG_ROBOT_IDENTITY" => spec.wrong_robot = true,
        "WRONG_TASK_AUTHORITY" => {
            spec.wrong_task_authority = true;
            task = TaskSpec::AuthorityNegative {
                expect: "place_without_scene".into(),
            };
        }
        "POLICY_CRASH" => spec.policy_crash = true,
        "AUTHORITY_RESTART" => spec.authority_restart = true,
        _ => {}
    }
    spec.parameters = p;
    spec.objects = objects;
    spec.envelope = envelope;
    spec.task = task;
    spec
}
