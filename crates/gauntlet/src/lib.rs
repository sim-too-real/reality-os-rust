//! Parameterized safety/control gauntlets. One function = many real scenarios.

use realityos_core::{
    Certificate, CertifiedCommand, DecideRequest, Intent, RealityOs, SkillIR, WorldView,
};
use realityos_embodiment::{catalog as robots, EmbodimentGraph, Environment};
use realityos_governor::SafeState;
use realityos_kernel::DecisionStatus;
use realityos_plant::{ActionParams, SimPlant};
use realityos_rate::{certify_tick_with_bus, run_dispose_ticks, TelemetryFrame};
use realityos_session::{RuntimeSession, StartArgs};
use realityos_vision::{compile_observation, see_from_pixels, Camera, Frame};

#[derive(Debug, Clone)]
pub struct CaseResult {
    pub name: String,
    pub ok: bool,
    pub expected_block: bool,
    pub blocked: bool,
    pub detail: String,
}

impl CaseResult {
    pub fn pass(&self) -> bool {
        self.blocked == self.expected_block
    }
}

#[derive(Clone, Copy)]
pub enum Fault {
    Happy,
    Estop,
    StaleSensor,
    Replay,
    Unacked,
    RefuseCert,
    Envelope,
    ForeignHash,
    Hold,
    GiftedPlace,
    Expired,
    NanAction,
    DimMismatch,
    ClockRollback,
    Overload,
    SequenceReplayGap,
    UnknownVerb,
}

impl Fault {
    pub const ALL: [Fault; 17] = [
        Fault::Happy,
        Fault::Estop,
        Fault::StaleSensor,
        Fault::Replay,
        Fault::Unacked,
        Fault::RefuseCert,
        Fault::Envelope,
        Fault::ForeignHash,
        Fault::Hold,
        Fault::GiftedPlace,
        Fault::Expired,
        Fault::NanAction,
        Fault::DimMismatch,
        Fault::ClockRollback,
        Fault::Overload,
        Fault::SequenceReplayGap,
        Fault::UnknownVerb,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            Fault::Happy => "happy",
            Fault::Estop => "estop",
            Fault::StaleSensor => "stale_sensor",
            Fault::Replay => "replay",
            Fault::Unacked => "unacked",
            Fault::RefuseCert => "refuse_cert",
            Fault::Envelope => "envelope",
            Fault::ForeignHash => "foreign_hash",
            Fault::Hold => "hold",
            Fault::GiftedPlace => "gifted_place",
            Fault::Expired => "expired",
            Fault::NanAction => "nan_action",
            Fault::DimMismatch => "dim_mismatch",
            Fault::ClockRollback => "clock_rollback",
            Fault::Overload => "overload",
            Fault::SequenceReplayGap => "sequence_replay",
            Fault::UnknownVerb => "unknown_verb",
        }
    }

    pub fn expect_block(self) -> bool {
        !matches!(self, Fault::Happy)
    }
}

fn plant_for(robot: &EmbodimentGraph) -> SimPlant {
    let peak = robot.tau_max().into_iter().fold(1.0_f64, f64::max);
    SimPlant::new(&robot.id, robot.dof().max(1), peak)
}

fn allow_cmd(
    id: &str,
    seq: i64,
    now: f64,
    action: Vec<f64>,
    release: &str,
    cal: &str,
    ack: bool,
) -> CertifiedCommand {
    let c = CertifiedCommand::issue(
        id,
        seq,
        now,
        30.0,
        Certificate::new(DecisionStatus::Allow, "gauntlet"),
        action,
    )
    .unwrap()
    .with_identity(release, "", cal);
    if ack {
        c.acknowledge()
    } else {
        c
    }
}

fn session(robot: &EmbodimentGraph, now: f64) -> RuntimeSession<SimPlant> {
    let plant = plant_for(robot);
    let mut args = StartArgs::simulation(format!("rel-{}", robot.id));
    args.max_action_abs = robot.tau_max().into_iter().fold(1.0, f64::max);
    let mut s = RuntimeSession::start(args, plant, now).unwrap();
    s.governor.mark_sensor(now, Some("hash".into()));
    s
}

fn world_for(robot: &EmbodimentGraph, env: &Environment) -> WorldView {
    WorldView {
        tau_max: robot.tau_max(),
        q: vec![0.0; robot.dof()],
        q_min: robot.q_min(),
        q_max: robot.q_max(),
        observation: None,
        speed_m_s: Some(0.1),
        decel_m_s2: Some(env.g_m_s2.max(0.1)),
        max_stop_m: Some(10.0),
        mass_kg: Some(5.0),
        ke_limit_j: Some(1e6),
        kt_nm_per_a: Some(0.1),
        current_a: Some(1.0),
        gear_ratio: Some(1.0),
        motor_eta: Some(0.9),
        body_region: Some(realityos_core::domains::BodyRegion::Hand),
        contact_force_n: Some(10.0),
        capabilities: robot
            .capabilities
            .items
            .iter()
            .map(|c| c.as_str().to_string())
            .collect(),
        mu: env.mu,
        g_m_s2: Some(env.g_m_s2),
        ..WorldView::default()
    }
}

/// Governor safety matrix: robots × environments × faults.
pub fn governor_matrix() -> Vec<CaseResult> {
    let mut out = Vec::new();
    let now = 100.0;
    for robot in robots() {
        for env in Environment::catalog() {
            for fault in Fault::ALL {
                let name = format!("gov|{}|{}|{}", robot.id, env.id, fault.as_str());
                let mut sess = session(&robot, now);
                sess.governor.envelope_mut().unwrap().max_action_abs =
                    robot.tau_max().into_iter().fold(1.0, f64::max);
                let n = robot.dof().max(1);
                let mut action = vec![0.05; n];
                let mut cmd = allow_cmd(
                    "g-1",
                    1,
                    now,
                    action.clone(),
                    sess.governor.identity.release_hash.as_str(),
                    sess.governor.identity.calibration_id_str(),
                    true,
                );
                let blocked = match fault {
                    Fault::Happy => {
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::Estop => {
                        sess.governor.engage_estop("gauntlet", now);
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::StaleSensor => {
                        sess.governor.config_mut().require_sensor_before_write = true;
                        sess.governor.config_mut().sensor_stale_s = 0.01;
                        sess.governor.mark_sensor(now - 10.0, Some("old".into()));
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::Replay => {
                        let _ = sess.bind_and_dispatch(cmd.clone(), &ActionParams::empty(), now);
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::Unacked => {
                        cmd = allow_cmd(
                            "g-1",
                            1,
                            now,
                            action.clone(),
                            sess.governor.identity.release_hash.as_str(),
                            sess.governor.identity.calibration_id_str(),
                            false,
                        );
                        !sess
                            .governor
                            .write_driver(&cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::RefuseCert => {
                        cmd = CertifiedCommand::issue(
                            "g-1",
                            1,
                            now,
                            30.0,
                            Certificate::new(DecisionStatus::Refuse, "gauntlet"),
                            action.clone(),
                        )
                        .unwrap()
                        .with_identity(
                            sess.governor.identity.release_hash.as_str(),
                            "",
                            sess.governor.identity.calibration_id_str(),
                        )
                        .acknowledge();
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::Envelope => {
                        action = vec![1e6; n];
                        cmd = allow_cmd(
                            "g-1",
                            1,
                            now,
                            action,
                            sess.governor.identity.release_hash.as_str(),
                            sess.governor.identity.calibration_id_str(),
                            true,
                        );
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::ForeignHash => {
                        cmd = allow_cmd(
                            "g-1",
                            1,
                            now,
                            action,
                            "foreign-rel",
                            sess.governor.identity.calibration_id_str(),
                            true,
                        );
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::Hold => {
                        sess.latch_safe_state(SafeState::Hold, "gauntlet");
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::GiftedPlace => {
                        let mut ros = RealityOs::new();
                        let d = ros.decide(DecideRequest::new(
                            Intent::language("place", "place"),
                            WorldView::default(),
                            now,
                        ));
                        !d.allowed
                    }
                    Fault::Expired => {
                        cmd = CertifiedCommand::issue(
                            "g-exp",
                            1,
                            now - 40.0,
                            1.0,
                            Certificate::new(DecisionStatus::Allow, "gauntlet"),
                            vec![0.05; n],
                        )
                        .unwrap()
                        .with_identity(
                            sess.governor.identity.release_hash.as_str(),
                            "",
                            sess.governor.identity.calibration_id_str(),
                        )
                        .acknowledge();
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::NanAction => CertifiedCommand::issue(
                        "g-nan",
                        1,
                        now,
                        30.0,
                        Certificate::new(DecisionStatus::Allow, "gauntlet"),
                        vec![f64::NAN; n],
                    )
                    .is_err(),
                    Fault::DimMismatch => {
                        cmd = allow_cmd(
                            "g-dim",
                            1,
                            now,
                            vec![0.05; n + 3],
                            sess.governor.identity.release_hash.as_str(),
                            sess.governor.identity.calibration_id_str(),
                            true,
                        );
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::ClockRollback => sess
                        .ingest_sensor(&[("j".into(), 0.1)], Some(now - 1.0), now)
                        .and_then(|_| {
                            sess.ingest_sensor(&[("j".into(), 0.2)], Some(now - 5.0), now)
                        })
                        .is_err(),
                    Fault::Overload => {
                        for i in 0..40 {
                            let _ = sess.push_outbound(TelemetryFrame::new(
                                "dbg",
                                now,
                                i.to_string(),
                            ));
                        }
                        !sess
                            .bind_and_dispatch(cmd, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::SequenceReplayGap => {
                        let first = allow_cmd(
                            "g-gap-1",
                            1,
                            now,
                            vec![0.05; n],
                            sess.governor.identity.release_hash.as_str(),
                            sess.governor.identity.calibration_id_str(),
                            true,
                        );
                        let _ = sess.bind_and_dispatch(first, &ActionParams::empty(), now);
                        let again = allow_cmd(
                            "g-gap-1",
                            5,
                            now,
                            vec![0.05; n],
                            sess.governor.identity.release_hash.as_str(),
                            sess.governor.identity.calibration_id_str(),
                            true,
                        );
                        !sess
                            .bind_and_dispatch(again, &ActionParams::empty(), now)
                            .ok
                    }
                    Fault::UnknownVerb => {
                        let mut ros = RealityOs::new();
                        let d = ros.decide(DecideRequest::new(
                            Intent::language("dance", "dance"),
                            world_for(&robot, &env),
                            now,
                        ));
                        !d.allowed
                    }
                };
                out.push(CaseResult {
                    name,
                    ok: blocked == fault.expect_block(),
                    expected_block: fault.expect_block(),
                    blocked,
                    detail: env.id.clone(),
                });
            }
        }
    }
    out
}

fn expect_ros_block(verb: &str, robot: &EmbodimentGraph, env: &Environment, world: &WorldView) -> bool {
    if let Some(skill) = SkillIR::admit(verb) {
        for cap in &skill.required_capabilities {
            if !world.capabilities.iter().any(|c| c == cap) {
                return true;
            }
        }
        if skill
            .preconditions
            .iter()
            .any(|p| p == "observation_fresh")
            && world.observation.is_none()
        {
            return true;
        }
        if verb == "walk" {
            return true;
        }
        // Catalog verb with no domain planner cannot invent a target.
        if !matches!(
            verb,
            "hold"
                | "reach"
                | "place"
                | "pick"
                | "grasp"
                | "insert"
                | "stop"
                | "walk"
                | "push"
        ) {
            return true;
        }
    }
    if verb == "stop" && env.id == "ice" && world.speed_m_s.unwrap_or(0.0) > 4.0 {
        return true;
    }
    if verb == "dance" {
        return true;
    }
    let _ = robot;
    false
}

/// Reality OS layer matrix: robots × env × skills/domains × vision variants.
pub fn reality_os_matrix() -> Vec<CaseResult> {
    let mut out = Vec::new();
    let now = 1.0;
    let mut verbs: Vec<String> = SkillIR::catalog().into_iter().map(|s| s.verb).collect();
    for extra in ["pfl", "motor", "energy", "dance"] {
        verbs.push(extra.into());
    }
    for robot in robots() {
        for env in Environment::catalog() {
            for verb in &verbs {
                let mut world = world_for(&robot, &env);
                if verb == "pfl" {
                    world.contact_force_n = Some(10.0);
                }
                if verb == "stop" && env.id == "ice" {
                    world.speed_m_s = Some(8.0);
                    world.max_stop_m = Some(1.0);
                }
                let mut ros = RealityOs::new();
                let d = ros.decide(DecideRequest::new(
                    Intent::language(verb, verb.as_str()),
                    world.clone(),
                    now,
                ));
                let expect_block = expect_ros_block(verb, &robot, &env, &world);
                let blocked = !d.allowed;
                out.push(CaseResult {
                    name: format!("ros|{}|{}|{verb}", robot.id, env.id),
                    ok: blocked == expect_block,
                    expected_block: expect_block,
                    blocked,
                    detail: format!("{} {}", d.status, d.physical_reason),
                });
            }
            // Place with compiled evidence on bodies that declare serial_arm.
            if robot.requires(realityos_kernel::Capability::SerialArm) {
                let frame = Frame::with_blob(32, 32, 16, 16, 3);
                let cam = Camera::default_workcell(32, 32);
                let ev = compile_observation(&frame, &cam, "cam0", now, 5.0).unwrap();
                let mut world = world_for(&robot, &env);
                world.observation = Some(ev);
                world.pose_std_m = Some(0.01);
                let mut ros = RealityOs::new();
                let d = ros.decide(DecideRequest::new(
                    Intent::language("place", "place"),
                    world,
                    now,
                ));
                out.push(CaseResult {
                    name: format!("ros|{}|{}|place_seen", robot.id, env.id),
                    ok: d.allowed,
                    expected_block: false,
                    blocked: !d.allowed,
                    detail: format!("{} {}", d.status, d.physical_reason),
                });
            }
        }
    }
    out.extend(vision_matrix());
    out
}

pub fn vision_matrix() -> Vec<CaseResult> {
    let mut out = Vec::new();
    let now = 1.0;
    let cases: Vec<(&str, Frame, Option<Camera>, bool)> = vec![
        (
            "blob",
            Frame::with_blob(32, 32, 16, 16, 3),
            Some(Camera::default_workcell(32, 32)),
            true,
        ),
        (
            "black",
            Frame::black(32, 32),
            Some(Camera::default_workcell(32, 32)),
            false,
        ),
        ("no_cam", Frame::with_blob(16, 16, 8, 8, 2), None, false),
        (
            "tiny",
            Frame::with_blob(64, 64, 32, 32, 1),
            Some(Camera::default_workcell(64, 64)),
            false,
        ),
        (
            "edge",
            Frame::with_blob(32, 32, 1, 1, 2),
            Some(Camera::default_workcell(32, 32)),
            true,
        ),
        (
            "bad_focal",
            Frame::with_blob(16, 16, 8, 8, 2),
            Some(Camera {
                fx: 0.0,
                fy: 16.0,
                cx: 8.0,
                cy: 8.0,
                table_z_m: 0.75,
            }),
            false,
        ),
        (
            "nan_cx",
            Frame::with_blob(16, 16, 8, 8, 2),
            Some(Camera {
                fx: 16.0,
                fy: 16.0,
                cx: f64::NAN,
                cy: 8.0,
                table_z_m: 0.75,
            }),
            false,
        ),
    ];
    for (label, frame, cam, expect_ok) in cases {
        let s = see_from_pixels(&frame, cam.as_ref());
        out.push(CaseResult {
            name: format!("vision|{label}"),
            ok: s.ok == expect_ok,
            expected_block: !expect_ok,
            blocked: !s.ok,
            detail: s.reason,
        });
        if let Some(c) = cam {
            let compiled = compile_observation(&frame, &c, "cam0", now, 4.0);
            let cam_ok = c.validate(frame.width, frame.height).is_ok();
            out.push(CaseResult {
                name: format!("vision|{label}|compile"),
                ok: compiled.is_ok() == cam_ok,
                expected_block: !cam_ok,
                blocked: compiled.is_err(),
                detail: format!("cam_ok={cam_ok}"),
            });
        }
    }
    out
}

pub fn rate_matrix() -> Vec<CaseResult> {
    let mut out = Vec::new();
    let env = realityos_core::BoundedTrustEnvelope {
        tau_max: vec![1.0],
        dq_max: vec![10.0],
        is_valid: true,
    };
    for band in realityos_rate::bands() {
        for robot in robots() {
            for (label, work_us, overload, expect_miss_or_hold) in [
                ("budget_ok", 1.0, false, false),
                ("deadline_miss", 500.0, false, true),
                ("overload_hold", 1.0, true, true),
            ] {
                let u = vec![0.01; robot.dof().max(1)];
                let ticks = run_dispose_ticks(
                    4,
                    band.hz,
                    &u[..1.min(u.len())],
                    &[0.0],
                    &env,
                    false,
                    work_us,
                );
                let miss = ticks.iter().any(|t| t.deadline_miss);
                let hold = certify_tick_with_bus(
                    &u[..1.min(u.len())],
                    &[0.0],
                    &env,
                    false,
                    true,
                    overload,
                );
                let blocked = miss || hold.mode == realityos_core::ExecutionMode::PassiveFallback && overload;
                let _ = expect_miss_or_hold;
                let expected = work_us > 20.0 || overload;
                out.push(CaseResult {
                    name: format!("rate|{}|{}|{label}", robot.id, band.name),
                    ok: blocked == expected,
                    expected_block: expected,
                    blocked,
                    detail: ticks
                        .first()
                        .map(realityos_rate::debug_tick)
                        .unwrap_or_default(),
                });
            }
        }
    }
    out
}

pub fn agent_matrix() -> Vec<CaseResult> {
    let mut out = Vec::new();
    for skill in SkillIR::catalog() {
        let raw = serde_json::json!({
            "program_name": skill.id,
            "phases": [{"name": "p0", "verb": skill.verb}]
        });
        let admitted = realityos_agent::admit_program(raw).is_ok();
        out.push(CaseResult {
            name: format!("agent|admit|{}", skill.verb),
            ok: admitted,
            expected_block: false,
            blocked: !admitted,
            detail: skill.id,
        });
    }
    for verb in ["dance", "move_actuator", "torque", "metal"] {
        let raw = serde_json::json!({
            "program_name": "bad",
            "phases": [{"name": "p0", "verb": verb}]
        });
        let admitted = realityos_agent::admit_program(raw).is_ok();
        out.push(CaseResult {
            name: format!("agent|refuse|{verb}"),
            ok: !admitted,
            expected_block: true,
            blocked: !admitted,
            detail: verb.into(),
        });
    }
    out
}

pub fn summarize(cases: &[CaseResult]) -> (usize, usize) {
    let n = cases.len();
    let bad = cases.iter().filter(|c| !c.pass()).count();
    (n, bad)
}

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_agent::{admit_program, offline_propose, propose_offline};

    fn assert_matrix(label: &str, cases: Vec<CaseResult>, min: usize) {
        let (n, bad) = summarize(&cases);
        assert!(n >= min, "{label} scenarios {n} < {min}");
        if bad > 0 {
            let names: Vec<_> = cases
                .iter()
                .filter(|c| !c.pass())
                .map(|c| format!("{} block={} expect={}", c.name, c.blocked, c.expected_block))
                .take(16)
                .collect();
            panic!("{bad}/{n} {label} cases failed: {names:?}");
        }
    }

    #[test]
    fn governor_gauntlet_hundreds_and_all_pass() {
        assert_matrix("governor", governor_matrix(), 300);
    }

    #[test]
    fn reality_os_gauntlet_hundreds_and_coherent() {
        assert_matrix("reality-os", reality_os_matrix(), 300);
    }

    #[test]
    fn rate_gauntlet_hz_and_overload() {
        assert_matrix("rate", rate_matrix(), 80);
    }

    #[test]
    fn agent_skillir_only() {
        assert_matrix("agent", agent_matrix(), 10);
        let p = offline_propose("hold the payload");
        assert!(!p.metal);
        assert!(propose_offline("hold still").is_ok());
        let raw = serde_json::json!({
            "program_name": "evil",
            "metal": true,
            "phases": [{"name": "x", "verb": "hold"}]
        });
        let a = admit_program(raw).unwrap();
        assert!(!a.metal);
    }
}
