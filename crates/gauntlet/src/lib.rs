//! Parameterized safety/control gauntlets. One function = many scenarios.

use realityos_core::{Certificate, CertifiedCommand, DecideRequest, Intent, RealityOs, WorldView};
use realityos_embodiment::{catalog as robots, EmbodimentGraph, Environment};
use realityos_governor::SafeState;
use realityos_kernel::DecisionStatus;
use realityos_plant::{ActionParams, SimPlant};
use realityos_session::{RuntimeSession, StartArgs};
use realityos_vision::{see_from_pixels, Camera, Frame};

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
}

impl Fault {
    pub const ALL: [Fault; 10] = [
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

/// Governor safety matrix: robots × environments × faults.
pub fn governor_matrix() -> Vec<CaseResult> {
    let mut out = Vec::new();
    let now = 100.0;
    for robot in robots() {
        for env in Environment::catalog() {
            for fault in Fault::ALL {
                let name = format!("{}|{}|{}", robot.id, env.id, fault.as_str());
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
                match fault {
                    Fault::Happy => {}
                    Fault::Estop => {
                        sess.governor.engage_estop("gauntlet", now);
                    }
                    Fault::StaleSensor => {
                        sess.governor.config_mut().require_sensor_before_write = true;
                        sess.governor.config_mut().sensor_stale_s = 0.01;
                        sess.governor.mark_sensor(now - 10.0, Some("old".into()));
                    }
                    Fault::Replay => {
                        let _ = sess.bind_and_dispatch(cmd.clone(), &ActionParams::empty(), now);
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
                    }
                    Fault::Envelope => {
                        action = vec![1e6; n];
                        cmd = allow_cmd(
                            "g-1",
                            1,
                            now,
                            action.clone(),
                            sess.governor.identity.release_hash.as_str(),
                            sess.governor.identity.calibration_id_str(),
                            true,
                        );
                    }
                    Fault::ForeignHash => {
                        cmd = allow_cmd(
                            "g-1",
                            1,
                            now,
                            action.clone(),
                            "foreign-rel",
                            sess.governor.identity.calibration_id_str(),
                            true,
                        );
                    }
                    Fault::Hold => sess.latch_safe_state(SafeState::Hold, "gauntlet"),
                    Fault::GiftedPlace => {
                        // exercised in reality_os_matrix; governor still sees a hold cmd
                    }
                }
                let blocked = match fault {
                    Fault::GiftedPlace => {
                        let mut ros = RealityOs::new();
                        let d = ros.decide(DecideRequest::new(
                            Intent::language("place", "place"),
                            WorldView::default(),
                            now,
                        ));
                        !d.allowed
                    }
                    Fault::Unacked => {
                        let t = sess
                            .governor
                            .write_driver(&cmd, &ActionParams::empty(), now);
                        !t.ok
                    }
                    _ => {
                        let r = sess.bind_and_dispatch(cmd, &ActionParams::empty(), now);
                        !r.ok
                    }
                };
                let expected = if matches!(fault, Fault::GiftedPlace) {
                    true
                } else {
                    fault.expect_block()
                };
                out.push(CaseResult {
                    name,
                    ok: blocked == expected,
                    expected_block: expected,
                    blocked,
                    detail: env.id.clone(),
                });
            }
        }
    }
    out
}

/// Reality OS layer matrix: robots × domains × vision/env.
pub fn reality_os_matrix() -> Vec<CaseResult> {
    let mut out = Vec::new();
    let now = 1.0;
    let verbs = [
        "hold", "place", "stop", "reach", "pfl", "motor", "energy", "walk",
    ];
    for robot in robots() {
        for env in Environment::catalog() {
            for verb in verbs {
                let mut world = WorldView {
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
                    ..WorldView::default()
                };
                if verb == "pfl" {
                    world.contact_force_n = Some(10.0);
                }
                let mut ros = RealityOs::new();
                let d = ros.decide(DecideRequest::new(Intent::language(verb, verb), world, now));
                let expect_block = matches!(verb, "place" | "walk");
                let blocked = !d.allowed;
                out.push(CaseResult {
                    name: format!("ros|{}|{}|{verb}", robot.id, env.id),
                    ok: blocked == expect_block,
                    expected_block: expect_block,
                    blocked,
                    detail: format!("{} {}", d.status, d.physical_reason),
                });
            }
        }
    }
    // Vision layer
    for (label, frame, cam, expect_ok) in [
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
    ] {
        let s = see_from_pixels(&frame, cam.as_ref());
        out.push(CaseResult {
            name: format!("vision|{label}"),
            ok: s.ok == expect_ok,
            expected_block: !expect_ok,
            blocked: !s.ok,
            detail: s.reason,
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
    use realityos_agent::{admit_program, offline_propose};
    use realityos_core::BoundedTrustEnvelope;
    use realityos_rate::run_dispose_ticks;

    #[test]
    fn governor_gauntlet_at_least_100_and_all_pass() {
        let cases = governor_matrix();
        let (n, bad) = summarize(&cases);
        assert!(n >= 100, "governor scenarios {n}");
        if bad > 0 {
            let names: Vec<_> = cases
                .iter()
                .filter(|c| !c.pass())
                .map(|c| c.name.as_str())
                .take(12)
                .collect();
            panic!("{bad}/{n} governor cases failed: {names:?}");
        }
    }

    #[test]
    fn reality_os_gauntlet_at_least_100_and_coherent() {
        let cases = reality_os_matrix();
        let (n, bad) = summarize(&cases);
        assert!(n >= 100, "reality-os scenarios {n}");
        if bad > 0 {
            let names: Vec<_> = cases
                .iter()
                .filter(|c| !c.pass())
                .map(|c| format!("{} ({})", c.name, c.detail))
                .take(12)
                .collect();
            panic!("{bad}/{n} reality-os cases failed: {names:?}");
        }
    }

    #[test]
    fn rate_gauntlet_six_bands_times_robots() {
        let env = BoundedTrustEnvelope {
            tau_max: vec![1.0],
            dq_max: vec![10.0],
            is_valid: true,
        };
        let mut n = 0;
        for band in realityos_rate::bands() {
            for robot in robots() {
                let u = vec![0.01; robot.dof().max(1)];
                let ticks =
                    run_dispose_ticks(4, band.hz, &u[..1.min(u.len())], &[0.0], &env, false, 1.0);
                assert!(ticks.iter().all(|t| !t.metal));
                n += 1;
            }
        }
        assert!(n >= 20);
    }

    #[test]
    fn agent_cannot_mint_metal() {
        let p = offline_propose("hold the payload");
        assert!(!p.metal);
        let raw = serde_json::json!({
            "program_name": "evil",
            "metal": true,
            "phases": [{"name": "x", "verb": "hold"}]
        });
        let a = admit_program(raw).unwrap();
        assert!(!a.metal);
    }
}
