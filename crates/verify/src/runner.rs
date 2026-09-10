//! Episode and matrix runners. Isolated mjModel/mjData per instance.

use crate::authority::{AuthorityOutcome, AuthorityRecord, SimAuthority};
use crate::bundle::RobotBundle;
use crate::driver::{SharedMujoco, SharedSimPort, SimActuationProbe};
use crate::evidence::{assemble_episode, EpisodeEvidence};
use crate::families::family_spec;
use crate::mujoco_exec::{checkin_worker, checkout_worker, MujocoInstance};
use crate::normalize::RobotManifest;
use crate::observation::{policy_observation, VerifierTruth};
use crate::policy::{
    CrashingPolicy, PdPolicy, Policy, PolicyError, ScriptedHold, ScriptedSetpoint,
};
use crate::qualify::qualify;
use crate::scenario::{applicable, ResolvedScenario};
use crate::task::TaskSpec;
use crate::validate::{validate_bundle, validate_runtime};
use crate::verifier::{inspect_step, update_stats, RuntimeViolation, VerifierStats, ViolationKind};
use realityos_plant::HardwareBackedPlant;
use serde_json::Value;
use std::sync::{Arc, Mutex};
use std::time::Instant;

#[derive(Debug, Clone)]
pub struct EpisodeRequest {
    pub bundle: std::path::PathBuf,
    pub family: String,
    pub seed: u64,
    pub policy: String,
}

pub fn load_and_normalize(
    bundle: &RobotBundle,
    objects: &[Value],
    seed: u64,
) -> Result<(MujocoInstance, RobotManifest), String> {
    let mut inst = checkout_worker().map_err(|e| e.to_string())?;
    let loaded = inst
        .load_bundle(bundle, objects, seed)
        .map_err(|e| e.to_string())?;
    let manifest = RobotManifest::from_inspect(bundle, &loaded["inspect"]);
    Ok((inst, manifest))
}

pub fn run_episode(
    bundle: &RobotBundle,
    family: &str,
    seed: u64,
    policy_name: &str,
) -> Result<EpisodeEvidence, String> {
    let ee = bundle
        .manifest
        .end_effectors
        .first()
        .map(|e| e.name.as_str())
        .unwrap_or("ee");
    let preview = family_spec(family, 3, ee);
    let preview_objects = preview.resolve(seed).objects;
    let (inst, manifest) = load_and_normalize(bundle, &preview_objects, seed)?;
    let spec = family_spec(family, manifest.nu.max(1) as usize, ee);
    let resolved = spec.resolve(seed);
    if resolved.objects != preview_objects {
        checkin_worker(inst);
        return run_resolved(bundle, resolved, policy_name);
    }
    run_loaded(
        bundle,
        inst,
        manifest,
        resolved,
        policy_name,
        Instant::now(),
    )
}

/// Run an already-resolved scenario (used by counterexample minimization).
pub fn run_resolved(
    bundle: &RobotBundle,
    resolved: ResolvedScenario,
    policy_name: &str,
) -> Result<EpisodeEvidence, String> {
    let wall = Instant::now();
    let (inst, manifest) = load_and_normalize(bundle, &resolved.objects, resolved.seed)?;
    run_loaded(bundle, inst, manifest, resolved, policy_name, wall)
}

fn run_loaded(
    bundle: &RobotBundle,
    mut inst: MujocoInstance,
    manifest: RobotManifest,
    resolved: ResolvedScenario,
    policy_name: &str,
    wall: Instant,
) -> Result<EpisodeEvidence, String> {
    let family = resolved.family.as_str();
    let mut report = validate_bundle(bundle, Some(&manifest));
    let inspect = inst.inspect.clone();
    let reset = inst.step(0).unwrap_or(Value::Null);
    let rollout = inst.passive_rollout(20).ok();
    validate_runtime(
        &mut report,
        &inspect,
        reset.get("state").unwrap_or(&reset),
        rollout.as_ref(),
    );
    let _ = inst.reset(None, None);
    if !report.ok() {
        checkin_worker(inst);
        return Err(format!("INVALID:{}", report.errors.join(",")));
    }
    if !applicable(family, bundle, &manifest) {
        checkin_worker(inst);
        return Ok(na_episode(
            &manifest,
            &resolved,
            policy_name,
            wall.elapsed().as_secs_f64(),
        ));
    }

    let robot_bodies: Vec<String> = manifest
        .bodies
        .iter()
        .filter(|b| b.name != "world")
        .map(|b| b.name.clone())
        .collect();

    let shared = Arc::new(SharedMujoco {
        inst: Mutex::new(inst),
        probe: SimActuationProbe::default(),
        robot_id: manifest.robot_id.clone(),
        model_hash: manifest.model_hash.clone(),
    });
    let port = SharedSimPort::new(shared.clone());
    let max_a = manifest.tau_max().into_iter().fold(1.0, f64::max);
    let plant =
        HardwareBackedPlant::new(port, &manifest.robot_id, manifest.nu.max(1) as usize, max_a);
    let mut auth = SimAuthority::open(plant, &manifest, 10.0)?;
    auth.freshness_s = resolved.envelope.observation_freshness_s;
    auth.command_lifetime_s = resolved.envelope.command_lifetime_s;

    let limits = manifest.tau_max();
    let ranges = manifest.ctrl_ranges();
    let position_mask = manifest.position_mask();
    let nu = manifest.nu.max(1) as usize;
    let mut ik_target = None;
    if let TaskSpec::Reach {
        end_effector,
        target,
        ..
    } = &resolved.task
    {
        if let Ok(mut g) = shared.inst.lock() {
            if let Ok((qpos, _err)) = g.solve_ik(end_effector, *target) {
                let mut t = manifest.actuator_qpos(&qpos);
                t.resize(nu, 0.0);
                ik_target = Some(t);
            }
        }
    }
    let mut policy: Box<dyn Policy> = match policy_name {
        "hold" => Box::new(ScriptedHold::default()),
        "crash" => Box::new(CrashingPolicy),
        "setpoint" => Box::new(ScriptedSetpoint {
            id: "setpoint".into(),
            action: limits.iter().map(|l| 0.25 * l).collect(),
        }),
        _ if resolved.policy_crash => Box::new(CrashingPolicy),
        _ => {
            let mut target = vec![0.0; nu];
            if let TaskSpec::JointTrack { target: t, .. } = &resolved.task {
                target = t
                    .iter()
                    .enumerate()
                    .map(|(i, v)| {
                        let [lo, hi] = ranges.get(i).copied().unwrap_or([-1.0, 1.0]);
                        v.clamp(lo, hi)
                    })
                    .collect();
                target.resize(nu, 0.0);
            } else if let Some(t) = ik_target {
                target = t;
            }
            Box::new(PdPolicy::new(target, ranges, position_mask))
        }
    };

    let initial = {
        let mut g = shared.inst.lock().map_err(|e| e.to_string())?;
        let st = g.step(0).map_err(|e| e.to_string())?;
        VerifierTruth::from_mujoco_state(&st["state"])
    };

    let dt = manifest.timestep.max(1e-4);
    let cycles =
        ((resolved.duration_s / (1.0 / resolved.control_hz.max(1.0))).ceil() as u32).clamp(2, 40);
    let substeps = ((1.0 / resolved.control_hz.max(1.0)) / dt).round().max(1.0) as u32;

    let mut decisions = Vec::new();
    let mut violations = Vec::new();
    let mut stats = VerifierStats {
        min_clearance: f64::MAX,
        min_obstacle_distance: f64::MAX,
        ..VerifierStats::default()
    };
    let mut truth = initial.clone();
    let mut termination = "horizon".to_string();
    let mut observations = 0u32;
    let episode_id = format!("{}-{}-{}", manifest.robot_id, family, resolved.seed);
    let mut last_proposal: Option<crate::policy::ActionProposal> = None;

    if let Some(force) = resolved.external_push {
        let preferred = resolved.push_body.clone().unwrap_or_else(|| {
            robot_bodies
                .iter()
                .find(|b| *b != "world" && *b != "base")
                .cloned()
                .or_else(|| robot_bodies.first().cloned())
                .unwrap_or_else(|| "link1".into())
        });
        let fallback = robot_bodies
            .iter()
            .find(|b| *b != "world")
            .cloned()
            .unwrap_or_else(|| preferred.clone());
        let mut g = shared.inst.lock().map_err(|e| e.to_string())?;
        if g.apply_force(&preferred, force).is_err() {
            let _ = g.apply_force(&fallback, force);
        }
    }

    for cyc in 0..cycles {
        let now = 10.0
            + truth
                .time_s
                .max(cyc as f64 * (1.0 / resolved.control_hz.max(1.0)));
        let mut obs = policy_observation(
            &manifest,
            &episode_id,
            &format!("obs-{cyc}"),
            &resolved.task,
            resolved.vision_mode,
            &truth,
            resolved.sensor_delay_s,
            resolved.sensor_dropout,
        );
        obs.timestamp_s = now - resolved.sensor_delay_s;
        if resolved.stale_observation {
            obs.timestamp_s = now - resolved.envelope.observation_freshness_s - 0.05;
        }
        observations += 1;

        let proposed = if resolved.policy_crash {
            Err(PolicyError::Crash("POLICY_CRASH".into()))
        } else {
            policy.propose(&obs)
        };

        match proposed {
            Err(_) => {
                termination = "POLICY_CRASH".into();
                violations.push(RuntimeViolation {
                    kind: ViolationKind::AuthorityViolation,
                    code: "POLICY_CRASH".into(),
                    detail: "policy did not produce ActionProposal".into(),
                    critical: false,
                    time_s: now,
                });
                break;
            }
            Ok(mut proposal) => {
                if resolved.wrong_robot {
                    proposal.robot_id = "wrong_robot_identity".into();
                }
                if resolved.stale_observation {
                    proposal.observation_timestamp = obs.timestamp_s;
                }
                let force_verb = if resolved.wrong_task_authority {
                    Some("place")
                } else {
                    None
                };
                let rec = auth.decide_and_maybe_write(
                    &proposal,
                    &manifest,
                    &resolved.task,
                    &obs,
                    now,
                    None,
                    force_verb,
                );
                if !rec.executed
                    && rec.outcome != AuthorityOutcome::Allowed
                    && rec.violations.iter().any(|v| {
                        v.contains("STALE")
                            || v.contains("REPLAY")
                            || v.contains("WRONG")
                            || v.contains("AUTHORITY")
                    })
                {
                    violations.push(RuntimeViolation {
                        kind: ViolationKind::AuthorityViolation,
                        code: rec
                            .violations
                            .first()
                            .cloned()
                            .unwrap_or_else(|| "AUTHORITY_VIOLATION".into()),
                        detail: rec.physical_reason.clone(),
                        critical: false,
                        time_s: now,
                    });
                }
                if resolved.replay_command || resolved.duplicate_command {
                    if let Some(prev) = last_proposal.clone() {
                        let replay = auth.decide_and_maybe_write(
                            &prev,
                            &manifest,
                            &resolved.task,
                            &obs,
                            now + 0.001,
                            Some(format!("{}:{}", prev.episode_id, prev.observation_id)),
                            None,
                        );
                        decisions.push(replay);
                    }
                }
                last_proposal = Some(proposal);
                decisions.push(rec);
            }
        }

        if resolved.authority_restart && cyc == 1 {
            let port = SharedSimPort::new(shared.clone());
            let plant = HardwareBackedPlant::new(port, &manifest.robot_id, nu, max_a);
            auth.restart(plant, &manifest, now)?;
            if let Some(prev) = last_proposal.clone() {
                let rec = auth.decide_and_maybe_write(
                    &prev,
                    &manifest,
                    &resolved.task,
                    &obs,
                    now,
                    Some(format!("{}:{}", prev.episode_id, prev.observation_id)),
                    None,
                );
                decisions.push(rec);
            }
        }

        let stepped = {
            let mut g = shared.inst.lock().map_err(|e| e.to_string())?;
            g.step(substeps).map_err(|e| e.to_string())?
        };
        truth = VerifierTruth::from_mujoco_state(stepped.get("state").unwrap_or(&stepped));
        let step_v = inspect_step(&manifest, &resolved.envelope, &mut truth, &robot_bodies);
        update_stats(&mut stats, &truth, &resolved.envelope);
        let critical = step_v.iter().any(|v| v.critical);
        violations.extend(step_v);
        if critical {
            termination = "NAN_STATE".into();
            break;
        }
    }

    if resolved.external_push.is_some() {
        let _ = shared
            .inst
            .lock()
            .map_err(|e| e.to_string())?
            .clear_forces();
    }

    let writes = shared.probe.snapshot().write_count;
    drop(auth);
    if let Ok(owned) = Arc::try_unwrap(shared) {
        if let Ok(inst) = owned.inst.into_inner() {
            checkin_worker(inst);
        }
    }
    if matches!(
        family,
        "COMMAND_REPLAY"
            | "DUPLICATE_COMMAND"
            | "WRONG_ROBOT_IDENTITY"
            | "WRONG_TASK_AUTHORITY"
            | "POLICY_CRASH"
            | "AUTHORITY_RESTART"
            | "STALE_OBSERVATION"
    ) && writes == 0
        && decisions.iter().any(|d| !d.executed)
    {
        // unauthorized scenarios must not actuate — recorded as authority success via refusals
    }

    let task_success = match family {
        "COMMAND_REPLAY"
        | "DUPLICATE_COMMAND"
        | "WRONG_ROBOT_IDENTITY"
        | "WRONG_TASK_AUTHORITY"
        | "POLICY_CRASH"
        | "AUTHORITY_RESTART"
        | "STALE_OBSERVATION" => {
            writes == 0
                || (family != "POLICY_CRASH"
                    && family != "WRONG_ROBOT_IDENTITY"
                    && family != "WRONG_TASK_AUTHORITY"
                    && writes <= 1
                    && decisions.iter().any(|d| !d.executed))
        }
        _ => resolved.task.evaluate(&truth, &manifest),
    };
    // Strict unauthorized: wrong identity / place / crash / stale must be zero writes.
    let task_success = match family {
        "WRONG_ROBOT_IDENTITY" | "WRONG_TASK_AUTHORITY" | "POLICY_CRASH" | "STALE_OBSERVATION" => {
            writes == 0
        }
        "COMMAND_REPLAY" | "DUPLICATE_COMMAND" | "AUTHORITY_RESTART" => decisions.iter().any(|d| {
            d.violations
                .iter()
                .any(|v| v.contains("REPLAY") || v.contains("DUPLICATE") || v.contains("AUTHORITY"))
        }),
        _ => task_success,
    };

    Ok(assemble_episode(
        &manifest,
        &resolved,
        policy_name,
        &initial,
        &truth,
        decisions,
        violations,
        &stats,
        task_success,
        false,
        &termination,
        truth.time_s,
        wall.elapsed().as_secs_f64(),
        writes,
        observations,
    ))
}

fn na_episode(
    manifest: &RobotManifest,
    resolved: &ResolvedScenario,
    policy: &str,
    wall: f64,
) -> EpisodeEvidence {
    assemble_episode(
        manifest,
        resolved,
        policy,
        &VerifierTruth::default(),
        &VerifierTruth::default(),
        Vec::new(),
        Vec::new(),
        &VerifierStats::default(),
        false,
        true,
        "NOT_APPLICABLE",
        0.0,
        wall,
        0,
        0,
    )
}

pub fn run_matrix(
    bundles: &[std::path::PathBuf],
    families: &[&str],
    seeds: impl IntoIterator<Item = u64> + Clone,
    policies: &[&str],
    parallel: bool,
) -> Vec<EpisodeEvidence> {
    let seed_list: Vec<u64> = seeds.into_iter().collect();
    let mut jobs: Vec<EpisodeRequest> = Vec::new();
    for b in bundles {
        for f in families {
            for seed in &seed_list {
                for p in policies {
                    jobs.push(EpisodeRequest {
                        bundle: b.clone(),
                        family: (*f).to_string(),
                        seed: *seed,
                        policy: (*p).to_string(),
                    });
                }
            }
        }
    }
    let total = jobs.len();
    let done = std::sync::atomic::AtomicUsize::new(0);
    let run_one = |j: &EpisodeRequest| -> Option<EpisodeEvidence> {
        let out = (|| {
            let bundle = RobotBundle::load(&j.bundle).map_err(|e| e.to_string())?;
            run_episode(&bundle, &j.family, j.seed, &j.policy)
        })();
        let n = done.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
        if n % 100 == 0 || n == total {
            eprintln!("realityos-verify episodes {n}/{total}");
        }
        match out {
            Ok(ep) => Some(ep),
            Err(e) => {
                if e.contains("allocate memory") {
                    std::thread::sleep(std::time::Duration::from_millis(50));
                    match (|| {
                        let bundle = RobotBundle::load(&j.bundle).map_err(|e| e.to_string())?;
                        run_episode(&bundle, &j.family, j.seed, &j.policy)
                    })() {
                        Ok(ep) => return Some(ep),
                        Err(e2) => {
                            eprintln!(
                                "episode_error {} {} seed={} {e2}",
                                j.bundle.display(),
                                j.family,
                                j.seed
                            );
                            return None;
                        }
                    }
                }
                eprintln!(
                    "episode_error {} {} seed={} {e}",
                    j.bundle.display(),
                    j.family,
                    j.seed
                );
                None
            }
        }
    };
    if parallel {
        use rayon::prelude::*;
        let n = std::env::var("REALITYOS_VERIFY_THREADS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or_else(|| {
                std::thread::available_parallelism()
                    .map(|p| p.get().min(6))
                    .unwrap_or(4)
            });
        rayon::ThreadPoolBuilder::new()
            .num_threads(n.max(1))
            .build()
            .expect("rayon")
            .install(|| jobs.par_iter().filter_map(run_one).collect())
    } else {
        jobs.iter().filter_map(run_one).collect()
    }
}

pub fn qualify_bundle(
    bundle: &RobotBundle,
) -> Result<crate::qualify::ControlQualification, String> {
    let (mut inst, manifest) = load_and_normalize(bundle, &[], 0)?;
    let out = qualify(&mut inst, &manifest);
    checkin_worker(inst);
    out
}

#[allow(dead_code)]
fn _keep_authority_record_ty(_: AuthorityRecord) {}
