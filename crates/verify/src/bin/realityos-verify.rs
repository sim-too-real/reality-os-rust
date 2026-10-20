//! CLI for the MuJoCo verification harness. SIMULATION_ONLY. Never metal.

use realityos_kernel::physical_action::{
    AbortHoldMode, WrenchAbortContract, WrenchLimitPolicy, WrenchLimitScope,
};
use realityos_verify::bundle::RobotBundle;
use realityos_verify::corpus::{milestone_robots, robot_dir};
use realityos_verify::evidence::{aggregate, render_markdown};
use realityos_verify::families::{family_spec, MILESTONE_FAMILIES};
use realityos_verify::foundation_report::{run_campaign, write_report};
use realityos_verify::honesty::{refuse_physical_proof_origin, SIMULATION_ONLY};
use realityos_verify::manipulation_benchmark::{
    evaluate_paired_screening_gate, paired_campaign_delta, resolve_benchmark_trial_with_suite,
    score_campaign, write_report as write_benchmark_report, BenchmarkConfig,
    BenchmarkExecutionError, BenchmarkReport, BenchmarkTask, BenchmarkTrialRecord,
    EmbodimentRegistry, EmbodimentRole, ScenarioSuiteManifest, SimulationWrenchSensorProfile,
};
use realityos_verify::mujoco_exec::{ensure_mujoco_or_skip, require_mujoco_env};
use realityos_verify::reduce::minimize;
use realityos_verify::runner::{
    load_and_normalize, qualify_bundle, run_episode, run_matrix, run_resolved,
};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

fn main() {
    let mut args = env::args().skip(1).collect::<Vec<_>>();
    if args.is_empty() {
        eprintln!(
            "usage: realityos-verify [inspect <id>|qualify <id>|episode <id> <family> <seed>|matrix|milestone|foundation-reach]"
        );
        std::process::exit(2);
    }
    let cmd = args.remove(0);
    if cmd != "manipulation-benchmark" && require_mujoco_env() {
        let _ = ensure_mujoco_or_skip();
    }
    match cmd.as_str() {
        "inspect" => inspect(&args[0]),
        "qualify" => qualify(&args[0]),
        "episode" => episode(&args),
        "matrix" | "milestone" => milestone(),
        "foundation-reach" => foundation_reach(),
        "manipulation-benchmark" => manipulation_benchmark(&args),
        other => {
            eprintln!("unknown {other}");
            std::process::exit(2);
        }
    }
}

#[derive(Clone)]
struct BenchmarkBundleSpec {
    robot_id: String,
    role: EmbodimentRole,
    task: BenchmarkTask,
    path: PathBuf,
    wrench_profile: Option<SimulationWrenchSensorProfile>,
}

const SCENARIO_SUITE_V3: &str = "robots/benchmarks/manipulation/scenario_suite_v3.json";

fn manipulation_benchmark(args: &[String]) {
    let parsed = match parse_benchmark_args(args) {
        Ok(parsed) => parsed,
        Err(error) => {
            eprintln!("{error}\nusage: realityos-verify manipulation-benchmark [--seeds N] [--pilot] [--holdout] [--suite FILE] [--out DIR] [--replay TRIAL.json] [--baseline DIR]");
            std::process::exit(2);
        }
    };
    if parsed.holdout {
        eprintln!(
            "suite-v3 does not run the historical WX250S holdout; it is not an untouched holdout"
        );
        std::process::exit(2);
    }
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let (candidate_sha, dirty) = git_revision(&repo_root);
    if parsed.holdout && dirty {
        eprintln!(
            "frozen holdout requires a clean, committed candidate; current worktree is dirty"
        );
        std::process::exit(2);
    }
    if parsed.holdout {
        println!(
            "HOLDOUT_GATE: candidate {} is committed and clean",
            candidate_sha
        );
    }

    let mut config = BenchmarkConfig::default();
    let scenario_suite = if parsed.replay.is_none() {
        let path = parsed
            .suite
            .clone()
            .unwrap_or_else(|| repo_root.join(SCENARIO_SUITE_V3));
        let suite = ScenarioSuiteManifest::load(&path).unwrap_or_else(|error| {
            eprintln!("invalid scenario suite {}: {error}", path.display());
            std::process::exit(2);
        });
        config.pilot_seeds = suite.pilot_seeds.clone();
        config.public_seeds = suite.public_seeds.clone();
        config.scenario_suite_id = Some(suite.suite_id.clone());
        config.scenario_suite_version = Some(suite.suite_version);
        config.scenario_suite_sha256 = Some(suite.digest().unwrap_or_else(|error| {
            eprintln!("cannot digest scenario suite: {error}");
            std::process::exit(2);
        }));
        Some(suite)
    } else {
        None
    };
    let records = if let Some(replay_path) = parsed.replay.as_ref() {
        let bytes = match fs::read(replay_path) {
            Ok(bytes) => bytes,
            Err(error) => {
                eprintln!(
                    "cannot read replay record {}: {error}",
                    replay_path.display()
                );
                std::process::exit(2);
            }
        };
        let saved: BenchmarkTrialRecord = match serde_json::from_slice(&bytes) {
            Ok(record) => record,
            Err(error) => {
                eprintln!("invalid replay record {}: {error}", replay_path.display());
                std::process::exit(2);
            }
        };
        let Some(trial) = saved.trial else {
            eprintln!("replay record has no resolved trial input");
            std::process::exit(2);
        };
        config.schema_version = trial.schema_version;
        config.scheduled_trial_ids.push(trial.trial_id.clone());
        let bundle = load_trial_bundle(&repo_root, &trial.robot_id).unwrap_or_else(|error| {
            eprintln!("cannot load replay bundle: {error}");
            std::process::exit(2);
        });
        let profile =
            load_trial_wrench_profile(&repo_root, &trial.robot_id).unwrap_or_else(|error| {
                eprintln!("cannot load replay wrench profile: {error}");
                std::process::exit(2);
            });
        let admission = simulation_admission_for_profile(120.0, profile.as_ref());
        vec![BenchmarkTrialRecord::from_run_result(
            trial.clone(),
            realityos_verify::manipulation_benchmark::run_benchmark_trial(
                &bundle,
                &trial,
                admission,
                profile.as_ref(),
            ),
        )]
    } else {
        let seeds = selected_benchmark_seeds(&config, parsed.seeds, parsed.pilot, parsed.holdout);
        if seeds.is_empty() {
            eprintln!("seed count must be at least 1");
            std::process::exit(2);
        }
        if seeds.len() > 100 {
            eprintln!("one benchmark invocation is limited to 100 frozen trials per seed set");
            std::process::exit(2);
        }
        let (specs, registry_digest) =
            benchmark_bundle_specs(&repo_root, false).unwrap_or_else(|error| {
                eprintln!("invalid benchmark embodiment registry: {error}");
                std::process::exit(2);
            });
        config.embodiment_registry_sha256 = Some(registry_digest);
        let suite = scenario_suite
            .as_ref()
            .expect("new campaigns load the validated scenario suite");
        let mut records = Vec::new();
        for spec in specs {
            records.extend(run_bundle_campaign(&spec, &seeds, &mut config, suite));
        }
        records
    };

    let mut report = match score_campaign(&config, &records) {
        Ok(report) => report,
        Err(error) => {
            eprintln!("benchmark scoring failed: {error}");
            std::process::exit(1);
        }
    };
    report.candidate_source_sha = Some(candidate_sha);
    report.candidate_worktree_dirty = dirty;
    let mut pairing_error: Option<String> = None;
    if let Some(baseline_dir) = parsed.baseline.as_ref() {
        report.baseline_source_sha = baseline_candidate_sha(baseline_dir);
        match load_baseline_records(baseline_dir).and_then(|baseline| {
            paired_campaign_delta(&baseline, &records).map_err(|error| error.to_string())
        }) {
            Ok(delta) => {
                let gate = evaluate_paired_screening_gate(&delta);
                println!(
                    "PAIRED_BASELINE matched={} screening_gate={} best_gain={:?} worst_regression={:?}",
                    delta.matched_trial_keys,
                    gate.passed,
                    gate.best_gain,
                    gate.worst_regression
                );
                report.paired_screening_gate = Some(gate);
                report.paired_delta = Some(delta);
                report.baseline_status = "suite_v3_paired_preserved_pilot".into();
            }
            Err(error) => {
                pairing_error = Some(error);
            }
        }
    } else {
        report.baseline_source_sha = None;
        report.baseline_status = if parsed.replay.is_some() {
            "historical_replay_not_compared".into()
        } else if parsed.pilot {
            "suite_v3_pilot_screening_not_compared".into()
        } else {
            "suite_v3_public_baseline".into()
        };
    }
    if let Some(error) = pairing_error.as_ref() {
        report.baseline_status = format!("suite_v3_pairing_failed: {error}");
    }
    if let Err(error) = write_benchmark_report(&report, &parsed.out_dir) {
        eprintln!("benchmark report write failed: {error}");
        std::process::exit(1);
    }
    println!(
        "MANIPULATION_BENCHMARK scheduled={} completed={} infra={} invalid={} unaccounted={} safety_gate={} task_success={}/{} correct_refusal={}/{} false_refusals={} false_acceptances={} evaluation_label_mismatches={} unauthorized_writes={} friction_narrowings={}/{}",
        report.scheduled_trials,
        report.completed_trials,
        report.infrastructure_trials,
        report.invalid_trials,
        report.unaccounted_trials,
        report.safety_gate_passed,
        report.task_success.numerator,
        report.task_success.denominator,
        report.correct_refusal.numerator,
        report.correct_refusal.denominator,
        report.false_refusals,
        report.false_acceptances,
        report.evaluation_label_mismatches,
        report.unauthorized_writes,
        report.wrench_metrics.strict_friction_narrowings,
        report.wrench_metrics.episodes_with_samples,
    );
    println!(
        "wrote {}/manipulation_benchmark.json and raw trial records",
        parsed.out_dir.display()
    );
    if let Some(error) = pairing_error {
        eprintln!("baseline pairing failed: {error}");
        std::process::exit(1);
    }
    if !report.safety_gate_passed {
        std::process::exit(1);
    }
    if parsed.pilot && !suite_pilot_has_positive_denominators(&report) {
        eprintln!("suite-v3 pilot did not schedule positive trials on both development arms");
        std::process::exit(1);
    }
}

struct BenchmarkArgs {
    seeds: Option<usize>,
    pilot: bool,
    holdout: bool,
    out_dir: PathBuf,
    replay: Option<PathBuf>,
    suite: Option<PathBuf>,
    baseline: Option<PathBuf>,
}

fn parse_benchmark_args(args: &[String]) -> Result<BenchmarkArgs, String> {
    let mut parsed = BenchmarkArgs {
        seeds: None,
        pilot: false,
        holdout: false,
        out_dir: PathBuf::from("verify-out/manipulation-benchmark"),
        replay: None,
        suite: None,
        baseline: None,
    };
    let mut index = 0;
    while index < args.len() {
        match args[index].as_str() {
            "--seeds" => {
                index += 1;
                let value = args.get(index).ok_or("--seeds requires a number")?;
                parsed.seeds = Some(
                    value
                        .parse::<usize>()
                        .map_err(|_| "--seeds must be a positive integer")?,
                );
            }
            "--pilot" => parsed.pilot = true,
            "--holdout" => parsed.holdout = true,
            "--out" => {
                index += 1;
                parsed.out_dir =
                    PathBuf::from(args.get(index).ok_or("--out requires a directory")?);
            }
            "--replay" => {
                index += 1;
                parsed.replay = Some(PathBuf::from(
                    args.get(index).ok_or("--replay requires a trial record")?,
                ));
            }
            "--suite" => {
                index += 1;
                parsed.suite = Some(PathBuf::from(
                    args.get(index).ok_or("--suite requires a scenario file")?,
                ));
            }
            "--baseline" => {
                index += 1;
                parsed.baseline = Some(PathBuf::from(
                    args.get(index)
                        .ok_or("--baseline requires a report directory")?,
                ));
            }
            other => return Err(format!("unknown benchmark option: {other}")),
        }
        index += 1;
    }
    if parsed.holdout && parsed.pilot {
        return Err("--holdout and --pilot are separate seed sets".into());
    }
    if parsed.replay.is_some()
        && (parsed.holdout || parsed.pilot || parsed.seeds.is_some() || parsed.baseline.is_some())
    {
        return Err(
            "--replay cannot be combined with --holdout, --pilot, --seeds, or --baseline".into(),
        );
    }
    if parsed.seeds == Some(0) {
        return Err("--seeds must be greater than zero".into());
    }
    Ok(parsed)
}

fn selected_benchmark_seeds(
    config: &BenchmarkConfig,
    count: Option<usize>,
    pilot: bool,
    holdout: bool,
) -> Vec<u64> {
    let source = if holdout {
        &config.holdout_seeds
    } else if pilot {
        &config.pilot_seeds
    } else {
        &config.public_seeds
    };
    source
        .iter()
        .copied()
        .take(count.unwrap_or(source.len()))
        .collect()
}

fn suite_pilot_has_positive_denominators(report: &BenchmarkReport) -> bool {
    let mut denominators = std::collections::BTreeMap::<String, u64>::new();
    for trial in report
        .records
        .iter()
        .filter_map(|record| record.trial.as_ref())
    {
        if trial.role == EmbodimentRole::Development
            && trial.task == BenchmarkTask::PushProbeReplan
            && trial.expectation
                == realityos_verify::manipulation_benchmark::TrialExpectation::ExpectedSuccess
        {
            *denominators.entry(trial.robot_id.clone()).or_default() += 1;
        }
    }
    denominators.len() >= 2 && denominators.values().all(|denominator| *denominator > 0)
}

fn benchmark_bundle_specs(
    repo_root: &Path,
    holdout: bool,
) -> Result<(Vec<BenchmarkBundleSpec>, String), String> {
    let registry_path = repo_root.join("robots/benchmarks/manipulation/embodiments.json");
    let registry = EmbodimentRegistry::load(&registry_path).map_err(|error| error.to_string())?;
    let digest = registry.digest().map_err(|error| error.to_string())?;
    let mut specs = Vec::new();
    for entry in registry.selected(holdout) {
        let relative = PathBuf::from(&entry.bundle_path);
        if relative.is_absolute()
            || relative
                .components()
                .any(|component| matches!(component, std::path::Component::ParentDir))
        {
            return Err(format!(
                "bundle path escapes repository: {}",
                entry.bundle_path
            ));
        }
        for task in &entry.tasks {
            specs.push(BenchmarkBundleSpec {
                robot_id: entry.robot_id.clone(),
                role: entry.role,
                task: *task,
                path: repo_root.join(relative.clone()),
                wrench_profile: entry.simulation_wrench_profile.clone(),
            });
        }
    }
    Ok((specs, digest))
}

fn run_bundle_campaign(
    spec: &BenchmarkBundleSpec,
    seeds: &[u64],
    config: &mut BenchmarkConfig,
    suite: &ScenarioSuiteManifest,
) -> Vec<BenchmarkTrialRecord> {
    let mut records = Vec::new();
    let mut schedule_error = None;
    let bundle = match RobotBundle::load(&spec.path) {
        Ok(bundle) if bundle.manifest.robot_id == spec.robot_id => Some(bundle),
        Ok(bundle) => {
            schedule_error = Some(format!(
                "registered robot id {} does not match bundle id {}",
                spec.robot_id, bundle.manifest.robot_id
            ));
            None
        }
        Err(error) => {
            schedule_error = Some(format!("bundle_load:{}", error));
            None
        }
    };
    let inspected = if let Some(bundle) = bundle.as_ref() {
        load_and_normalize(bundle, &[], seeds[0])
            .map(|(worker, manifest)| {
                realityos_verify::mujoco_exec::checkin_worker(worker);
                manifest
            })
            .map_err(|error| format!("strict_mujoco_preflight:{error}"))
    } else {
        Err(schedule_error
            .clone()
            .unwrap_or_else(|| "bundle unavailable".into()))
    };
    let manifest = match inspected {
        Ok(manifest) => Some(manifest),
        Err(error) => {
            schedule_error = Some(error);
            None
        }
    };

    for (offset, seed) in seeds.iter().copied().enumerate() {
        let trial_index = match spec.task {
            BenchmarkTask::PushProbeReplan => offset as u32,
            BenchmarkTask::PinchGraspLift if offset == 1 => 0,
            BenchmarkTask::PinchGraspLift => 15_u32.saturating_add(offset as u32),
        };
        let fallback_id = format!(
            "v{}:{}:{}:{}:{}:{}",
            realityos_verify::manipulation_benchmark::BENCHMARK_SCHEMA_VERSION,
            spec.role.as_str(),
            spec.robot_id,
            spec.task.as_str(),
            seed,
            trial_index
        );
        if let Some(error) = schedule_error.as_ref() {
            config.scheduled_trial_ids.push(fallback_id.clone());
            records.push(BenchmarkTrialRecord::failed_before_resolution(
                fallback_id,
                None,
                BenchmarkExecutionError::Infrastructure(error.clone()),
            ));
            continue;
        }
        let bundle = bundle.as_ref().expect("manifest requires loaded bundle");
        let manifest = manifest.as_ref().expect("manifest was inspected");
        let progress = env::var_os("REALITYOS_BENCHMARK_PROGRESS").is_some();
        let resolve_started = std::time::Instant::now();
        if progress {
            eprintln!(
                "benchmark trial start robot={} task={} seed={} index={} phase=resolve",
                spec.robot_id,
                spec.task.as_str(),
                seed,
                trial_index
            );
        }
        let trial = match resolve_benchmark_trial_with_suite(
            seed,
            trial_index,
            spec.task,
            spec.role,
            bundle,
            manifest,
            suite,
        ) {
            Ok(trial) => trial,
            Err(error) => {
                if progress {
                    eprintln!(
                        "benchmark trial invalid robot={} task={} seed={} index={} phase=resolve elapsed_s={:.3} error={}",
                        spec.robot_id,
                        spec.task.as_str(),
                        seed,
                        trial_index,
                        resolve_started.elapsed().as_secs_f64(),
                        error
                    );
                }
                config.scheduled_trial_ids.push(fallback_id.clone());
                records.push(BenchmarkTrialRecord::failed_before_resolution(
                    fallback_id,
                    None,
                    BenchmarkExecutionError::InvalidTrial(error.to_string()),
                ));
                continue;
            }
        };
        if progress {
            eprintln!(
                "benchmark trial resolved robot={} task={} seed={} index={} expectation={:?} elapsed_s={:.3} phase=execute",
                spec.robot_id,
                spec.task.as_str(),
                seed,
                trial_index,
                trial.expectation,
                resolve_started.elapsed().as_secs_f64()
            );
        }
        config.scheduled_trial_ids.push(trial.trial_id.clone());
        let execution_started = std::time::Instant::now();
        let result = realityos_verify::manipulation_benchmark::run_benchmark_trial(
            bundle,
            &trial,
            simulation_admission_for_profile(120.0, spec.wrench_profile.as_ref()),
            spec.wrench_profile.as_ref(),
        );
        if progress {
            eprintln!(
                "benchmark trial end robot={} task={} seed={} index={} execution_s={:.3} result={}",
                spec.robot_id,
                spec.task.as_str(),
                seed,
                trial_index,
                execution_started.elapsed().as_secs_f64(),
                result
                    .as_ref()
                    .map(|_| "completed".to_owned())
                    .unwrap_or_else(|error| error.to_string())
            );
        }
        records.push(BenchmarkTrialRecord::from_run_result(trial, result));
    }
    records
}

fn simulation_admission(expires_at_s: f64) -> realityos_reasoner_bridge::SimulationAdmission {
    let mut wrench_limit_policy = WrenchLimitPolicy {
        scope: WrenchLimitScope {
            sensor_id: "ft-tool".into(),
            sensor_frame: "tool0".into(),
            caps: [20.0, 20.0, 30.0, 2.0, 2.0, 2.0],
            policy_digest: String::new(),
            abort_contract_id: "abort-and-reobserve".into(),
        },
        sensor_calibration_hash: "cli-sensor-calibration-v1".into(),
        sensor_sample_rate_hz: 125.0,
        max_position_tracking_correction_rad: 0.0,
        position_tracking_correction_gain: 0.0,
        max_rise_rate: [10.0; 6],
        max_sensing_latency_s: 0.01,
        max_abort_latency_s: 0.03,
        max_sample_age_s: 0.02,
        max_inter_sample_gap_s: 0.01,
        max_cross_sensor_skew_s: 0.005,
        monitor_id: "cli-wrench-monitor-v1".into(),
        compensation_digest: "c".repeat(64),
        abort_contract: WrenchAbortContract {
            contract_id: "abort-and-reobserve".into(),
            hold_mode: AbortHoldMode::HoldCurrentJointPosition,
            allowed_stop_actuator_ids: vec!["tool".into()],
            max_time_to_stop_s: 0.03,
            max_stop_writes: 1,
        },
    };
    wrench_limit_policy.scope.policy_digest = wrench_limit_policy.digest();
    realityos_reasoner_bridge::SimulationAdmission {
        execution_bound_m: 0.1,
        expires_at_s,
        wrench_limit_policy,
    }
}

fn simulation_admission_for_profile(
    expires_at_s: f64,
    profile: Option<&SimulationWrenchSensorProfile>,
) -> realityos_reasoner_bridge::SimulationAdmission {
    if let Some(profile) = profile {
        return realityos_reasoner_bridge::SimulationAdmission {
            execution_bound_m: 0.1,
            expires_at_s,
            wrench_limit_policy: profile.limit_policy.clone(),
        };
    }
    simulation_admission(expires_at_s)
}

fn load_trial_wrench_profile(
    repo_root: &Path,
    robot_id: &str,
) -> Result<Option<SimulationWrenchSensorProfile>, String> {
    let registry_path = repo_root.join("robots/benchmarks/manipulation/embodiments.json");
    let registry = EmbodimentRegistry::load(&registry_path).map_err(|error| error.to_string())?;
    registry
        .registrations
        .iter()
        .find(|entry| entry.robot_id == robot_id)
        .map(|entry| entry.simulation_wrench_profile.clone())
        .ok_or_else(|| format!("robot {robot_id} is not registered in manipulation benchmark"))
}

fn load_trial_bundle(repo_root: &Path, robot_id: &str) -> Result<RobotBundle, String> {
    let registry_path = repo_root.join("robots/benchmarks/manipulation/embodiments.json");
    let registry = EmbodimentRegistry::load(&registry_path).map_err(|error| error.to_string())?;
    let entry = registry
        .registrations
        .iter()
        .find(|entry| entry.robot_id == robot_id)
        .ok_or_else(|| format!("robot {robot_id} is not registered in manipulation benchmark"))?;
    RobotBundle::load(repo_root.join(&entry.bundle_path)).map_err(|error| error.to_string())
}

fn load_baseline_records(dir: &Path) -> Result<Vec<BenchmarkTrialRecord>, String> {
    let trials = dir.join("trials");
    let mut paths = Vec::new();
    for entry in fs::read_dir(&trials)
        .map_err(|error| format!("baseline trials {}: {error}", trials.display()))?
    {
        let path = entry.map_err(|error| error.to_string())?.path();
        if path.extension().and_then(|ext| ext.to_str()) == Some("json") {
            paths.push(path);
        }
    }
    paths.sort();
    if paths.is_empty() {
        return Err(format!("no baseline trial records in {}", trials.display()));
    }
    let mut records = Vec::with_capacity(paths.len());
    for path in paths {
        let bytes = fs::read(&path).map_err(|error| format!("{}: {error}", path.display()))?;
        let record = realityos_verify::manipulation_benchmark::load_benchmark_trial_record(&bytes)
            .map_err(|error| format!("{}: {error}", path.display()))?;
        records.push(record);
    }
    Ok(records)
}

fn baseline_candidate_sha(dir: &Path) -> Option<String> {
    let bytes = fs::read(dir.join("manipulation_benchmark.json")).ok()?;
    let value: serde_json::Value = serde_json::from_slice(&bytes).ok()?;
    value
        .get("candidate_source_sha")
        .and_then(|sha| sha.as_str())
        .map(str::to_string)
}

fn git_revision(repo_root: &Path) -> (String, bool) {
    let revision = std::process::Command::new("git")
        .args(["rev-parse", "HEAD"])
        .current_dir(repo_root)
        .output()
        .ok()
        .filter(|output| output.status.success())
        .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_string())
        .unwrap_or_else(|| "unknown".into());
    let dirty = std::process::Command::new("git")
        .args(["status", "--porcelain"])
        .current_dir(repo_root)
        .output()
        .map(|output| !output.stdout.is_empty())
        .unwrap_or(true);
    (revision, dirty)
}

fn inspect(id: &str) {
    let b = RobotBundle::load(robot_dir(id)).expect("bundle");
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "robot_id": b.manifest.robot_id,
            "format": b.format,
            "source_hash": b.source_hash,
            "metal": false,
            "evidence_status": SIMULATION_ONLY,
        }))
        .unwrap()
    );
}

fn qualify(id: &str) {
    let b = RobotBundle::load(robot_dir(id)).expect("bundle");
    let q = qualify_bundle(&b).expect("qualify");
    println!("{}", serde_json::to_string_pretty(&q).unwrap());
}

fn episode(args: &[String]) {
    let id = &args[0];
    let family = args.get(1).map(String::as_str).unwrap_or("JOINT_TRACKING");
    let seed = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1);
    let b = RobotBundle::load(robot_dir(id)).expect("bundle");
    let ep = run_episode(&b, family, seed, "pd").expect("episode");
    refuse_physical_proof_origin(&ep.to_json_value()).expect("sim evidence");
    println!("{}", serde_json::to_string_pretty(&ep).unwrap());
}

fn milestone() {
    let out = PathBuf::from("verify-out");
    fs::create_dir_all(&out).ok();
    let robots = milestone_robots();
    for path in &robots {
        if let Ok(b) = RobotBundle::load(path) {
            if let Ok(q) = qualify_bundle(&b) {
                let _ = fs::write(
                    out.join(format!(
                        "control_qualification_{}.json",
                        b.manifest.robot_id
                    )),
                    serde_json::to_string_pretty(&q).unwrap(),
                );
            }
        }
    }
    let n_seeds: u64 = env::var("REALITYOS_VERIFY_SEEDS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(100);
    let seeds: Vec<u64> = (0..n_seeds).collect();
    let n_policies = 1_u64;
    let scheduled = robots.len() as u64 * MILESTONE_FAMILIES.len() as u64 * n_seeds * n_policies;
    let episodes = run_matrix(&robots, MILESTONE_FAMILIES, seeds, &["pd"], true);
    let mut report = aggregate(&episodes);
    report.scheduled_jobs = scheduled;
    report.unaccounted_jobs = scheduled.saturating_sub(episodes.len() as u64);
    report.campaign_complete = report.scheduled_jobs
        == report.completed_jobs + report.infra_error_jobs
        && report.unaccounted_jobs == 0;
    refuse_physical_proof_origin(&serde_json::to_value(&report).unwrap()).expect("report");
    fs::write(
        out.join("verification_report.json"),
        serde_json::to_string_pretty(&report).unwrap(),
    )
    .unwrap();
    fs::write(out.join("VERIFICATION_REPORT.md"), render_markdown(&report)).unwrap();
    for (i, ep) in episodes.iter().enumerate() {
        if i < 8 || !ep.task_success || ep.infra_error.is_some() {
            let _ = ep.write(out.join(format!(
                "episode_{}_{}_{}.json",
                ep.robot_id, ep.family, ep.seed
            )));
        }
    }
    if let Some(fail) = episodes.iter().find(|e| {
        !e.task_success
            && !e.not_applicable
            && e.infra_error.is_none()
            && !matches!(
                e.family.as_str(),
                "COMMAND_REPLAY"
                    | "DUPLICATE_COMMAND"
                    | "WRONG_ROBOT_IDENTITY"
                    | "WRONG_TASK_AUTHORITY"
                    | "POLICY_CRASH"
                    | "AUTHORITY_RESTART"
                    | "STALE_OBSERVATION"
            )
    }) {
        if let Ok(bundle) = RobotBundle::load(robot_dir(&fail.robot_id)) {
            let ee = bundle
                .manifest
                .end_effectors
                .first()
                .map(|e| e.name.as_str())
                .unwrap_or("ee");
            let spec = family_spec(&fail.family, 3, ee);
            let reduced = minimize(&spec, fail, |resolved| {
                run_resolved(&bundle, resolved.clone(), "pd").ok()
            });
            let _ = fs::write(
                out.join("counterexample.json"),
                serde_json::to_string_pretty(&reduced).unwrap(),
            );
        }
    }
    println!(
        "SIMULATION CAMPAIGN COMPLETE scheduled={} completed={} na={} infra={} metal=false evidence_status={} verification_class=SIMULATION_VERIFIED",
        report.scheduled_jobs,
        report.completed_jobs,
        report.not_applicable_jobs,
        report.infra_error_jobs,
        SIMULATION_ONLY
    );
    println!("wrote verify-out/verification_report.json");
    if !report.campaign_complete || report.unaccounted_jobs > 0 {
        eprintln!(
            "unaccounted jobs: scheduled={} results={} infra={}",
            report.scheduled_jobs,
            episodes.len(),
            report.infra_error_jobs
        );
        std::process::exit(1);
    }
    if report.infra_error_jobs > 0 {
        eprintln!("infra errors remain visible; milestone exits non-zero");
        std::process::exit(1);
    }
}

fn foundation_reach() {
    let out = PathBuf::from("verify-out");
    let report = run_campaign().expect("foundation-reach campaign");
    write_report(&report, &out).expect("write foundation report");
    println!(
        "FOUNDATION REACH scenarios={} successes={} refusals={} probes={} ctrl_writes={} metal=false evidence_status={}",
        report.scenario_count,
        report.successes,
        report.refusals,
        report.probes,
        report.ctrl_writes,
        SIMULATION_ONLY
    );
    for r in &report.robots {
        println!(
            "  {} ({}) success={} refuse={:?} writes={}",
            r.robot_id, r.role, r.reach.task_success, r.reach.skill_refuse, r.reach.ctrl_writes
        );
    }
    println!("wrote verify-out/foundation_report.json");
}
