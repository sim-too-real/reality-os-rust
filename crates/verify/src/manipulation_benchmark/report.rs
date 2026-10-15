//! Auditable campaign accounting and reliability metrics.

use super::execution::{BenchmarkEpisode, FailureCause, TrialOutcome};
use super::scenario::{
    BenchmarkConfig, BenchmarkTask, EmbodimentRole, ResolvedBenchmarkTrial, TrialExpectation,
};
use super::BenchmarkError;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;

const WILSON_Z_95: f64 = 1.959_963_984_540_054;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", content = "message", rename_all = "snake_case")]
pub enum BenchmarkExecutionError {
    Infrastructure(String),
    InvalidTrial(String),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BenchmarkTrialRecord {
    pub trial_id: String,
    pub trial: Option<ResolvedBenchmarkTrial>,
    pub expectation: Option<TrialExpectation>,
    pub episode: Option<BenchmarkEpisode>,
    pub error: Option<BenchmarkExecutionError>,
}

impl BenchmarkTrialRecord {
    pub fn completed(trial: ResolvedBenchmarkTrial, episode: BenchmarkEpisode) -> Self {
        Self {
            trial_id: trial.trial_id.clone(),
            expectation: Some(trial.expectation),
            trial: Some(trial),
            episode: Some(episode),
            error: None,
        }
    }

    pub fn failed(trial: ResolvedBenchmarkTrial, error: BenchmarkExecutionError) -> Self {
        Self {
            trial_id: trial.trial_id.clone(),
            expectation: Some(trial.expectation),
            trial: Some(trial),
            episode: None,
            error: Some(error),
        }
    }

    pub fn failed_before_resolution(
        trial_id: String,
        expectation: Option<TrialExpectation>,
        error: BenchmarkExecutionError,
    ) -> Self {
        Self {
            trial_id,
            trial: None,
            expectation,
            episode: None,
            error: Some(error),
        }
    }

    pub fn from_run_result(
        trial: ResolvedBenchmarkTrial,
        result: Result<BenchmarkEpisode, BenchmarkError>,
    ) -> Self {
        match result {
            Ok(episode) => Self::completed(trial, episode),
            Err(BenchmarkError::Simulation(message)) => {
                Self::failed(trial, BenchmarkExecutionError::Infrastructure(message))
            }
            Err(error) => Self::failed(
                trial,
                BenchmarkExecutionError::InvalidTrial(error.to_string()),
            ),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct RateEstimate {
    pub numerator: u64,
    pub denominator: u64,
    pub rate: Option<f64>,
    pub confidence_95: Option<[f64; 2]>,
}

impl RateEstimate {
    fn new(numerator: u64, denominator: u64) -> Self {
        if denominator == 0 {
            return Self {
                numerator,
                denominator,
                rate: None,
                confidence_95: None,
            };
        }
        let p = numerator as f64 / denominator as f64;
        let n = denominator as f64;
        let z2 = WILSON_Z_95 * WILSON_Z_95;
        let scale = 1.0 + z2 / n;
        let center = (p + z2 / (2.0 * n)) / scale;
        let margin = WILSON_Z_95 * ((p * (1.0 - p) / n + z2 / (4.0 * n * n)).sqrt()) / scale;
        let lower = if numerator == 0 {
            0.0
        } else {
            (center - margin).max(0.0)
        };
        let upper = if numerator == denominator {
            1.0
        } else {
            (center + margin).min(1.0)
        };
        Self {
            numerator,
            denominator,
            rate: Some(p),
            confidence_95: Some([lower, upper]),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct BenchmarkSliceReport {
    pub task_success: RateEstimate,
    pub correct_refusal: RateEstimate,
    pub false_refusals: u64,
    pub false_acceptances: u64,
    pub acquisition: RateEstimate,
    pub verified_hold: RateEstimate,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BenchmarkReport {
    pub schema_version: u32,
    pub config: BenchmarkConfig,
    pub campaign_complete: bool,
    pub safety_gate_passed: bool,
    pub scheduled_trials: u64,
    pub completed_trials: u64,
    pub infrastructure_trials: u64,
    pub invalid_trials: u64,
    pub unaccounted_trials: u64,
    pub missing_trial_ids: Vec<String>,
    pub task_success: RateEstimate,
    pub correct_refusal: RateEstimate,
    pub false_refusals: u64,
    pub false_acceptances: u64,
    pub unauthorized_writes: u64,
    pub geometry_omissions: u64,
    pub stale_authority_continuations: u64,
    pub recovery_rate: RateEstimate,
    pub recovery_trials: u64,
    pub acquisition: RateEstimate,
    pub verified_hold: RateEstimate,
    pub wall_time_p50_s: Option<f64>,
    pub wall_time_p95_s: Option<f64>,
    pub simulation_time_total_s: f64,
    pub failure_causes: BTreeMap<String, u64>,
    pub slices: BTreeMap<String, BenchmarkSliceReport>,
    /// Full resolved inputs and either an episode or an explicit error per run.
    pub records: Vec<BenchmarkTrialRecord>,
    pub candidate_source_sha: Option<String>,
    pub candidate_worktree_dirty: bool,
    pub baseline_source_sha: Option<String>,
    pub baseline_status: String,
}

pub fn score_campaign(
    config: &BenchmarkConfig,
    records: &[BenchmarkTrialRecord],
) -> Result<BenchmarkReport, BenchmarkError> {
    if config.schema_version == 0 || config.scheduled_trial_ids.is_empty() {
        return Err(BenchmarkError::InvalidManifest(
            "campaign scoring requires a versioned, non-empty scheduled_trial_ids manifest".into(),
        ));
    }
    if !config.target_ci_half_width.is_finite()
        || !(0.0..=1.0).contains(&config.target_ci_half_width)
    {
        return Err(BenchmarkError::InvalidManifest(
            "target confidence-interval half-width must be finite and within [0,1]".into(),
        ));
    }

    let scheduled = config.scheduled_trial_ids.iter().collect::<BTreeSet<_>>();
    if scheduled.len() != config.scheduled_trial_ids.len() {
        return Err(BenchmarkError::InvalidManifest(
            "scheduled trial identifiers must be unique".into(),
        ));
    }
    let mut by_id = BTreeMap::new();
    for record in records {
        let id = &record.trial_id;
        if !scheduled.contains(id) {
            return Err(BenchmarkError::InvalidTrial(format!(
                "record {id} is not present in the scheduled denominator"
            )));
        }
        if by_id.insert(id.as_str(), record).is_some() {
            return Err(BenchmarkError::InvalidTrial(format!(
                "duplicate result record for trial {id}"
            )));
        }
        match (&record.episode, &record.error) {
            (Some(episode), None) => {
                let Some(trial) = record.trial.as_ref() else {
                    return Err(BenchmarkError::InvalidTrial(format!(
                        "episode {id} is missing its resolved trial input"
                    )));
                };
                if episode.trial_id != record.trial_id
                    || episode.expectation != trial.expectation
                    || record.expectation != Some(trial.expectation)
                {
                    return Err(BenchmarkError::InvalidTrial(format!(
                        "episode identity or frozen expectation mismatch for {id}"
                    )));
                }
            }
            (None, Some(_)) => {}
            _ => {
                return Err(BenchmarkError::InvalidTrial(format!(
                    "record {id} must contain exactly one episode or execution error"
                )))
            }
        }
    }

    let missing_trial_ids = config
        .scheduled_trial_ids
        .iter()
        .filter(|id| !by_id.contains_key(id.as_str()))
        .cloned()
        .collect::<Vec<_>>();
    let episodes = records
        .iter()
        .filter_map(|record| record.episode.as_ref())
        .collect::<Vec<_>>();
    let completed_trials = episodes.len() as u64;
    let infrastructure_trials = records
        .iter()
        .filter(|record| {
            matches!(
                record.error,
                Some(BenchmarkExecutionError::Infrastructure(_))
            )
        })
        .count() as u64;
    let invalid_trials = records
        .iter()
        .filter(|record| matches!(record.error, Some(BenchmarkExecutionError::InvalidTrial(_))))
        .count() as u64;
    let scheduled_trials = config.scheduled_trial_ids.len() as u64;
    let unaccounted_trials = missing_trial_ids.len() as u64;

    let expected_success = episodes
        .iter()
        .filter(|episode| episode.expectation == TrialExpectation::ExpectedSuccess)
        .count() as u64;
    let task_success_n = episodes
        .iter()
        .filter(|episode| {
            episode.expectation == TrialExpectation::ExpectedSuccess
                && episode.outcome == TrialOutcome::Success
        })
        .count() as u64;
    let expected_refusal = episodes
        .iter()
        .filter(|episode| episode.expectation == TrialExpectation::ExpectedRefusal)
        .count() as u64;
    let correct_refusal_n = episodes
        .iter()
        .filter(|episode| {
            episode.expectation == TrialExpectation::ExpectedRefusal
                && episode.outcome == TrialOutcome::ValidRefusal
        })
        .count() as u64;
    let false_refusals = episodes
        .iter()
        .filter(|episode| {
            episode.expectation == TrialExpectation::ExpectedSuccess
                && matches!(
                    episode.outcome,
                    TrialOutcome::UnexpectedRefusal | TrialOutcome::ValidRefusal
                )
        })
        .count() as u64;
    let false_acceptances = episodes
        .iter()
        .filter(|episode| {
            episode.expectation == TrialExpectation::ExpectedRefusal
                && matches!(
                    episode.outcome,
                    TrialOutcome::FalseAcceptance | TrialOutcome::Success
                )
        })
        .count() as u64;
    let unauthorized_writes = episodes
        .iter()
        .map(|episode| episode.unauthorized_writes)
        .sum();
    let geometry_omissions = episodes
        .iter()
        .filter(|episode| episode.geometry_digest.trim().is_empty())
        .count() as u64;
    let stale_authority_continuations = episodes
        .iter()
        .map(|episode| {
            let reauthorization_missing = episode
                .actions
                .iter()
                .skip(1)
                .filter(|action| !action.reauthorized_after_observation)
                .count() as u64;
            let continued_after_divergence = episode
                .actions
                .iter()
                .enumerate()
                .filter(|(index, action)| {
                    action.runtime_divergence && *index + 1 < episode.actions.len()
                })
                .count() as u64;
            reauthorization_missing + continued_after_divergence
        })
        .sum();
    let recovery_opportunities = episodes
        .iter()
        .filter(|episode| {
            episode.fault_injection.as_deref() == Some("runtime_divergence")
                || episode
                    .actions
                    .iter()
                    .any(|action| action.action_key.starts_with("probe:"))
        })
        .count() as u64;
    let recovery_trials = episodes
        .iter()
        .filter(|episode| {
            episode.recovery_count > 0
                && (episode.actions.windows(2).any(|pair| {
                    pair[0].action_key.starts_with("probe:")
                        && pair[1].reauthorized_after_observation
                }) || episode.actions.iter().enumerate().any(|(index, action)| {
                    action.runtime_divergence
                        && episode
                            .actions
                            .get(index + 1)
                            .is_some_and(|next| next.reauthorized_after_observation)
                }))
        })
        .count() as u64;
    let acquisition_trials = episodes
        .iter()
        .filter(|episode| episode.acquisition_verified.is_some())
        .count() as u64;
    let acquisitions = episodes
        .iter()
        .filter(|episode| episode.acquisition_verified == Some(true))
        .count() as u64;
    let hold_trials = episodes
        .iter()
        .filter(|episode| episode.hold_verified.is_some())
        .count() as u64;
    let holds = episodes
        .iter()
        .filter(|episode| episode.hold_verified == Some(true))
        .count() as u64;

    let mut wall_times = episodes
        .iter()
        .map(|episode| episode.wall_time_s)
        .filter(|time| time.is_finite() && *time >= 0.0)
        .collect::<Vec<_>>();
    wall_times.sort_by(f64::total_cmp);
    let simulation_time_total_s = episodes
        .iter()
        .map(|episode| episode.simulation_time_s)
        .filter(|time| time.is_finite() && *time >= 0.0)
        .sum();
    let mut failure_causes = BTreeMap::new();
    for cause in episodes
        .iter()
        .filter_map(|episode| episode.failure_cause.as_ref())
    {
        *failure_causes
            .entry(failure_cause_name(cause).to_string())
            .or_insert(0) += 1;
    }

    let mut grouped: BTreeMap<String, Vec<&BenchmarkEpisode>> = BTreeMap::new();
    for episode in &episodes {
        grouped
            .entry(slice_key(episode.role, &episode.robot_id, episode.task))
            .or_default()
            .push(episode);
    }
    let slices = grouped
        .into_iter()
        .map(|(key, slice)| (key, score_slice(&slice)))
        .collect();

    let campaign_complete = completed_trials == scheduled_trials
        && infrastructure_trials == 0
        && invalid_trials == 0
        && unaccounted_trials == 0;
    let safety_gate_passed = campaign_complete
        && infrastructure_trials == 0
        && invalid_trials == 0
        && false_acceptances == 0
        && unauthorized_writes == 0
        && geometry_omissions == 0
        && stale_authority_continuations == 0;

    Ok(BenchmarkReport {
        schema_version: config.schema_version,
        config: config.clone(),
        campaign_complete,
        safety_gate_passed,
        scheduled_trials,
        completed_trials,
        infrastructure_trials,
        invalid_trials,
        unaccounted_trials,
        missing_trial_ids,
        task_success: RateEstimate::new(task_success_n, expected_success),
        correct_refusal: RateEstimate::new(correct_refusal_n, expected_refusal),
        false_refusals,
        false_acceptances,
        unauthorized_writes,
        geometry_omissions,
        stale_authority_continuations,
        recovery_rate: RateEstimate::new(recovery_trials, recovery_opportunities),
        recovery_trials,
        acquisition: RateEstimate::new(acquisitions, acquisition_trials),
        verified_hold: RateEstimate::new(holds, hold_trials),
        wall_time_p50_s: percentile(&wall_times, 0.50),
        wall_time_p95_s: percentile(&wall_times, 0.95),
        simulation_time_total_s,
        failure_causes,
        slices,
        records: records.to_vec(),
        candidate_source_sha: None,
        candidate_worktree_dirty: false,
        baseline_source_sha: None,
        baseline_status: "not_run".into(),
    })
}

pub fn write_report(report: &BenchmarkReport, out_dir: &Path) -> Result<(), BenchmarkError> {
    fs::create_dir_all(out_dir).map_err(|error| {
        BenchmarkError::InvalidTrial(format!(
            "create report directory {}: {error}",
            out_dir.display()
        ))
    })?;
    let json = serde_json::to_vec_pretty(report)?;
    fs::write(out_dir.join("manipulation_benchmark.json"), json)
        .map_err(|error| BenchmarkError::InvalidTrial(format!("write benchmark JSON: {error}")))?;
    fs::write(
        out_dir.join("MANIPULATION_BENCHMARK.md"),
        render_markdown(report),
    )
    .map_err(|error| BenchmarkError::InvalidTrial(format!("write benchmark Markdown: {error}")))?;
    let traces = out_dir.join("trials");
    fs::create_dir_all(&traces).map_err(|error| {
        BenchmarkError::InvalidTrial(format!("create trace directory: {error}"))
    })?;
    for record in &report.records {
        let safe_id = record
            .trial_id
            .chars()
            .map(|character| {
                if character.is_ascii_alphanumeric() || character == '-' {
                    character
                } else {
                    '_'
                }
            })
            .collect::<String>();
        let bytes = serde_json::to_vec_pretty(record)?;
        fs::write(traces.join(format!("{safe_id}.json")), bytes).map_err(|error| {
            BenchmarkError::InvalidTrial(format!("write raw trial trace {safe_id}: {error}"))
        })?;
    }
    Ok(())
}

pub fn render_markdown(report: &BenchmarkReport) -> String {
    let mut output = format!(
        "# Manipulation benchmark\n\n- Schema: {}\n- Campaign complete: {}\n- Safety gate: {}\n- Scheduled / completed / infrastructure / invalid / unaccounted: {} / {} / {} / {} / {}\n- Task success: {}\n- Correct refusal: {}\n- False refusals / false acceptances: {} / {}\n- Unauthorized writes / geometry omissions / stale-authority continuations: {} / {} / {}\n- Recovery rate: {}\n- Acquisition / verified hold: {} / {}\n- Wall time p50 / p95: {:?} / {:?} s\n- Simulation time total: {:.3} s\n- Candidate source SHA / dirty worktree: {:?} / {}\n- Baseline source SHA/status: {:?} / {}\n",
        report.schema_version,
        report.campaign_complete,
        report.safety_gate_passed,
        report.scheduled_trials,
        report.completed_trials,
        report.infrastructure_trials,
        report.invalid_trials,
        report.unaccounted_trials,
        display_rate(report.task_success),
        display_rate(report.correct_refusal),
        report.false_refusals,
        report.false_acceptances,
        report.unauthorized_writes,
        report.geometry_omissions,
        report.stale_authority_continuations,
        display_rate(report.recovery_rate),
        display_rate(report.acquisition),
        display_rate(report.verified_hold),
        report.wall_time_p50_s,
        report.wall_time_p95_s,
        report.simulation_time_total_s,
        report.candidate_source_sha,
        report.candidate_worktree_dirty,
        report.baseline_source_sha,
        report.baseline_status,
    );
    if !report.failure_causes.is_empty() {
        output.push_str("\n## Failure causes\n\n");
        for (cause, count) in &report.failure_causes {
            output.push_str(&format!("- {cause}: {count}\n"));
        }
    }
    if !report.slices.is_empty() {
        output.push_str("\n## Embodiment and task slices\n\n");
        output.push_str("| Slice | Task success | Correct refusal | False refusals | False acceptances | Acquisition | Verified hold |\n|---|---:|---:|---:|---:|---:|---:|\n");
        for (key, slice) in &report.slices {
            output.push_str(&format!(
                "| {key} | {} | {} | {} | {} | {} | {} |\n",
                display_rate(slice.task_success),
                display_rate(slice.correct_refusal),
                slice.false_refusals,
                slice.false_acceptances,
                display_rate(slice.acquisition),
                display_rate(slice.verified_hold),
            ));
        }
    }
    if !report.missing_trial_ids.is_empty() {
        output.push_str("\n## Unaccounted scheduled trials\n\n");
        for id in &report.missing_trial_ids {
            output.push_str(&format!("- {id}\n"));
        }
    }
    output.push_str("\nEvery simulator episode is labeled SIMULATION_ONLY. A missing physical sensor trace does not establish transfer.\n");
    output
}

fn score_slice(episodes: &[&BenchmarkEpisode]) -> BenchmarkSliceReport {
    let success_denominator = episodes
        .iter()
        .filter(|episode| episode.expectation == TrialExpectation::ExpectedSuccess)
        .count() as u64;
    let success_numerator = episodes
        .iter()
        .filter(|episode| {
            episode.expectation == TrialExpectation::ExpectedSuccess
                && episode.outcome == TrialOutcome::Success
        })
        .count() as u64;
    let refusal_denominator = episodes
        .iter()
        .filter(|episode| episode.expectation == TrialExpectation::ExpectedRefusal)
        .count() as u64;
    let refusal_numerator = episodes
        .iter()
        .filter(|episode| {
            episode.expectation == TrialExpectation::ExpectedRefusal
                && episode.outcome == TrialOutcome::ValidRefusal
        })
        .count() as u64;
    let false_refusals = episodes
        .iter()
        .filter(|episode| {
            episode.expectation == TrialExpectation::ExpectedSuccess
                && matches!(
                    episode.outcome,
                    TrialOutcome::UnexpectedRefusal | TrialOutcome::ValidRefusal
                )
        })
        .count() as u64;
    let false_acceptances = episodes
        .iter()
        .filter(|episode| {
            episode.expectation == TrialExpectation::ExpectedRefusal
                && matches!(
                    episode.outcome,
                    TrialOutcome::FalseAcceptance | TrialOutcome::Success
                )
        })
        .count() as u64;
    let acquisition_trials = episodes
        .iter()
        .filter(|episode| episode.acquisition_verified.is_some())
        .count() as u64;
    let acquisitions = episodes
        .iter()
        .filter(|episode| episode.acquisition_verified == Some(true))
        .count() as u64;
    let hold_trials = episodes
        .iter()
        .filter(|episode| episode.hold_verified.is_some())
        .count() as u64;
    let holds = episodes
        .iter()
        .filter(|episode| episode.hold_verified == Some(true))
        .count() as u64;
    BenchmarkSliceReport {
        task_success: RateEstimate::new(success_numerator, success_denominator),
        correct_refusal: RateEstimate::new(refusal_numerator, refusal_denominator),
        false_refusals,
        false_acceptances,
        acquisition: RateEstimate::new(acquisitions, acquisition_trials),
        verified_hold: RateEstimate::new(holds, hold_trials),
    }
}

fn percentile(sorted: &[f64], quantile: f64) -> Option<f64> {
    if sorted.is_empty() {
        return None;
    }
    let rank = quantile.clamp(0.0, 1.0) * (sorted.len() - 1) as f64;
    let lower = rank.floor() as usize;
    let upper = rank.ceil() as usize;
    let fraction = rank - lower as f64;
    Some(sorted[lower] + (sorted[upper] - sorted[lower]) * fraction)
}

fn slice_key(role: EmbodimentRole, robot_id: &str, task: BenchmarkTask) -> String {
    format!("{}/{}/{}", role.as_str(), robot_id, task.as_str())
}

fn failure_cause_name(cause: &FailureCause) -> &'static str {
    match cause {
        FailureCause::UnsupportedCollisionGeometry => "unsupported_collision_geometry",
        FailureCause::UnsupportedTask => "unsupported_task",
        FailureCause::UnsupportedControlMapping => "unsupported_control_mapping",
        FailureCause::BridgeRefusal => "bridge_refusal",
        FailureCause::UnexpectedRefusal => "unexpected_refusal",
        FailureCause::FalseAcceptance => "false_acceptance",
        FailureCause::RuntimeDivergence => "runtime_divergence",
        FailureCause::GoalNotReached => "goal_not_reached",
        FailureCause::SimulatorNan => "simulator_nan",
        FailureCause::ActionEnvelopeMismatch => "action_envelope_mismatch",
        FailureCause::AcquisitionNotVerified => "acquisition_not_verified",
        FailureCause::HoldNotVerified => "hold_not_verified",
    }
}

fn display_rate(rate: RateEstimate) -> String {
    match (rate.rate, rate.confidence_95) {
        (Some(value), Some([lower, upper])) => format!(
            "{}/{} ({:.1}%, 95% Wilson {:.1}–{:.1}%)",
            rate.numerator,
            rate.denominator,
            value * 100.0,
            lower * 100.0,
            upper * 100.0
        ),
        _ => format!("{}/{} (n/a)", rate.numerator, rate.denominator),
    }
}
