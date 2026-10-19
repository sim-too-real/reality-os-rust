//! Auditable campaign accounting and reliability metrics.

use super::execution::{BenchmarkEpisode, FailureCause, TrialOutcome};
use super::scenario::{
    paired_trial_key, BenchmarkConfig, BenchmarkTask, EmbodimentRole, ResolvedBenchmarkTrial,
    TrialExpectation, BENCHMARK_SCHEMA_VERSION,
};
use super::wrench_sensor::WrenchAbortReason;
use super::BenchmarkError;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::Path;

const WILSON_Z_95: f64 = 1.959_963_984_540_054;
const PAIRED_BOOTSTRAP_RESAMPLES: u64 = 10_000;

#[derive(Debug, Clone, Copy)]
struct PairedAvailabilityObservation {
    baseline_success: bool,
    candidate_success: bool,
    baseline_information: bool,
    candidate_information: bool,
    baseline_followup: bool,
    candidate_followup: bool,
}

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

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct WrenchReportMetrics {
    pub push_trials: u64,
    pub episodes_with_samples: u64,
    pub sample_count: u64,
    pub strict_friction_narrowings: u64,
    pub inference_refusals: u64,
    pub force_cap_trips: u64,
    pub stale_or_missing_sample_aborts: u64,
    pub unexpected_contact_aborts: u64,
    pub actuator_saturation_aborts: u64,
    pub inference_rate: RateEstimate,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PairedMetricDelta {
    pub baseline: RateEstimate,
    pub candidate: RateEstimate,
    pub delta: Option<f64>,
    pub confidence_95: Option<[f64; 2]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PairedEmbodimentDelta {
    pub role: EmbodimentRole,
    pub robot_id: String,
    pub paired_expected_success_trials: u64,
    pub expected_success: PairedMetricDelta,
    pub information_bearing_probe: PairedMetricDelta,
    pub justified_followup: PairedMetricDelta,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PairedCampaignDelta {
    pub matched_trial_keys: u64,
    pub scenario_manifest_digest: String,
    pub bootstrap_resamples: u64,
    pub embodiments: BTreeMap<String, PairedEmbodimentDelta>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PairedScreeningGate {
    pub passed: bool,
    pub gain_at_least_10pp: bool,
    pub no_metric_regressed_more_than_5pp: bool,
    pub best_gain: Option<f64>,
    pub worst_regression: Option<f64>,
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
    #[serde(default)]
    pub evaluation_label_mismatches: u64,
    pub unauthorized_writes: u64,
    pub geometry_omissions: u64,
    pub stale_authority_continuations: u64,
    #[serde(default)]
    pub unverified_wrench_stops: u64,
    pub recovery_rate: RateEstimate,
    pub recovery_trials: u64,
    pub acquisition: RateEstimate,
    pub verified_hold: RateEstimate,
    pub wall_time_p50_s: Option<f64>,
    pub wall_time_p95_s: Option<f64>,
    pub simulation_time_total_s: f64,
    pub failure_causes: BTreeMap<String, u64>,
    pub slices: BTreeMap<String, BenchmarkSliceReport>,
    #[serde(default)]
    pub wrench_metrics: WrenchReportMetrics,
    #[serde(default)]
    pub paired_delta: Option<PairedCampaignDelta>,
    #[serde(default)]
    pub paired_screening_gate: Option<PairedScreeningGate>,
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
    if !matches!(config.schema_version, 5 | 6 | BENCHMARK_SCHEMA_VERSION)
        || config.scheduled_trial_ids.is_empty()
    {
        return Err(BenchmarkError::InvalidManifest(
            "campaign scoring supports schema-v5/v6 history or schema-v7 execution and requires a non-empty scheduled_trial_ids manifest".into(),
        ));
    }
    if !config.target_ci_half_width.is_finite()
        || !(0.0..=1.0).contains(&config.target_ci_half_width)
    {
        return Err(BenchmarkError::InvalidManifest(
            "target confidence-interval half-width must be finite and within [0,1]".into(),
        ));
    }
    if config.schema_version == BENCHMARK_SCHEMA_VERSION
        && config.scenario_suite_sha256.is_some()
        && (config
            .scenario_suite_id
            .as_deref()
            .map_or(true, str::is_empty)
            || config.scenario_suite_version.is_none())
    {
        return Err(BenchmarkError::InvalidManifest(
            "scenario suite digest requires a suite identifier and version".into(),
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
        if let Some(trial) = record.trial.as_ref() {
            if trial.trial_id != record.trial_id {
                return Err(BenchmarkError::InvalidTrial(format!(
                    "record and resolved trial identity mismatch for {id}"
                )));
            }
            if matches!(config.schema_version, 6 | BENCHMARK_SCHEMA_VERSION) {
                validate_paired_key(trial, config.schema_version)?;
            } else if trial.schema_version != config.schema_version {
                return Err(BenchmarkError::InvalidTrial(format!(
                    "schema-v{} trial cannot be scored by schema-v{} campaign",
                    trial.schema_version, config.schema_version
                )));
            }
            if config.schema_version == BENCHMARK_SCHEMA_VERSION
                && config.scenario_suite_sha256.is_some()
                && (trial.scenario_suite_id != config.scenario_suite_id
                    || trial.scenario_suite_version != config.scenario_suite_version
                    || (trial.task == BenchmarkTask::PushProbeReplan
                        && match trial.stratum {
                            Some(super::scenario::PushTrialStratum::ExpectedSuccess) => {
                                trial.expectation != TrialExpectation::ExpectedSuccess
                            }
                            Some(super::scenario::PushTrialStratum::ExpectedRefusal) => {
                                trial.expectation != TrialExpectation::ExpectedRefusal
                            }
                            None => true,
                        })
                    || (trial.task != BenchmarkTask::PushProbeReplan && trial.stratum.is_some()))
            {
                return Err(BenchmarkError::InvalidTrial(format!(
                    "schema-v{} trial {id} does not match the configured scenario suite and frozen stratum",
                    BENCHMARK_SCHEMA_VERSION
                )));
            }
            if matches!(config.schema_version, 6 | BENCHMARK_SCHEMA_VERSION)
                && record.episode.as_ref().is_some_and(|episode| {
                    episode.schema_version != config.schema_version
                        || episode.paired_trial_key != trial.paired_trial_key
                })
            {
                return Err(BenchmarkError::InvalidTrial(format!(
                    "schema-v{} episode {id} is missing or has a mismatched paired_trial_key",
                    config.schema_version
                )));
            }
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
    let evaluation_label_mismatches = episodes
        .iter()
        .filter(|episode| episode.outcome == TrialOutcome::EvaluationLabelMismatch)
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
            episode
                .actions
                .iter()
                .enumerate()
                .skip(1)
                .filter(|(index, action)| {
                    let previous = &episode.actions[*index - 1];
                    !action.reauthorized_after_observation
                        || (previous.runtime_divergence
                            && (action.grant_id == previous.grant_id
                                || action.observation_epoch == previous.observation_epoch))
                })
                .count() as u64
        })
        .sum();
    let unverified_wrench_stops = episodes
        .iter()
        .filter(|episode| {
            episode
                .wrench_abort
                .as_ref()
                .is_some_and(|abort| !abort.stop_verified)
                || episode.wrench_abort_result.as_deref()
                    == Some("preauthorized_hold_unverified_fault_latched")
        })
        .count() as u64;
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
    let wrench_metrics = score_wrench_metrics(&episodes);

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
        && stale_authority_continuations == 0
        && unverified_wrench_stops == 0;

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
        evaluation_label_mismatches,
        unauthorized_writes,
        geometry_omissions,
        stale_authority_continuations,
        unverified_wrench_stops,
        recovery_rate: RateEstimate::new(recovery_trials, recovery_opportunities),
        recovery_trials,
        acquisition: RateEstimate::new(acquisitions, acquisition_trials),
        verified_hold: RateEstimate::new(holds, hold_trials),
        wall_time_p50_s: percentile(&wall_times, 0.50),
        wall_time_p95_s: percentile(&wall_times, 0.95),
        simulation_time_total_s,
        failure_causes,
        slices,
        wrench_metrics,
        paired_delta: None,
        paired_screening_gate: None,
        records: records.to_vec(),
        candidate_source_sha: None,
        candidate_worktree_dirty: false,
        baseline_source_sha: None,
        baseline_status: "not_run".into(),
    })
}

/// Compare two campaigns only when every scheduled randomized input is paired
/// exactly once. Legacy schema-v5 trials derive the schema-independent key;
/// schema-v6/v7 records must carry and validate their persisted key.
pub fn paired_campaign_delta(
    baseline: &[BenchmarkTrialRecord],
    candidate: &[BenchmarkTrialRecord],
) -> Result<PairedCampaignDelta, BenchmarkError> {
    let baseline_by_key = index_paired_records(baseline, "baseline")?;
    let candidate_by_key = index_paired_records(candidate, "candidate")?;
    let baseline_keys = baseline_by_key.keys().cloned().collect::<BTreeSet<_>>();
    let candidate_keys = candidate_by_key.keys().cloned().collect::<BTreeSet<_>>();
    if baseline_keys != candidate_keys {
        let missing = baseline_keys
            .difference(&candidate_keys)
            .take(3)
            .cloned()
            .collect::<Vec<_>>();
        let extra = candidate_keys
            .difference(&baseline_keys)
            .take(3)
            .cloned()
            .collect::<Vec<_>>();
        return Err(BenchmarkError::InvalidTrial(format!(
            "paired campaigns must have identical unique randomized-input keys (baseline-only: {missing:?}, candidate-only: {extra:?})"
        )));
    }
    if baseline_keys.is_empty() {
        return Err(BenchmarkError::InvalidTrial(
            "paired campaigns must contain at least one resolved trial".into(),
        ));
    }

    let mut digest = Sha256::new();
    digest.update(b"realityos.manipulation.scenario_manifest/1\0");
    for key in &baseline_keys {
        digest.update((key.len() as u64).to_le_bytes());
        digest.update(key.as_bytes());
    }
    let scenario_manifest_digest = hex::encode(digest.finalize());

    let mut grouped: BTreeMap<String, Vec<PairedAvailabilityObservation>> = BTreeMap::new();
    let mut embodiment_identity = BTreeMap::new();
    for key in &baseline_keys {
        let base_record = baseline_by_key[key];
        let candidate_record = candidate_by_key[key];
        let trial = base_record.trial.as_ref().expect("indexed resolved trial");
        if trial.task != BenchmarkTask::PushProbeReplan
            || trial.expectation != TrialExpectation::ExpectedSuccess
        {
            continue;
        }
        let embodiment_key = format!("{}:{}", trial.role.as_str(), trial.robot_id);
        embodiment_identity.insert(embodiment_key.clone(), (trial.role, trial.robot_id.clone()));
        let baseline_episode = base_record.episode.as_ref();
        let candidate_episode = candidate_record.episode.as_ref();
        let base_success =
            baseline_episode.is_some_and(|episode| episode.outcome == TrialOutcome::Success);
        let candidate_success =
            candidate_episode.is_some_and(|episode| episode.outcome == TrialOutcome::Success);
        let base_information = baseline_episode.is_some_and(information_bearing_probe);
        let candidate_information = candidate_episode.is_some_and(information_bearing_probe);
        let base_followup = baseline_episode.is_some_and(justified_probe_followup);
        let candidate_followup = candidate_episode.is_some_and(justified_probe_followup);
        grouped
            .entry(embodiment_key)
            .or_default()
            .push(PairedAvailabilityObservation {
                baseline_success: base_success,
                candidate_success,
                baseline_information: base_information,
                candidate_information,
                baseline_followup: base_followup,
                candidate_followup,
            });
    }

    let mut embodiments = BTreeMap::new();
    for (key, pairs) in grouped {
        let (role, robot_id) = embodiment_identity
            .remove(&key)
            .expect("group has an embodiment identity");
        let baseline_success = pairs
            .iter()
            .map(|pair| pair.baseline_success)
            .collect::<Vec<_>>();
        let candidate_success = pairs
            .iter()
            .map(|pair| pair.candidate_success)
            .collect::<Vec<_>>();
        let baseline_information = pairs
            .iter()
            .map(|pair| pair.baseline_information)
            .collect::<Vec<_>>();
        let candidate_information = pairs
            .iter()
            .map(|pair| pair.candidate_information)
            .collect::<Vec<_>>();
        let baseline_followup = pairs
            .iter()
            .map(|pair| pair.baseline_followup)
            .collect::<Vec<_>>();
        let candidate_followup = pairs
            .iter()
            .map(|pair| pair.candidate_followup)
            .collect::<Vec<_>>();
        let denominator = pairs.len() as u64;
        embodiments.insert(
            key.clone(),
            PairedEmbodimentDelta {
                role,
                robot_id,
                paired_expected_success_trials: denominator,
                expected_success: paired_metric_delta(
                    &baseline_success,
                    &candidate_success,
                    &scenario_manifest_digest,
                    &key,
                    "expected_success",
                ),
                information_bearing_probe: paired_metric_delta(
                    &baseline_information,
                    &candidate_information,
                    &scenario_manifest_digest,
                    &key,
                    "information_bearing_probe",
                ),
                justified_followup: paired_metric_delta(
                    &baseline_followup,
                    &candidate_followup,
                    &scenario_manifest_digest,
                    &key,
                    "justified_followup",
                ),
            },
        );
    }

    Ok(PairedCampaignDelta {
        matched_trial_keys: baseline_keys.len() as u64,
        scenario_manifest_digest,
        bootstrap_resamples: PAIRED_BOOTSTRAP_RESAMPLES,
        embodiments,
    })
}

pub fn evaluate_paired_screening_gate(delta: &PairedCampaignDelta) -> PairedScreeningGate {
    let mut deltas = Vec::new();
    for embodiment in delta.embodiments.values() {
        deltas.extend([
            embodiment.expected_success.delta,
            embodiment.information_bearing_probe.delta,
            embodiment.justified_followup.delta,
        ]);
    }
    let available = deltas.into_iter().flatten().collect::<Vec<_>>();
    let best_gain = available.iter().copied().max_by(f64::total_cmp);
    let worst_regression = available.iter().copied().min_by(f64::total_cmp);
    let gain_at_least_10pp = best_gain.is_some_and(|gain| gain >= 0.10);
    let no_metric_regressed_more_than_5pp =
        worst_regression.is_some_and(|regression| regression >= -0.05);
    PairedScreeningGate {
        passed: gain_at_least_10pp && no_metric_regressed_more_than_5pp,
        gain_at_least_10pp,
        no_metric_regressed_more_than_5pp,
        best_gain,
        worst_regression,
    }
}

fn index_paired_records<'a>(
    records: &'a [BenchmarkTrialRecord],
    label: &str,
) -> Result<BTreeMap<String, &'a BenchmarkTrialRecord>, BenchmarkError> {
    let mut indexed = BTreeMap::new();
    for record in records {
        let trial = record.trial.as_ref().ok_or_else(|| {
            BenchmarkError::InvalidTrial(format!(
                "{label} paired campaign record {} has no resolved trial",
                record.trial_id
            ))
        })?;
        if record.trial_id != trial.trial_id {
            return Err(BenchmarkError::InvalidTrial(format!(
                "{label} paired campaign record {} has mismatched trial identity",
                record.trial_id
            )));
        }
        if record.expectation != Some(trial.expectation) {
            return Err(BenchmarkError::InvalidTrial(format!(
                "{label} paired campaign record {} has mismatched expectation",
                record.trial_id
            )));
        }
        match (&record.episode, &record.error) {
            (Some(episode), None)
                if episode.trial_id == trial.trial_id
                    && episode.expectation == trial.expectation
                    && episode.task == trial.task
                    && episode.role == trial.role
                    && episode.robot_id == trial.robot_id
                    && episode.schema_version == trial.schema_version
                    && (trial.schema_version == 5
                        || episode.paired_trial_key == trial.paired_trial_key) => {}
            (None, Some(_)) => {}
            _ => {
                return Err(BenchmarkError::InvalidTrial(format!(
                    "{label} paired campaign record {} has an invalid episode/error pairing",
                    record.trial_id
                )))
            }
        }
        let key = match trial.schema_version {
            5 => paired_trial_key(trial)?,
            6 | BENCHMARK_SCHEMA_VERSION => {
                validate_paired_key(trial, trial.schema_version)?;
                trial
                    .paired_trial_key
                    .clone()
                    .expect("validated paired key")
            }
            schema => {
                return Err(BenchmarkError::InvalidTrial(format!(
                    "paired comparison does not support schema-v{schema} trials"
                )))
            }
        };
        if indexed.insert(key.clone(), record).is_some() {
            return Err(BenchmarkError::InvalidTrial(format!(
                "{label} paired campaign contains duplicate randomized-input key {key}"
            )));
        }
    }
    Ok(indexed)
}

fn validate_paired_key(
    trial: &ResolvedBenchmarkTrial,
    expected_schema_version: u32,
) -> Result<(), BenchmarkError> {
    if trial.schema_version != expected_schema_version {
        return Err(BenchmarkError::InvalidTrial(format!(
            "schema-v{} trial cannot be used as schema-v{} execution input",
            trial.schema_version, expected_schema_version
        )));
    }
    let computed = paired_trial_key(trial)?;
    if trial.paired_trial_key.as_deref() != Some(computed.as_str()) {
        return Err(BenchmarkError::InvalidTrial(format!(
            "schema-v{} trial is missing or has a mismatched paired_trial_key",
            expected_schema_version
        )));
    }
    Ok(())
}

fn score_wrench_metrics(episodes: &[&BenchmarkEpisode]) -> WrenchReportMetrics {
    let push_episodes = episodes
        .iter()
        .filter(|episode| episode.task == BenchmarkTask::PushProbeReplan)
        .copied()
        .collect::<Vec<_>>();
    let episodes_with_samples = push_episodes
        .iter()
        .filter(|episode| episode.wrench_sample_count > 0)
        .count() as u64;
    let strict_friction_narrowings = push_episodes
        .iter()
        .filter(|episode| information_bearing_probe(episode))
        .count() as u64;
    let inference_refusals = push_episodes
        .iter()
        .filter(|episode| {
            episode
                .wrench_inference_reason
                .as_deref()
                .is_some_and(|reason| reason.starts_with("no_inference:"))
        })
        .count() as u64;
    let force_cap_trips = push_episodes
        .iter()
        .filter(|episode| {
            episode
                .wrench_abort
                .as_ref()
                .is_some_and(|abort| matches!(abort.reason, WrenchAbortReason::Limit { .. }))
        })
        .count() as u64;
    let stale_or_missing_sample_aborts = push_episodes
        .iter()
        .filter(|episode| {
            episode.wrench_abort.as_ref().is_some_and(|abort| {
                matches!(
                    abort.reason,
                    WrenchAbortReason::MissingSample
                        | WrenchAbortReason::StaleSample
                        | WrenchAbortReason::GapExceeded
                        | WrenchAbortReason::DigestMismatch
                )
            })
        })
        .count() as u64;
    let unexpected_contact_aborts = push_episodes
        .iter()
        .filter(|episode| {
            episode
                .wrench_abort
                .as_ref()
                .is_some_and(|abort| matches!(abort.reason, WrenchAbortReason::UnexpectedContact))
        })
        .count() as u64;
    let actuator_saturation_aborts = push_episodes
        .iter()
        .filter(|episode| {
            episode.wrench_abort.as_ref().is_some_and(|abort| {
                matches!(abort.reason, WrenchAbortReason::ActuatorSaturation { .. })
            })
        })
        .count() as u64;
    WrenchReportMetrics {
        push_trials: push_episodes.len() as u64,
        episodes_with_samples,
        sample_count: push_episodes
            .iter()
            .map(|episode| episode.wrench_sample_count)
            .sum(),
        strict_friction_narrowings,
        inference_refusals,
        force_cap_trips,
        stale_or_missing_sample_aborts,
        unexpected_contact_aborts,
        actuator_saturation_aborts,
        inference_rate: RateEstimate::new(strict_friction_narrowings, episodes_with_samples),
    }
}

fn information_bearing_probe(episode: &BenchmarkEpisode) -> bool {
    if !episode
        .wrench_inference_reason
        .as_deref()
        .is_some_and(|reason| reason.starts_with("updated:"))
        || episode.wrench_sample_count == 0
        || episode.wrench_abort.is_some()
        || episode
            .wrench_sensor_profile_digest
            .as_deref()
            .map_or(true, str::is_empty)
        || episode
            .wrench_compensation_digest
            .as_deref()
            .map_or(true, str::is_empty)
    {
        return false;
    }
    let (Some([prior_low, prior_high]), Some([post_low, post_high])) = (
        episode.prior_support_friction_interval,
        episode.posterior_support_friction_interval,
    ) else {
        return false;
    };
    [prior_low, prior_high, post_low, post_high]
        .iter()
        .all(|value| value.is_finite())
        && episode
            .wrench_inference_interval
            .is_some_and(|interval| interval == [post_low, post_high])
        && prior_low <= post_low
        && post_low <= post_high
        && post_high <= prior_high
        && (post_high - post_low) < (prior_high - prior_low)
}

fn justified_probe_followup(episode: &BenchmarkEpisode) -> bool {
    information_bearing_probe(episode)
        && episode.actions.windows(2).any(|pair| {
            pair[0].action_key.starts_with("probe:")
                && pair[1].action_key.starts_with("push:")
                && pair[1].reauthorized_after_observation
                && pair[0].grant_id != pair[1].grant_id
                && pair[0].observation_epoch != pair[1].observation_epoch
        })
}

fn paired_metric_delta(
    baseline: &[bool],
    candidate: &[bool],
    manifest_digest: &str,
    embodiment_key: &str,
    metric_name: &str,
) -> PairedMetricDelta {
    debug_assert_eq!(baseline.len(), candidate.len());
    let differences = baseline
        .iter()
        .zip(candidate)
        .map(|(before, after)| i8::from(*after) - i8::from(*before))
        .collect::<Vec<_>>();
    let denominator = differences.len() as u64;
    let baseline_rate = RateEstimate::new(
        baseline.iter().filter(|value| **value).count() as u64,
        denominator,
    );
    let candidate_rate = RateEstimate::new(
        candidate.iter().filter(|value| **value).count() as u64,
        denominator,
    );
    if differences.is_empty() {
        return PairedMetricDelta {
            baseline: baseline_rate,
            candidate: candidate_rate,
            delta: None,
            confidence_95: None,
        };
    }
    let delta = differences
        .iter()
        .map(|value| f64::from(*value))
        .sum::<f64>()
        / differences.len() as f64;
    let mut seed_hasher = Sha256::new();
    seed_hasher.update(b"realityos.manipulation.paired_bootstrap/1\0");
    seed_hasher.update(manifest_digest.as_bytes());
    seed_hasher.update(embodiment_key.as_bytes());
    seed_hasher.update(metric_name.as_bytes());
    let seed_bytes = seed_hasher.finalize();
    let mut seed = [0u8; 8];
    seed.copy_from_slice(&seed_bytes[..8]);
    let mut rng = StdRng::seed_from_u64(u64::from_le_bytes(seed));
    let n = differences.len();
    let mut resampled = Vec::with_capacity(PAIRED_BOOTSTRAP_RESAMPLES as usize);
    for _ in 0..PAIRED_BOOTSTRAP_RESAMPLES {
        let mut sum = 0i64;
        for _ in 0..n {
            sum += i64::from(differences[rng.gen_range(0..n)]);
        }
        resampled.push(sum as f64 / n as f64);
    }
    resampled.sort_by(f64::total_cmp);
    PairedMetricDelta {
        baseline: baseline_rate,
        candidate: candidate_rate,
        delta: Some(delta),
        confidence_95: Some([
            percentile(&resampled, 0.025).unwrap_or(delta),
            percentile(&resampled, 0.975).unwrap_or(delta),
        ]),
    }
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
        "# Manipulation benchmark\n\n- Schema: {}\n- Campaign complete: {}\n- Safety gate: {}\n- Scheduled / completed / infrastructure / invalid / unaccounted: {} / {} / {} / {} / {}\n- Task success: {}\n- Correct refusal: {}\n- False refusals / false acceptances / evaluation-label mismatches: {} / {} / {}\n- Unauthorized writes / geometry omissions / stale-authority continuations / unverified wrench stops: {} / {} / {} / {}\n- Recovery rate: {}\n- Acquisition / verified hold: {} / {}\n- Wall time p50 / p95: {:?} / {:?} s\n- Simulation time total: {:.3} s\n- Candidate source SHA / dirty worktree: {:?} / {}\n- Baseline source SHA/status: {:?} / {}\n",
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
        report.evaluation_label_mismatches,
        report.unauthorized_writes,
        report.geometry_omissions,
        report.stale_authority_continuations,
        report.unverified_wrench_stops,
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
    output.push_str(&format!(
        "\n## Force/torque evidence\n\n- Push episodes / episodes with samples / samples: {} / {} / {}\n- Strict friction narrowings / inference refusals: {} / {}\n- Force-cap trips / stale-or-missing-sample aborts: {} / {}\n- Unexpected-contact / actuator-saturation aborts: {} / {}\n- Strict narrowing rate among sampled episodes: {}\n",
        report.wrench_metrics.push_trials,
        report.wrench_metrics.episodes_with_samples,
        report.wrench_metrics.sample_count,
        report.wrench_metrics.strict_friction_narrowings,
        report.wrench_metrics.inference_refusals,
        report.wrench_metrics.force_cap_trips,
        report.wrench_metrics.stale_or_missing_sample_aborts,
        report.wrench_metrics.unexpected_contact_aborts,
        report.wrench_metrics.actuator_saturation_aborts,
        display_rate(report.wrench_metrics.inference_rate),
    ));
    if let Some(delta) = &report.paired_delta {
        output.push_str(&format!(
            "\n## Paired campaign delta\n\n- Matched randomized-input keys: {}\n- Scenario manifest digest: `{}`\n- Paired bootstrap resamples: {}\n",
            delta.matched_trial_keys,
            delta.scenario_manifest_digest,
            delta.bootstrap_resamples,
        ));
        output.push_str("\n| Embodiment | Expected success | Information-bearing probe | Justified follow-up |\n|---|---|---|---|\n");
        for (key, embodiment) in &delta.embodiments {
            output.push_str(&format!(
                "| {key} (n={}) | {} | {} | {} |\n",
                embodiment.paired_expected_success_trials,
                display_paired_delta(&embodiment.expected_success),
                display_paired_delta(&embodiment.information_bearing_probe),
                display_paired_delta(&embodiment.justified_followup),
            ));
        }
        if let Some(gate) = &report.paired_screening_gate {
            output.push_str(&format!(
                "\nScreening gate: **{}**; gain at least 10 pp: {}; no metric regressed more than 5 pp: {}; best gain: {}; worst regression: {}.\n",
                if gate.passed { "PASS" } else { "FAIL" },
                gate.gain_at_least_10pp,
                gate.no_metric_regressed_more_than_5pp,
                display_optional_delta(gate.best_gain),
                display_optional_delta(gate.worst_regression),
            ));
        }
    }
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
        FailureCause::EvaluationLabelMismatch => "evaluation_label_mismatch",
        FailureCause::RuntimeDivergence => "runtime_divergence",
        FailureCause::GoalNotReached => "goal_not_reached",
        FailureCause::SimulatorNan => "simulator_nan",
        FailureCause::ActionEnvelopeMismatch => "action_envelope_mismatch",
        FailureCause::AcquisitionNotVerified => "acquisition_not_verified",
        FailureCause::HoldNotVerified => "hold_not_verified",
        FailureCause::ForceMonitorAbort => "force_monitor_abort",
        FailureCause::ForceBaselineUnavailable => "force_baseline_unavailable",
        FailureCause::AuthorityExpired => "authority_expired",
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

fn display_paired_delta(delta: &PairedMetricDelta) -> String {
    match (
        delta.baseline.rate,
        delta.candidate.rate,
        delta.delta,
        delta.confidence_95,
    ) {
        (Some(before), Some(after), Some(change), Some([lower, upper])) => format!(
            "{:.1}% → {:.1}% ({:+.1} pp, paired 95% bootstrap {:.1}–{:.1} pp)",
            before * 100.0,
            after * 100.0,
            change * 100.0,
            lower * 100.0,
            upper * 100.0,
        ),
        _ => format!(
            "{} → {} (n/a)",
            display_rate(delta.baseline),
            display_rate(delta.candidate)
        ),
    }
}

fn display_optional_delta(value: Option<f64>) -> String {
    value
        .map(|delta| format!("{:+.1} pp", delta * 100.0))
        .unwrap_or_else(|| "n/a".into())
}
