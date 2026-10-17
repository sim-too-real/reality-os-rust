//! Reproducible manipulation campaign contract and execution support.

pub mod execution;
pub mod geometry;
pub mod physical_evidence;
pub mod report;
pub mod scenario;
pub mod wrench_sensor;

pub use execution::{
    run_benchmark_trial, ActionTrace, BenchmarkEpisode, FailureCause, TrialOutcome,
};
pub use geometry::{
    validate_collision_geometry, CollisionGeomRecord, CollisionGeometryManifest,
    GeometryGateResult, MeshAssetDigest,
};
pub use physical_evidence::{
    validate_physical_observation, PhysicalEvidenceStatus, PhysicalObservationEvidence,
    PhysicalSensorIdentity, PhysicalSensorRequirement, PhysicalSensorTrace, SensorOrigin,
};
pub use report::{
    evaluate_paired_screening_gate, paired_campaign_delta, render_markdown, score_campaign,
    write_report, BenchmarkExecutionError, BenchmarkReport, BenchmarkSliceReport,
    BenchmarkTrialRecord, PairedCampaignDelta, PairedEmbodimentDelta, PairedMetricDelta,
    PairedScreeningGate, RateEstimate, WrenchReportMetrics,
};
pub use scenario::{
    paired_trial_key, resolve_benchmark_trial, trial_digest, upgrade_v5_trial_for_v6,
    BenchmarkConfig, BenchmarkTask, EmbodimentRegistration, EmbodimentRegistry, EmbodimentRole,
    ResolvedBenchmarkTrial, TrialExpectation, BENCHMARK_SCHEMA_VERSION,
};
pub use wrench_sensor::{SimulationWrenchSensorProfile, SimulationWrenchWindow};

use thiserror::Error;

#[derive(Debug, Error)]
pub enum BenchmarkError {
    #[error("invalid benchmark manifest: {0}")]
    InvalidManifest(String),
    #[error("invalid benchmark trial: {0}")]
    InvalidTrial(String),
    #[error("benchmark serialization failed: {0}")]
    Serialization(#[from] serde_json::Error),
    #[error("benchmark geometry rejected: {0}")]
    Geometry(String),
    #[error("benchmark simulator failed: {0}")]
    Simulation(String),
}
