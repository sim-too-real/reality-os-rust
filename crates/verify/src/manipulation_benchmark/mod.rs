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
    render_markdown, score_campaign, write_report, BenchmarkExecutionError, BenchmarkReport,
    BenchmarkSliceReport, BenchmarkTrialRecord, RateEstimate,
};
pub use scenario::{
    resolve_benchmark_trial, trial_digest, BenchmarkConfig, BenchmarkTask, EmbodimentRegistration,
    EmbodimentRegistry, EmbodimentRole, ResolvedBenchmarkTrial, TrialExpectation,
    BENCHMARK_SCHEMA_VERSION,
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
