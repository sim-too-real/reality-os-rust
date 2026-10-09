//! One generated subsystem on the decide path.
//!
//! Simulation stays labeled simulation. A solver read-back is not a measurement.
//! This kernel does not write motors. A missing scene blocks at observation.
//! A scene that passes the observation gate still blocks at authorized execution.

use crate::plan::Intent;
use crate::see::evaluate_manip_observation_gate;
use realityos_kernel::{DecisionStatus, ObservationEvidence};

pub const BLOCKED_BY_SPECIFIC_DEPENDENCY: &str = "BLOCKED_BY_SPECIFIC_DEPENDENCY";

pub struct GeneratedSubsystem<'a> {
    pub duty_id: &'a str,
    pub phenotype: &'a str,
    pub simulation_pa: Option<f64>,
    pub simulation_label: &'a str,
    pub observation: Option<&'a ObservationEvidence>,
    pub solver_source: Option<&'a str>,
    pub now_s: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GeneratedArmRecord {
    pub duty_id: String,
    pub phenotype: String,
    pub simulation_pa: Option<f64>,
    pub simulation_label: String,
    pub observation_recorded: bool,
    pub authorized_execution: bool,
    pub measurement: Option<f64>,
    pub discrepancy: Option<f64>,
    pub blocked_step: String,
    pub outcome: String,
    pub cause: String,
}

pub fn run_generated_subsystem(
    input: &GeneratedSubsystem<'_>,
) -> Result<GeneratedArmRecord, String> {
    if input.duty_id.trim().is_empty() || input.phenotype.trim().is_empty() {
        return Err("generated subsystem needs a duty and a phenotype".into());
    }
    if input.simulation_pa.is_some() && input.simulation_label != "simulation" {
        return Err("a simulation value must be labeled simulation".into());
    }
    if let Some(source) = input.solver_source {
        if source == "solver" || source.contains("readback") || source.contains("read-back") {
            return Err("a solver read-back is not a Reality OS measurement".into());
        }
    }
    let intent = Intent::language("observe the generated subsystem before motion", "place");
    let gate = evaluate_manip_observation_gate(&intent, input.observation, None, 0.01, input.now_s);
    if !gate.ok {
        let blocked_step = if gate.status == DecisionStatus::Probe {
            "probing"
        } else {
            "observation"
        };
        return Ok(record(input, false, blocked_step, &gate.physical_reason));
    }
    Ok(record(
        input,
        true,
        "authorized_execution",
        "Reality OS does not write motors; no metal grant authorizes execution",
    ))
}

fn record(
    input: &GeneratedSubsystem<'_>,
    observation_recorded: bool,
    blocked_step: &str,
    cause: &str,
) -> GeneratedArmRecord {
    GeneratedArmRecord {
        duty_id: input.duty_id.into(),
        phenotype: input.phenotype.into(),
        simulation_pa: input.simulation_pa,
        simulation_label: input.simulation_label.into(),
        observation_recorded,
        authorized_execution: false,
        measurement: None,
        discrepancy: None,
        blocked_step: blocked_step.into(),
        outcome: BLOCKED_BY_SPECIFIC_DEPENDENCY.into(),
        cause: cause.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn arm<'a>(
        observation: Option<&'a ObservationEvidence>,
        solver_source: Option<&'a str>,
        label: &'a str,
    ) -> GeneratedSubsystem<'a> {
        GeneratedSubsystem {
            duty_id: "motor-driven-axis",
            phenotype: "field>armature>shaft",
            simulation_pa: Some(1.0e6),
            simulation_label: label,
            observation,
            solver_source,
            now_s: 1.0,
        }
    }

    #[test]
    fn missing_observation_blocks_before_a_measurement() {
        let record = run_generated_subsystem(&arm(None, None, "simulation")).unwrap();
        assert_eq!(record.outcome, BLOCKED_BY_SPECIFIC_DEPENDENCY);
        assert_eq!(record.blocked_step, "observation");
        assert!(record.measurement.is_none());
        assert!(record.discrepancy.is_none());
        assert!(!record.authorized_execution);
        assert!(!record.observation_recorded);
        assert_eq!(record.simulation_label, "simulation");
        assert_eq!(record.simulation_pa, Some(1.0e6));
    }

    #[test]
    fn a_scene_still_does_not_authorize_motion_or_store_a_measurement() {
        let observation = ObservationEvidence::new(
            "fixture-camera",
            "cal-1",
            0.0,
            0.0,
            "digest-1",
            "epoch-1",
            0.9,
            0.1,
            10.0,
        )
        .unwrap();
        let record = run_generated_subsystem(&arm(Some(&observation), None, "simulation")).unwrap();
        assert_eq!(record.blocked_step, "authorized_execution");
        assert!(record.observation_recorded);
        assert!(record.measurement.is_none());
        assert!(!record.authorized_execution);
        assert_eq!(record.outcome, BLOCKED_BY_SPECIFIC_DEPENDENCY);
    }

    #[test]
    fn solver_readback_is_not_stored() {
        let error = run_generated_subsystem(&arm(None, Some("solver"), "simulation")).unwrap_err();
        assert!(error.contains("not a Reality OS measurement"));
        let error =
            run_generated_subsystem(&arm(None, Some("solver_readback"), "simulation")).unwrap_err();
        assert!(error.contains("not a Reality OS measurement"));
    }

    #[test]
    fn an_unlabeled_simulation_is_refused() {
        let error = run_generated_subsystem(&arm(None, None, "solver")).unwrap_err();
        assert!(error.contains("labeled simulation"));
    }
}
