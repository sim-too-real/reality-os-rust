//! THE one sanctioned crossing from ActuationCommand to plant.act().

use serde::{Deserialize, Serialize};

use crate::caps::ActionParams;
use crate::command::{ActuationCommand, ExecuteBind};
use crate::error::PlantError;
use crate::ledger::CommandLedger;
use crate::signing::signature_violations;
use crate::traits::Plant;
use crate::write_guard::with_certified_write;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecuteResult {
    pub ok: bool,
    pub executed: bool,
    pub violations: Vec<String>,
    pub command_id: String,
    pub realized: Option<crate::caps::PlantRealized>,
}

impl ExecuteResult {
    fn refused(command_id: &str, violations: Vec<String>) -> Self {
        Self {
            ok: false,
            executed: false,
            violations,
            command_id: command_id.into(),
            realized: None,
        }
    }
}

pub fn execute_certified_command(
    plant: &mut dyn Plant,
    command: &dyn ActuationCommand,
    params: &ActionParams,
    ledger: &mut CommandLedger,
    now_s: f64,
    bind: &ExecuteBind<'_>,
) -> ExecuteResult {
    let cid = command.command_id();
    if !command.certificate_status().allowed() {
        return ExecuteResult::refused(cid, vec!["command_certificate_not_executable".into()]);
    }
    if !command.acknowledged() {
        return ExecuteResult::refused(cid, vec!["command_not_acknowledged".into()]);
    }
    if bind.require_signature || bind.signing_key.is_some() {
        let sig = signature_violations(command, bind.signing_key, bind.require_signature);
        if !sig.is_empty() {
            return ExecuteResult::refused(cid, sig);
        }
    }
    let action = command.allowed_action();
    if action.is_empty() {
        return ExecuteResult::refused(cid, vec!["missing_allowed_action".into()]);
    }
    if action.iter().any(|x| !x.is_finite()) {
        return ExecuteResult::refused(cid, vec!["non_finite_allowed_action".into()]);
    }
    let violations = ledger.check(
        command,
        now_s,
        bind.expected_release_hash,
        bind.expected_calibration_ids,
        bind.expected_sensor_packet_hash,
        bind.require_sensor_packet_hash,
        bind.require_monotonic_sequence,
    );
    if !violations.is_empty() {
        return ExecuteResult::refused(cid, violations);
    }
    if let Err(e) = ledger.consume(command) {
        return ExecuteResult::refused(cid, vec![e.to_string()]);
    }

    let write = with_certified_write(|| {
        if let Some(wps) = command.follow_waypoints() {
            plant.follow_waypoints(wps)
        } else {
            plant.act(action, params)
        }
    });
    match write {
        Ok(realized) => ExecuteResult {
            ok: true,
            executed: true,
            violations: Vec::new(),
            command_id: cid.into(),
            realized: Some(realized),
        },
        Err(PlantError::EstopEngaged) => ExecuteResult::refused(cid, vec!["estop_engaged".into()]),
        Err(e) => ExecuteResult::refused(cid, vec![e.to_string()]),
    }
}
