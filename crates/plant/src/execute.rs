//! THE one sanctioned crossing from ActuationCommand to plant.act().

use realityos_kernel::CommandOutcome;
use serde::{Deserialize, Serialize};

use crate::caps::{check_hard_action_bounds, ActionParams};
use crate::command::{ActuationCommand, ExecuteBind};
use crate::error::PlantError;
use crate::hil_faults;
use crate::ledger::CommandLedger;
use crate::signing::{signature_violations, signing_key_hash};
use crate::traits::Plant;
use crate::write_guard::with_certified_write;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecuteResult {
    pub ok: bool,
    pub executed: bool,
    pub outcome: CommandOutcome,
    pub violations: Vec<String>,
    pub command_id: String,
    pub realized: Option<crate::caps::PlantRealized>,
}

impl ExecuteResult {
    fn refused(command_id: &str, violations: Vec<String>) -> Self {
        Self {
            ok: false,
            executed: false,
            outcome: CommandOutcome::Refused,
            violations,
            command_id: command_id.into(),
            realized: None,
        }
    }

    fn unknown(command_id: &str, violations: Vec<String>) -> Self {
        Self {
            ok: false,
            executed: true,
            outcome: CommandOutcome::Unknown,
            violations,
            command_id: command_id.into(),
            realized: None,
        }
    }
}

fn online_rails(plant: &dyn Plant, bind: &ExecuteBind<'_>) -> bool {
    plant.production_locked() || bind.force_online_rails
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
    let rails = online_rails(plant, bind);
    if !command.certificate_status().allowed() {
        return ExecuteResult::refused(cid, vec!["command_certificate_not_executable".into()]);
    }
    if !command.acknowledged() {
        return ExecuteResult::refused(cid, vec!["command_not_acknowledged".into()]);
    }
    if rails && command.follow_waypoints().is_some() {
        return ExecuteResult::refused(cid, vec!["online_refuses_waypoint_shortcut".into()]);
    }
    if rails && command.actuator_ids().is_empty() {
        return ExecuteResult::refused(cid, vec!["online_requires_actuator_ids".into()]);
    }
    if rails && command.release_hash().is_empty() {
        return ExecuteResult::refused(cid, vec!["online_requires_release_hash".into()]);
    }
    if let Some(expect) = plant.production_key_hash() {
        match bind.signing_key {
            None => {
                return ExecuteResult::refused(cid, vec!["production_signing_key_missing".into()]);
            }
            Some(k) if signing_key_hash(k) != expect => {
                return ExecuteResult::refused(cid, vec!["production_signing_key_mismatch".into()]);
            }
            Some(_) => {}
        }
    }
    let require_signature = rails || bind.require_signature || bind.signing_key.is_some();
    if require_signature {
        let sig = signature_violations(command, bind.signing_key, true);
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
    if rails {
        if let Err(e) = check_hard_action_bounds(action, &plant.caps()) {
            return ExecuteResult::refused(cid, vec![e.to_string()]);
        }
    }
    let require_sensor = rails || bind.require_sensor_packet_hash;
    let require_seq = rails || bind.require_monotonic_sequence;
    let violations = ledger.check(
        command,
        now_s,
        bind.expected_release_hash,
        bind.expected_calibration_ids,
        bind.expected_sensor_packet_hash,
        require_sensor,
        require_seq,
    );
    if !violations.is_empty() {
        return ExecuteResult::refused(cid, violations);
    }
    hil_faults::crash_if("before_prepare");
    if let Err(e) = ledger.prepare(command) {
        return ExecuteResult::refused(cid, vec![e.to_string()]);
    }
    hil_faults::crash_if("after_prepare_before_write");

    let write = with_certified_write(|| {
        if let Some(wps) = command.follow_waypoints() {
            plant.follow_waypoints(wps)
        } else {
            plant.act(action, params)
        }
    });
    match write {
        Ok(realized) => {
            hil_faults::crash_if("after_write_before_ack");
            match ledger.ack(command) {
                Ok(_) => {
                    hil_faults::crash_if("after_ack");
                    ExecuteResult {
                        ok: true,
                        executed: true,
                        outcome: CommandOutcome::Executed,
                        violations: Vec::new(),
                        command_id: cid.into(),
                        realized: Some(realized),
                    }
                }
                Err(e) => {
                    let _ = ledger.mark_unknown(command);
                    ExecuteResult::unknown(cid, vec![e.to_string()])
                }
            }
        }
        Err(PlantError::EstopEngaged) => {
            let _ = ledger.mark_unknown(command);
            ExecuteResult::unknown(cid, vec!["estop_engaged".into()])
        }
        Err(e) => {
            let _ = ledger.mark_unknown(command);
            ExecuteResult::unknown(cid, vec![e.to_string()])
        }
    }
}
