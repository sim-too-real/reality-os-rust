//! RuntimeGovernor — fail-closed actuation boundary.
//!
//! Does not replace RealityOS.decide. Does not invent. Both must cross here
//! to touch a plant.

use realityos_plant::{
    execute_certified_command, ActionParams, ActuationCommand, CommandLedger, ExecuteBind, Plant,
};
use serde_json::{json, Map, Value};

use crate::envelope::DriverEnvelopePack;
use crate::identity::RuntimeIdentity;
use crate::latch::EstopLatch;
use crate::trace::RuntimeTrace;

const LATCHING_PREFIXES: &[&str] = &[
    "missing command_id",
    "replayed command_id",
    "command expired",
    "release_hash",
    "calibration_ids",
    "sensor_packet_hash",
    "command missing sensor_packet_hash",
    "missing expected_sensor_packet_hash",
    "missing_allowed_action",
    "non_numeric_allowed_action",
    "non_finite_allowed_action",
];

#[derive(Debug, Clone)]
pub struct GovernorConfig {
    pub heartbeat_period_s: f64,
    pub sensor_stale_s: f64,
    pub require_sensor_before_write: bool,
    pub require_online_identity: bool,
    pub require_command_signature: bool,
    pub require_monotonic_sequence: bool,
    pub require_sensor_packet_hash: bool,
    pub repeated_refuse_n: u32,
}

impl Default for GovernorConfig {
    fn default() -> Self {
        Self {
            heartbeat_period_s: 1.0,
            sensor_stale_s: 2.0,
            require_sensor_before_write: true,
            require_online_identity: false,
            require_command_signature: false,
            require_monotonic_sequence: false,
            require_sensor_packet_hash: false,
            repeated_refuse_n: 3,
        }
    }
}

pub struct RuntimeGovernor<P: Plant> {
    pub identity: RuntimeIdentity,
    plant: P,
    ledger: CommandLedger,
    config: GovernorConfig,
    envelope: Option<DriverEnvelopePack>,
    last_heartbeat_s: f64,
    last_sensor_s: f64,
    expected_sensor_packet_hash: Option<String>,
    signing_key: Option<Vec<u8>>,
    latch: EstopLatch,
    traces: Vec<RuntimeTrace>,
    watchdog_period_s: f64,
    last_watchdog_s: f64,
}

impl<P: Plant> RuntimeGovernor<P> {
    pub fn new(identity: RuntimeIdentity, plant: P) -> Self {
        Self {
            identity,
            plant,
            ledger: CommandLedger::new(),
            config: GovernorConfig::default(),
            envelope: None,
            last_heartbeat_s: 0.0,
            last_sensor_s: 0.0,
            expected_sensor_packet_hash: None,
            signing_key: None,
            latch: EstopLatch::default(),
            traces: Vec::new(),
            watchdog_period_s: 0.05,
            last_watchdog_s: 0.0,
        }
    }

    pub fn plant(&self) -> &P {
        &self.plant
    }

    pub fn plant_mut(&mut self) -> &mut P {
        &mut self.plant
    }

    pub fn ledger(&self) -> &CommandLedger {
        &self.ledger
    }

    pub fn ledger_mut(&mut self) -> &mut CommandLedger {
        &mut self.ledger
    }

    pub fn config(&self) -> &GovernorConfig {
        &self.config
    }

    pub fn config_mut(&mut self) -> &mut GovernorConfig {
        &mut self.config
    }

    pub fn envelope(&self) -> Option<&DriverEnvelopePack> {
        self.envelope.as_ref()
    }

    pub fn envelope_mut(&mut self) -> Option<&mut DriverEnvelopePack> {
        self.envelope.as_mut()
    }

    pub fn set_envelope(&mut self, env: DriverEnvelopePack) {
        self.envelope = Some(env);
    }

    pub fn set_signing_key(&mut self, key: Option<Vec<u8>>) {
        self.signing_key = key;
    }

    pub fn last_heartbeat_s(&self) -> f64 {
        self.last_heartbeat_s
    }

    pub fn last_sensor_s(&self) -> f64 {
        self.last_sensor_s
    }

    /// Software supervisor. Not an independent hardware watchdog.
    pub fn watchdog_tick(&mut self, now_s: f64) -> RuntimeTrace {
        if !now_s.is_finite() {
            return self.engage_estop("watchdog_non_finite_time", 0.0);
        }
        if self.last_watchdog_s > 0.0
            && (now_s - self.last_watchdog_s) > self.watchdog_period_s * 2.0
        {
            return self.engage_estop("software_watchdog_miss", now_s);
        }
        self.last_watchdog_s = now_s;
        self.emit(RuntimeTrace::new(true, "watchdog_tick", now_s))
    }

    pub fn traces(&self) -> &[RuntimeTrace] {
        &self.traces
    }

    pub fn estop(&self) -> bool {
        self.latch.engaged
    }

    pub fn abort_latched(&self) -> bool {
        self.latch.abort_latched
    }

    fn emit(&mut self, t: RuntimeTrace) -> RuntimeTrace {
        let mut body = Map::new();
        body.insert("event".into(), json!(t.event.clone()));
        body.insert("ok".into(), json!(t.ok));
        body.insert("violations".into(), json!(t.violations.clone()));
        if let Some(id) = &t.command_id {
            body.insert("command_id".into(), json!(id));
        }
        body.insert(
            "identity".into(),
            json!({
                "release_hash": self.identity.release_hash.as_str(),
                "serial_or_as_built": self.identity.serial_str(),
                "firmware_id": self.identity.firmware_str(),
                "calibration_id": self.identity.calibration_id_str(),
            }),
        );
        let _ = self.ledger.append_event("governor_event", body);
        self.traces.push(t.clone());
        t
    }

    pub fn heartbeat(&mut self, now_s: f64) -> RuntimeTrace {
        self.last_heartbeat_s = now_s;
        self.emit(RuntimeTrace::new(true, "heartbeat", now_s))
    }

    pub fn mark_sensor(&mut self, now_s: f64, content_hash: Option<String>) {
        self.last_sensor_s = now_s;
        if let Some(h) = content_hash {
            self.expected_sensor_packet_hash = Some(h);
        }
    }

    pub fn engage_estop(&mut self, reason: impl Into<String>, now_s: f64) -> RuntimeTrace {
        let reason = reason.into();
        self.latch.engage(&reason);
        self.plant.engage_estop(&reason);
        self.emit(RuntimeTrace::new(false, "estop", now_s).with_violations(vec![reason]))
    }

    pub fn clear_estop_requires_recovery(
        &mut self,
        operator_ack: bool,
        now_s: f64,
    ) -> RuntimeTrace {
        let mut errs = Vec::new();
        if !operator_ack {
            errs.push("operator_ack_required".into());
        }
        if now_s - self.last_heartbeat_s > self.config.heartbeat_period_s * 2.0 {
            errs.push("heartbeat_stale_for_recovery".into());
        }
        if !errs.is_empty() {
            return self
                .emit(RuntimeTrace::new(false, "recovery_refused", now_s).with_violations(errs));
        }
        if let Err(e) = self.plant.clear_estop(true) {
            return self.emit(
                RuntimeTrace::new(false, "recovery_refused", now_s)
                    .with_violations(vec![e.to_string()]),
            );
        }
        self.latch.clear();
        self.emit(RuntimeTrace::new(true, "recovery_cleared", now_s))
    }

    pub fn latch_abort(&mut self, reason: impl Into<String>, now_s: f64) -> RuntimeTrace {
        let reason = reason.into();
        self.latch.latch_abort(&reason);
        self.emit(RuntimeTrace::new(false, "abort_latched", now_s).with_violations(vec![reason]))
    }

    pub fn pre_actuation_check(&self, now_s: f64) -> Vec<String> {
        let mut errs = Vec::new();
        if self.config.require_online_identity {
            if !self.identity.complete_online() {
                errs.push("incomplete_online_runtime_identity".into());
            }
        } else if !self.identity.complete() {
            errs.push("incomplete_runtime_identity".into());
        }
        if self.latch.engaged {
            errs.push("estop_engaged".into());
        }
        if self.latch.abort_latched {
            errs.push(format!(
                "abort_latched:{}",
                self.latch.reason.as_deref().unwrap_or("unknown")
            ));
        }
        if self.last_heartbeat_s <= 0.0
            || (now_s - self.last_heartbeat_s) > self.config.heartbeat_period_s * 2.0
        {
            errs.push("heartbeat_missing_or_stale".into());
        }
        if self.config.require_sensor_before_write && self.last_sensor_s <= 0.0 {
            errs.push("sensor_never_marked".into());
        }
        if self.last_sensor_s > 0.0 && (now_s - self.last_sensor_s) > self.config.sensor_stale_s {
            errs.push("sensor_stale".into());
        }
        errs
    }

    /// THE driver write. Never calls plant.act directly.
    pub fn write_driver(
        &mut self,
        command: &dyn ActuationCommand,
        params: &ActionParams,
        now_s: f64,
    ) -> RuntimeTrace {
        let mut pre = self.pre_actuation_check(now_s);
        let cmd_hash = command.release_hash();
        if cmd_hash.is_empty() {
            pre.push("command_missing_release_hash".into());
        } else if cmd_hash != self.identity.release_hash.as_str() {
            pre.push("release_hash_mismatch_command_vs_session".into());
        }
        let cmd_as_built = command.as_built_hash();
        if !cmd_as_built.is_empty()
            && !self.identity.design_str().is_empty()
            && cmd_as_built != self.identity.design_str()
        {
            pre.push("as_built_hash_mismatch_command_vs_session".into());
        }
        let cal = self.identity.calibration_id_str();
        if !cal.is_empty() && !command.calibration_ids().iter().any(|c| c == cal) {
            pre.push("calibration_id_not_on_command".into());
        }
        if let Some(env) = &self.envelope {
            if env.require_for_write && !env.is_complete() {
                pre.push("envelope_pack_incomplete".into());
            } else {
                pre.extend(env.check_action(command.allowed_action()));
            }
        }
        if !pre.is_empty() {
            if pre
                .iter()
                .any(|e| e.contains("release_hash") || e.contains("estop"))
            {
                self.latch.abort_latched = true;
                self.latch.reason = Some(pre[0].clone());
            }
            return self.emit(
                RuntimeTrace::new(false, "driver_write_refused", now_s)
                    .with_command(command.command_id())
                    .with_violations(pre),
            );
        }
        if !command.acknowledged() {
            return self.emit(
                RuntimeTrace::new(false, "driver_write_refused", now_s)
                    .with_command(command.command_id())
                    .with_violations(vec!["command_not_acknowledged".into()]),
            );
        }

        let cals = vec![self.identity.calibration_id_str().to_string()];
        let bind = ExecuteBind {
            expected_release_hash: Some(self.identity.release_hash.as_str()),
            expected_calibration_ids: &cals,
            expected_sensor_packet_hash: self.expected_sensor_packet_hash.as_deref(),
            require_sensor_packet_hash: self.config.require_sensor_packet_hash,
            require_monotonic_sequence: self.config.require_monotonic_sequence,
            require_signature: self.config.require_command_signature,
            signing_key: self.signing_key.as_deref(),
        };
        let result = execute_certified_command(
            &mut self.plant,
            command,
            params,
            &mut self.ledger,
            now_s,
            &bind,
        );
        let ok = result.ok && result.executed;
        if result.outcome == realityos_kernel::CommandOutcome::Unknown {
            self.latch.abort_latched = true;
            self.latch.reason = Some("unknown_outcome".into());
            return self.emit(
                RuntimeTrace::new(false, "driver_write_unknown", now_s)
                    .with_command(result.command_id)
                    .with_violations(result.violations),
            );
        }
        if !ok
            && result
                .violations
                .iter()
                .any(|v| LATCHING_PREFIXES.iter().any(|p| v.starts_with(p)))
        {
            self.latch.abort_latched = true;
            self.latch.reason = Some(result.violations.join(","));
        }
        self.emit(
            RuntimeTrace::new(
                ok,
                if ok {
                    "driver_write"
                } else {
                    "driver_write_refused"
                },
                now_s,
            )
            .with_command(result.command_id)
            .with_violations(result.violations),
        )
    }

    pub fn apply_journal_continuity(&mut self, operator_ack: bool, now_s: f64) -> Value {
        if self.ledger.is_unreadable() {
            self.latch.engage("journal_unreadable");
            return json!({
                "applied": true,
                "ok": false,
                "start_refused": true,
                "violations": ["journal_unreadable"],
                "metal": false,
            });
        }
        let serial = self.identity.serial_str().to_string();
        let state = self.ledger.continuity_state(&serial);
        let mut violations = Vec::new();
        if let Some(last) = &state.last_identity {
            if !operator_ack {
                if let Some(fw) = last.get("firmware_id").and_then(Value::as_str) {
                    if !fw.is_empty() && fw != self.identity.firmware_str() {
                        violations.push("identity_continuity_firmware_mismatch".to_string());
                    }
                }
                if let Some(cal) = last.get("calibration_id").and_then(Value::as_str) {
                    if !cal.is_empty() && cal != self.identity.calibration_id_str() {
                        violations.push("identity_continuity_calibration_mismatch".to_string());
                    }
                }
                if let Some(rh) = last.get("release_hash").and_then(Value::as_str) {
                    if !rh.is_empty() && rh != self.identity.release_hash.as_str() {
                        violations.push("identity_continuity_release_mismatch".to_string());
                    }
                }
            }
        }
        if state.estop {
            self.latch
                .engage(state.estop_reason.unwrap_or_else(|| "estop".into()));
        }
        if state.identity_refuse_n >= self.config.repeated_refuse_n {
            self.latch.latch_abort(
                state
                    .last_identity_violation
                    .unwrap_or_else(|| "repeated_identity_refuse".into()),
            );
        }
        if !violations.is_empty() {
            self.latch.latch_abort(violations[0].clone());
            return json!({
                "applied": true,
                "ok": false,
                "start_refused": true,
                "violations": violations,
                "now_s": now_s,
                "metal": false,
            });
        }
        json!({
            "applied": true,
            "ok": true,
            "estop": self.latch.engaged,
            "abort_latched": self.latch.abort_latched,
            "metal": false,
        })
    }
}
