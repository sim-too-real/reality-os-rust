//! RuntimeGovernor — fail-closed actuation boundary.
//!
//! Does not replace RealityOS.decide. Does not invent. Both must cross here
//! to touch a plant.
//!
//! `OnlineLocked` exposes observe/operate methods but not policy-weakening
//! mutation: no `config_mut`, `envelope_mut`, `plant_mut`, `ledger_mut`,
//! or signing-key replacement.

use std::marker::PhantomData;
use std::path::Path;
use std::sync::Arc;

use realityos_core::{CertifiedCommand, IssuedCommand};
use realityos_kernel::{AuthorityClock, FakeClock};
use realityos_plant::{
    execute_certified_command, hash_sensor_packet, ActionParams, ActuationCommand, CommandLedger,
    ExecuteBind, Plant, SensorPacket,
};
use serde_json::{json, Map, Value};

use crate::envelope::DriverEnvelopePack;
use crate::identity::{match_expected_to_measured, RuntimeIdentity, ValidatedRuntimeIdentity};
use crate::latch::{EstopLatch, SafeState};
use crate::rail::{OnlineLocked, Rail, Simulation, UnlockedRail};
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
    "runtime_instance_mismatch",
    "actuator_scope_not_authorized",
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

impl GovernorConfig {
    pub fn online_locked() -> Self {
        Self {
            heartbeat_period_s: 1.0,
            sensor_stale_s: 2.0,
            require_sensor_before_write: true,
            require_online_identity: true,
            require_command_signature: true,
            require_monotonic_sequence: true,
            require_sensor_packet_hash: true,
            repeated_refuse_n: 3,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OnlineInitError(pub String);

impl std::fmt::Display for OnlineInitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for OnlineInitError {}

pub struct RuntimeGovernor<P: Plant, R: Rail = Simulation> {
    identity: RuntimeIdentity,
    plant: P,
    ledger: CommandLedger,
    config: GovernorConfig,
    envelope: Option<DriverEnvelopePack>,
    last_heartbeat_s: f64,
    last_sensor_s: f64,
    last_now_s: f64,
    expected_sensor_packet_hash: Option<String>,
    signing_key: Option<Vec<u8>>,
    authorized_actuator_ids: Vec<String>,
    safe_state: SafeState,
    latch: EstopLatch,
    traces: Vec<RuntimeTrace>,
    watchdog_period_s: f64,
    last_watchdog_s: f64,
    clock: Arc<dyn AuthorityClock>,
    validated: Option<ValidatedRuntimeIdentity>,
    hardware_session_dead: bool,
    last_device_capture_s: f64,
    _rail: PhantomData<R>,
}

/// Unforgeable ONLINE write capability.
/// Only [`RuntimeGovernor<OnlineLocked>::authorize_issued`] can construct this.
#[derive(Debug)]
pub struct OnlineWrite {
    command: CertifiedCommand,
}

impl OnlineWrite {
    pub fn command_id(&self) -> &str {
        self.command.command_id()
    }

    pub fn as_command(&self) -> &CertifiedCommand {
        &self.command
    }
}

impl<P: Plant> RuntimeGovernor<P, Simulation> {
    pub fn new(identity: RuntimeIdentity, plant: P) -> Self {
        Self::assemble(
            identity,
            plant,
            CommandLedger::new(),
            GovernorConfig::default(),
            None,
        )
    }
}

impl<P: Plant> RuntimeGovernor<P, crate::rail::Hil> {
    pub fn new_hil(identity: RuntimeIdentity, plant: P) -> Self {
        let cfg = GovernorConfig {
            require_monotonic_sequence: true,
            require_sensor_before_write: true,
            ..GovernorConfig::default()
        };
        Self::assemble(identity, plant, CommandLedger::new(), cfg, None)
    }
}

impl<P: Plant, R: Rail> RuntimeGovernor<P, R> {
    fn assemble(
        identity: RuntimeIdentity,
        plant: P,
        ledger: CommandLedger,
        config: GovernorConfig,
        signing_key: Option<Vec<u8>>,
    ) -> Self {
        Self {
            identity,
            plant,
            ledger,
            config,
            envelope: None,
            last_heartbeat_s: 0.0,
            last_sensor_s: 0.0,
            last_now_s: 0.0,
            expected_sensor_packet_hash: None,
            signing_key,
            authorized_actuator_ids: Vec::new(),
            safe_state: SafeState::Running,
            latch: EstopLatch::default(),
            traces: Vec::new(),
            watchdog_period_s: 0.05,
            last_watchdog_s: 0.0,
            clock: FakeClock::arc(0.0),
            validated: None,
            hardware_session_dead: false,
            last_device_capture_s: 0.0,
            _rail: PhantomData,
        }
    }

    pub fn authority_now_s(&self) -> f64 {
        self.clock.monotonic_now().secs()
    }

    pub fn validated_identity(&self) -> Option<&ValidatedRuntimeIdentity> {
        self.validated.as_ref()
    }

    pub fn hardware_session_dead(&self) -> bool {
        self.hardware_session_dead
    }

    pub fn last_device_capture_s(&self) -> f64 {
        self.last_device_capture_s
    }

    /// Re-probe the attached driver. Identity change, disconnect, placeholder,
    /// or missing identity FAULT/ABORTs this runtime instance. Recovery is a
    /// complete ONLINE restart — not continued execution under this instance.
    fn verify_live_hardware(&mut self, now_s: f64) -> Vec<String> {
        if self.hardware_session_dead {
            return vec!["hardware_session_requires_online_restart".into()];
        }
        let Some(measured) = self.plant.probe_identity() else {
            self.kill_hardware_session("hardware_identity_missing", now_s);
            return vec!["hardware_identity_missing".into()];
        };
        if let Err(e) =
            match_expected_to_measured(&self.identity, &measured, &self.authorized_actuator_ids)
        {
            self.kill_hardware_session(&e, now_s);
            return vec![e];
        }
        vec![]
    }

    fn kill_hardware_session(&mut self, reason: &str, now_s: f64) {
        let _ = now_s;
        self.hardware_session_dead = true;
        self.validated = None;
        self.safe_state = self.safe_state.tighten(SafeState::Fault);
        self.latch.engage(reason);
        self.plant.engage_estop(reason);
    }

    pub fn identity(&self) -> &RuntimeIdentity {
        &self.identity
    }

    pub fn plant(&self) -> &P {
        &self.plant
    }

    pub fn ledger(&self) -> &CommandLedger {
        &self.ledger
    }

    pub fn config(&self) -> &GovernorConfig {
        &self.config
    }

    pub fn envelope(&self) -> Option<&DriverEnvelopePack> {
        self.envelope.as_ref()
    }

    pub fn last_heartbeat_s(&self) -> f64 {
        self.last_heartbeat_s
    }

    /// Authority seconds since the last software-watchdog stamp.
    ///
    /// The stamp is taken at the start of the tick, so this includes that
    /// tick's journal+seal persist time. Idle serve must use this, not a
    /// wall Instant schedule, when deciding whether the next tick is due.
    pub fn watchdog_age_s(&self) -> f64 {
        let now = self.clock.monotonic_now().secs();
        if !now.is_finite() {
            return f64::INFINITY;
        }
        if self.last_watchdog_s <= 0.0 {
            return now.max(0.0);
        }
        (now - self.last_watchdog_s).max(0.0)
    }

    pub fn last_sensor_s(&self) -> f64 {
        self.last_sensor_s
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

    fn watchdog_tick_at(&mut self, now_s: f64) -> RuntimeTrace {
        if !now_s.is_finite() {
            return self.engage_estop_at("watchdog_non_finite_time", 0.0);
        }
        if self.last_watchdog_s > 0.0
            && (now_s - self.last_watchdog_s) > self.watchdog_period_s * 2.0
        {
            return self.engage_estop_at("software_watchdog_miss", now_s);
        }
        self.last_watchdog_s = now_s;
        self.emit(RuntimeTrace::new(true, "watchdog_tick", now_s))
    }

    /// Software supervisor using the authority clock. Not an independent hardware watchdog.
    pub fn watchdog_tick_now(&mut self) -> RuntimeTrace {
        self.watchdog_tick_at(self.clock.monotonic_now().secs())
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

    fn heartbeat_at(&mut self, now_s: f64) -> RuntimeTrace {
        self.last_heartbeat_s = now_s;
        self.emit(RuntimeTrace::new(true, "heartbeat", now_s))
    }

    pub fn heartbeat_now(&mut self) -> RuntimeTrace {
        self.heartbeat_at(self.clock.monotonic_now().secs())
    }

    fn record_sensor_at(
        &mut self,
        samples: &[(String, f64)],
        timestamp_s: f64,
        sequence: u64,
        frame_id: &str,
        sensor_id: &str,
    ) -> Result<String, String> {
        if samples.is_empty() {
            return Err("sensor_reading_empty".into());
        }
        if samples.iter().any(|(_, v)| !v.is_finite()) {
            return Err("sensor_sample_non_finite".into());
        }
        if !timestamp_s.is_finite() {
            return Err("sensor_timestamp_non_finite".into());
        }
        let hash = hash_sensor_packet(samples, timestamp_s, frame_id, sensor_id, sequence);
        // ONLINE freshness is authority receive time. Device timestamp is informative.
        let freshness_s = if R::ONLINE_LOCKED {
            self.clock.monotonic_now().secs()
        } else {
            timestamp_s
        };
        self.last_device_capture_s = timestamp_s;
        self.last_sensor_s = freshness_s;
        self.expected_sensor_packet_hash = Some(hash.clone());
        Ok(hash)
    }

    /// Production ingest: packet may carry a device capture time; receive is stamped here.
    pub fn ingest_sensor_packet(&mut self, mut packet: SensorPacket) -> Result<String, String> {
        if packet.samples.is_empty() {
            return Err("sensor_reading_empty".into());
        }
        if packet.samples.iter().any(|(_, v)| !v.is_finite()) {
            return Err("sensor_sample_non_finite".into());
        }
        if !packet.timestamp_s.is_finite() {
            return Err("sensor_timestamp_non_finite".into());
        }
        let receive_s = self.clock.monotonic_now().secs();
        if !receive_s.is_finite() {
            return Err("authority_receive_non_finite".into());
        }
        if R::ONLINE_LOCKED {
            let cal = self.identity.calibration_id_str();
            if !cal.is_empty()
                && !packet.calibration_hash.is_empty()
                && packet.calibration_hash != cal
            {
                return Err("sensor_calibration_mismatch".into());
            }
        }
        packet.authority_receive_s = Some(receive_s);
        packet.rehash();
        let hash = packet.content_hash.clone();
        self.last_device_capture_s = packet.timestamp_s;
        self.last_sensor_s = if R::ONLINE_LOCKED {
            receive_s
        } else {
            packet.timestamp_s
        };
        self.expected_sensor_packet_hash = Some(hash.clone());
        Ok(hash)
    }

    pub fn acquire_sensor(&mut self) -> Result<String, String> {
        let now = self.clock.monotonic_now().secs();
        match self.plant.read_driver_sensor(now) {
            Some(Ok(pkt)) => self.ingest_sensor_packet(pkt),
            Some(Err(e)) => Err(e.to_string()),
            None => Err("no_driver_sensor".into()),
        }
    }

    fn engage_estop_at(&mut self, reason: impl Into<String>, now_s: f64) -> RuntimeTrace {
        let reason = reason.into();
        self.latch.engage(&reason);
        self.plant.engage_estop(&reason);
        self.emit(RuntimeTrace::new(false, "estop", now_s).with_violations(vec![reason]))
    }

    pub fn engage_estop_now(&mut self, reason: impl Into<String>) -> RuntimeTrace {
        self.engage_estop_at(reason, self.clock.monotonic_now().secs())
    }

    fn clear_estop_requires_recovery_at(&mut self, operator_ack: bool, now_s: f64) -> RuntimeTrace {
        let mut errs = Vec::new();
        if R::ONLINE_LOCKED && self.hardware_session_dead {
            return self.emit(
                RuntimeTrace::new(false, "recovery_refused", now_s)
                    .with_violations(vec!["hardware_session_requires_online_restart".into()]),
            );
        }
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

    pub fn clear_estop_requires_recovery_now(&mut self, operator_ack: bool) -> RuntimeTrace {
        self.clear_estop_requires_recovery_at(operator_ack, self.clock.monotonic_now().secs())
    }

    fn latch_abort_at(&mut self, reason: impl Into<String>, now_s: f64) -> RuntimeTrace {
        let reason = reason.into();
        self.latch.latch_abort(&reason);
        self.emit(RuntimeTrace::new(false, "abort_latched", now_s).with_violations(vec![reason]))
    }

    pub fn latch_abort_now(&mut self, reason: impl Into<String>) -> RuntimeTrace {
        self.latch_abort_at(reason, self.clock.monotonic_now().secs())
    }

    pub fn pre_actuation_check(&self, now_s: f64) -> Vec<String> {
        let mut errs = Vec::new();
        if R::ONLINE_LOCKED && self.hardware_session_dead {
            errs.push("hardware_session_requires_online_restart".into());
        }
        if R::ONLINE_LOCKED && self.validated.is_none() {
            errs.push("online_identity_not_hardware_bound".into());
        }
        if self.safe_state.blocks_actuation() {
            errs.push(format!(
                "dispatch_safe_state_latched:{}",
                self.safe_state.as_str()
            ));
        }
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
        if R::ONLINE_LOCKED && self.last_now_s > 0.0 && now_s < self.last_now_s - 1e-12 {
            errs.push("time_rollback".into());
        }
        errs
    }

    pub fn latch_safe_state(&mut self, state: SafeState) {
        if R::ONLINE_LOCKED {
            self.safe_state = self.safe_state.tighten(state);
        } else {
            self.safe_state = state;
        }
    }

    pub fn safe_state(&self) -> SafeState {
        self.safe_state
    }

    fn write_driver_inner(
        &mut self,
        command: &dyn ActuationCommand,
        params: &ActionParams,
        now_s: f64,
    ) -> RuntimeTrace {
        if R::ONLINE_LOCKED {
            let hw = self.verify_live_hardware(now_s);
            if !hw.is_empty() {
                return self.emit(
                    RuntimeTrace::new(false, "driver_write_refused", now_s)
                        .with_command(command.command_id())
                        .with_violations(hw),
                );
            }
        }
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
        if R::ONLINE_LOCKED {
            let expect = self
                .validated
                .as_ref()
                .map(|v| v.instance_hash(&self.authorized_actuator_ids))
                .unwrap_or_default();
            if command.runtime_instance_hash().is_empty() {
                pre.push("command_missing_runtime_instance_hash".into());
            } else if command.runtime_instance_hash() != expect {
                pre.push("runtime_instance_mismatch".into());
            }
            if command.actuator_ids().is_empty() {
                pre.push("online_requires_actuator_ids".into());
            } else if !command
                .actuator_ids()
                .iter()
                .all(|id| self.authorized_actuator_ids.iter().any(|a| a == id))
            {
                pre.push("actuator_scope_not_authorized".into());
            }
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
                .any(|e| e.contains("release_hash") || e.contains("estop") || e == "time_rollback")
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
            force_online_rails: R::ONLINE_LOCKED,
        };
        let result = execute_certified_command(
            &mut self.plant,
            command,
            params,
            &mut self.ledger,
            now_s,
            &bind,
        );
        self.last_now_s = now_s;
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

impl<P: Plant, R: UnlockedRail> RuntimeGovernor<P, R> {
    pub fn plant_mut(&mut self) -> &mut P {
        &mut self.plant
    }

    pub fn ledger_mut(&mut self) -> &mut CommandLedger {
        &mut self.ledger
    }

    pub fn config_mut(&mut self) -> &mut GovernorConfig {
        &mut self.config
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

    pub fn heartbeat(&mut self, now_s: f64) -> RuntimeTrace {
        self.heartbeat_at(now_s)
    }

    pub fn watchdog_tick(&mut self, now_s: f64) -> RuntimeTrace {
        self.watchdog_tick_at(now_s)
    }

    pub fn engage_estop(&mut self, reason: impl Into<String>, now_s: f64) -> RuntimeTrace {
        self.engage_estop_at(reason, now_s)
    }

    pub fn clear_estop_requires_recovery(
        &mut self,
        operator_ack: bool,
        now_s: f64,
    ) -> RuntimeTrace {
        self.clear_estop_requires_recovery_at(operator_ack, now_s)
    }

    pub fn latch_abort(&mut self, reason: impl Into<String>, now_s: f64) -> RuntimeTrace {
        self.latch_abort_at(reason, now_s)
    }

    pub fn record_sensor(
        &mut self,
        samples: &[(String, f64)],
        timestamp_s: f64,
        sequence: u64,
        frame_id: &str,
        sensor_id: &str,
    ) -> Result<String, String> {
        self.record_sensor_at(samples, timestamp_s, sequence, frame_id, sensor_id)
    }

    pub fn mark_sensor(&mut self, now_s: f64, content_hash: Option<String>) {
        self.last_sensor_s = now_s;
        if let Some(h) = content_hash {
            self.expected_sensor_packet_hash = Some(h);
        }
    }

    /// SIM/HIL driver write. ONLINE uses [`RuntimeGovernor<OnlineLocked>::write_online`].
    pub fn write_driver(
        &mut self,
        command: &dyn ActuationCommand,
        params: &ActionParams,
        now_s: f64,
    ) -> RuntimeTrace {
        self.write_driver_inner(command, params, now_s)
    }
}

impl<P: Plant> RuntimeGovernor<P, OnlineLocked> {
    pub fn new_online(
        identity: RuntimeIdentity,
        mut plant: P,
        journal_path: impl AsRef<Path>,
        signing_key: Vec<u8>,
        first_online: bool,
        actuator_ids: Vec<String>,
        clock: Arc<dyn AuthorityClock>,
    ) -> Result<Self, OnlineInitError> {
        if signing_key.is_empty() {
            return Err(OnlineInitError("online_requires_signing_key".into()));
        }
        if actuator_ids.is_empty() {
            return Err(OnlineInitError("online_requires_actuator_ids".into()));
        }
        if !identity.complete_online() {
            return Err(OnlineInitError("incomplete_online_runtime_identity".into()));
        }
        let measured = plant
            .probe_identity()
            .ok_or_else(|| OnlineInitError("online_requires_hardware_identity".into()))?;
        let validated = ValidatedRuntimeIdentity::bind(identity.clone(), &measured, &actuator_ids)
            .map_err(OnlineInitError)?;
        plant.lock_production(&signing_key);
        let ledger = CommandLedger::with_online_journal(journal_path, first_online)
            .map_err(|e| OnlineInitError(e.to_string()))?;
        let mut env = DriverEnvelopePack::from_max_action(&plant.caps().max_action, true);
        if env.max_action_abs <= 0.0 {
            env.max_action_abs = 1.0;
        }
        let mut g = Self::assemble(
            identity,
            plant,
            ledger,
            GovernorConfig::online_locked(),
            Some(signing_key),
        );
        g.clock = clock;
        g.validated = Some(validated);
        g.authorized_actuator_ids = actuator_ids;
        g.envelope = Some(env);
        // Each emit fsyncs journal+seal. Stamping both ticks with one pre-emit
        // time makes the next watchdog_tick_now see persist latency as a miss
        // (software_watchdog_miss_before_bind on GHA / slow disks). Use current
        // authority time for the watchdog after the heartbeat persist.
        g.heartbeat_at(g.clock.monotonic_now().secs());
        let now_s = g.clock.monotonic_now().secs();
        let _ = g.watchdog_tick_at(now_s);
        let cont = g.apply_journal_continuity(false, now_s);
        if cont.get("start_refused").and_then(Value::as_bool) == Some(true) {
            return Err(OnlineInitError(
                cont.get("violations")
                    .and_then(Value::as_array)
                    .and_then(|a| a.first())
                    .and_then(Value::as_str)
                    .unwrap_or("journal_continuity_refused")
                    .into(),
            ));
        }
        Ok(g)
    }

    /// Bind, sign, and acknowledge a kernel-issued command with governor-owned
    /// identity, evidence, actuator ids, and signing key.
    pub fn authorize_issued(&self, issued: IssuedCommand) -> Result<OnlineWrite, Vec<String>> {
        if self.hardware_session_dead {
            return Err(vec!["hardware_session_requires_online_restart".into()]);
        }
        let Some(validated) = &self.validated else {
            return Err(vec!["online_identity_not_hardware_bound".into()]);
        };
        let Some(hash) = self.expected_sensor_packet_hash.as_deref() else {
            return Err(vec!["online_requires_sensor_hash".into()]);
        };
        let Some(key) = self.signing_key.as_deref() else {
            return Err(vec!["online_signing_key_missing".into()]);
        };
        if self.authorized_actuator_ids.is_empty() {
            return Err(vec!["online_requires_actuator_ids".into()]);
        }
        let instance = validated.instance_hash(&self.authorized_actuator_ids);
        let cmd = issued.bind_online(
            self.identity.release_hash.as_str(),
            self.identity.design_str(),
            self.identity.calibration_id_str(),
            hash,
            self.authorized_actuator_ids.clone(),
            &instance,
        )?;
        Ok(OnlineWrite {
            command: cmd.seal_online(key)?,
        })
    }

    /// Test/HIL fault injection: caller time. Ordinary ONLINE code must use
    /// [`Self::write_online_now`].
    pub(crate) fn write_online(
        &mut self,
        write: &OnlineWrite,
        params: &ActionParams,
        now_s: f64,
    ) -> RuntimeTrace {
        self.write_driver_inner(&write.command, params, now_s)
    }

    /// Production write: issue/write time comes from the authority clock.
    pub fn write_online_now(&mut self, write: &OnlineWrite, params: &ActionParams) -> RuntimeTrace {
        let now_s = self.clock.monotonic_now().secs();
        self.write_online(write, params, now_s)
    }

    /// HIL fault injection only: drop authority-owned sensor evidence.
    pub fn hil_drop_sensor_evidence(&mut self) {
        self.last_sensor_s = 0.0;
        self.expected_sensor_packet_hash = None;
    }
}
