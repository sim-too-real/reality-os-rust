//! Metal proof artifact. Aggregates are derived from case measurements.

use serde::{Deserialize, Serialize};

pub const PROOF_SCHEMA: &str = "realityos.metal_proof/1";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum BlockingLayer {
    ProtocolBlocked,
    AuthorizationBlocked,
    EgressBlocked,
    OsBlocked,
    CrashRecoveryBlocked,
    None,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CaseRecord {
    pub name: String,
    pub expected_authorization: bool,
    pub decision_result: String,
    pub writes_before: u64,
    pub writes_after: u64,
    pub write_delta: u64,
    pub device_acknowledgement: bool,
    pub observed_motion: Option<String>,
    pub blocking_layer: BlockingLayer,
    pub journal_result: String,
    pub proposal: String,
    pub unauthorized_write: bool,
    /// Command-egress attempts. Not a physical device write.
    #[serde(default)]
    pub egress_attempt_delta: u64,
    #[serde(default)]
    pub serial_tx_before: u64,
    #[serde(default)]
    pub serial_tx_after: u64,
    /// Certified command frames that passed write_all+flush. Not setup/sensor.
    #[serde(default)]
    pub serial_tx_delta: u64,
    #[serde(default)]
    pub device_ack_delta: u64,
    #[serde(default)]
    pub unauthorized_device_ack_delta: u64,
    /// Authority-owned present after the command. Do not infer from Goal.
    #[serde(default)]
    pub observed_present_after: Option<i32>,
    #[serde(default)]
    pub commanded_goal: Option<i32>,
    #[serde(default)]
    pub experiment_min: Option<i32>,
    #[serde(default)]
    pub experiment_max: Option<i32>,
    /// Authority violation tokens from the measured IPC body. Identity and
    /// disconnect aggregates must not be inferred from case-name substrings.
    #[serde(default)]
    pub violations: Vec<String>,
}

impl CaseRecord {
    #[allow(clippy::too_many_arguments)]
    pub fn measure(
        name: impl Into<String>,
        proposal: impl Into<String>,
        decision_result: impl Into<String>,
        blocking_layer: BlockingLayer,
        writes_before: u64,
        writes_after: u64,
        expected_authorization: bool,
        device_acknowledgement: bool,
        observed_motion: Option<String>,
        journal_result: impl Into<String>,
    ) -> Self {
        let write_delta = writes_after.saturating_sub(writes_before);
        Self {
            name: name.into(),
            expected_authorization,
            decision_result: decision_result.into(),
            writes_before,
            writes_after,
            write_delta,
            device_acknowledgement,
            observed_motion,
            blocking_layer,
            journal_result: journal_result.into(),
            proposal: proposal.into(),
            unauthorized_write: false,
            egress_attempt_delta: write_delta,
            serial_tx_before: 0,
            serial_tx_after: 0,
            serial_tx_delta: 0,
            device_ack_delta: 0,
            unauthorized_device_ack_delta: 0,
            observed_present_after: None,
            commanded_goal: None,
            experiment_min: None,
            experiment_max: None,
            violations: Vec::new(),
        }
    }

    pub fn with_violations(mut self, violations: Vec<String>) -> Self {
        self.violations = violations;
        self
    }

    #[allow(clippy::too_many_arguments)]
    pub fn with_certified_transport(
        mut self,
        egress_before: u64,
        egress_after: u64,
        serial_tx_before: u64,
        serial_tx_after: u64,
        ack_before: u64,
        ack_after: u64,
        observed_present_after: Option<i32>,
        commanded_goal: Option<i32>,
        experiment_min: Option<i32>,
        experiment_max: Option<i32>,
    ) -> Self {
        self.egress_attempt_delta = egress_after.saturating_sub(egress_before);
        self.serial_tx_before = serial_tx_before;
        self.serial_tx_after = serial_tx_after;
        self.serial_tx_delta = serial_tx_after.saturating_sub(serial_tx_before);
        self.device_ack_delta = ack_after.saturating_sub(ack_before);
        self.unauthorized_device_ack_delta = if self.expected_authorization {
            0
        } else {
            self.device_ack_delta
        };
        self.observed_present_after = observed_present_after;
        self.commanded_goal = commanded_goal;
        self.experiment_min = experiment_min;
        self.experiment_max = experiment_max;
        self.unauthorized_write = !self.expected_authorization && self.serial_tx_delta > 0;
        self
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct ProofAggregates {
    pub valid_commands: u64,
    pub valid_physical_writes: u64,
    pub hostile_cases: u64,
    pub unauthorized_physical_writes: u64,
    pub identity_mismatch_refusals: u64,
    pub disconnect_refusals: u64,
    pub duplicate_writes_after_restart: u64,
}

pub fn aggregates_from_cases(cases: &[CaseRecord]) -> ProofAggregates {
    let mut a = ProofAggregates::default();
    for c in cases {
        if c.expected_authorization {
            if c.serial_tx_delta > 0 {
                a.valid_commands += 1;
                a.valid_physical_writes += c.serial_tx_delta;
            }
        } else {
            a.hostile_cases += 1;
            if c.unauthorized_write || c.serial_tx_delta > 0 {
                a.unauthorized_physical_writes += c.serial_tx_delta;
            }
            if refusal_has_token(c, IDENTITY_MISMATCH_TOKENS) {
                a.identity_mismatch_refusals += 1;
            }
            if refusal_has_token(c, DISCONNECT_TOKENS) {
                a.disconnect_refusals += 1;
            }
            if c.blocking_layer == BlockingLayer::CrashRecoveryBlocked
                && c.serial_tx_delta > 0
                && c.name.contains("restart")
            {
                a.duplicate_writes_after_restart += c.serial_tx_delta;
            }
        }
    }
    a
}

const IDENTITY_MISMATCH_TOKENS: &[&str] = &[
    "hardware_firmware_mismatch",
    "hardware_serial_mismatch",
    "hardware_identity_mismatch",
    "metal_serial_mismatch",
    "identity_mismatch",
];

const DISCONNECT_TOKENS: &[&str] = &[
    "online_hardware_disconnected",
    "driver not connected",
    "metal_live_io_deadline",
    "metal_serial_closed",
];

fn refusal_evidence(c: &CaseRecord) -> String {
    let mut blob = c.decision_result.to_ascii_lowercase();
    for v in &c.violations {
        blob.push(' ');
        blob.push_str(&v.to_ascii_lowercase());
    }
    blob
}

fn refusal_has_token(c: &CaseRecord, tokens: &[&str]) -> bool {
    let blob = refusal_evidence(c);
    tokens.iter().any(|tok| blob.contains(tok))
}

/// Experiment acceptance for a no-load XL330 hold, in position ticks.
///
/// XL330 resolution is 4096 ticks/rev (0.088°). Robotis does not specify
/// zero-count hold. Under factory Position P Gain 400 a parked horn still
/// hunts a few counts. 4 ticks ≈ 0.35°, well below the certified 32-tick
/// (~2.8°) nudge. Not a datasheet accuracy spec and not certified.
pub const HOLD_STILL_MAX_ABS_TICKS: i64 = 4;

/// Device present delta from campaign `observed_motion`.
/// Prefers `delta=N`; otherwise `present A->B`.
pub fn present_position_delta(motion: Option<&str>) -> Option<i64> {
    let s = motion?;
    if let Some(idx) = s.find("delta=") {
        let tok = s[idx + 6..].split_whitespace().next()?;
        if tok == "None" {
            return None;
        }
        return tok.parse().ok();
    }
    let rest = s.split_once("present ")?.1;
    let pair = rest.split_whitespace().next()?;
    let (a, b) = pair.split_once("->")?;
    Some(b.parse::<i64>().ok()? - a.parse::<i64>().ok()?)
}

/// Hold stayed inside the no-load hunt band. Missing present is not a hold.
pub fn hold_still(motion: Option<&str>) -> bool {
    present_position_delta(motion).is_some_and(|d| d.abs() <= HOLD_STILL_MAX_ABS_TICKS)
}

/// Nudge moved farther than no-load hunt. A 1-count flicker is not item 8.
pub fn nudge_moved(motion: Option<&str>) -> bool {
    present_position_delta(motion).is_some_and(|d| d.abs() > HOLD_STILL_MAX_ABS_TICKS)
}

fn parse_goal_from_motion(motion: Option<&str>) -> Option<i32> {
    let s = motion?;
    let idx = s.find("goal=")?;
    s[idx + 5..].split_whitespace().next()?.parse().ok()
}

fn parse_present_after_from_motion(motion: Option<&str>) -> Option<i32> {
    let s = motion?;
    let rest = s.split_once("present ")?.1;
    let pair = rest.split_whitespace().next()?;
    let (_, b) = pair.split_once("->")?;
    b.parse().ok()
}

fn observed_present_after(c: &CaseRecord) -> Option<i32> {
    c.observed_present_after
        .or_else(|| parse_present_after_from_motion(c.observed_motion.as_deref()))
}

fn commanded_goal(c: &CaseRecord) -> Option<i32> {
    c.commanded_goal
        .or_else(|| parse_goal_from_motion(c.observed_motion.as_deref()))
}

fn case_cage(c: &CaseRecord, meta: &ProofMeta) -> Option<(i32, i32)> {
    let min = c.experiment_min.or(meta.experiment_min)?;
    let max = c.experiment_max.or(meta.experiment_max)?;
    if min <= max {
        Some((min, max))
    } else {
        None
    }
}

fn in_cage(pos: i32, min: i32, max: i32) -> bool {
    pos >= min && pos <= max
}

fn motion_toward_goal(present_before: i32, present_after: i32, goal: i32) -> bool {
    let need = goal - present_before;
    let got = present_after - present_before;
    need != 0 && got.signum() == need.signum() && i64::from(got.abs()) > HOLD_STILL_MAX_ABS_TICKS
}

fn certified_command_ok(c: &CaseRecord) -> bool {
    c.expected_authorization
        && c.serial_tx_delta == 1
        && c.device_acknowledgement
        && c.device_ack_delta >= 1
}

fn valid_hold_measured(c: &CaseRecord, meta: &ProofMeta) -> bool {
    if c.name != "valid_hold" || !certified_command_ok(c) {
        return false;
    }
    let Some(after) = observed_present_after(c) else {
        return false;
    };
    if !hold_still(c.observed_motion.as_deref()) {
        return false;
    }
    let Some((min, max)) = case_cage(c, meta) else {
        return false;
    };
    in_cage(after, min, max)
}

fn valid_nudge_measured(c: &CaseRecord, meta: &ProofMeta) -> bool {
    if c.name != "valid_nudge" || !certified_command_ok(c) {
        return false;
    }
    let Some(after) = observed_present_after(c) else {
        return false;
    };
    let Some(goal) = commanded_goal(c) else {
        return false;
    };
    if !nudge_moved(c.observed_motion.as_deref()) {
        return false;
    }
    let Some(delta) = present_position_delta(c.observed_motion.as_deref()) else {
        return false;
    };
    let Ok(delta_i32) = i32::try_from(delta) else {
        return false;
    };
    let before = after.saturating_sub(delta_i32);
    if !motion_toward_goal(before, after, goal) {
        return false;
    }
    let Some((min, max)) = case_cage(c, meta) else {
        return false;
    };
    in_cage(after, min, max) && in_cage(goal, min, max)
}

fn pwm_cap_configured_and_read_back(meta: &ProofMeta) -> bool {
    let (Some(req), Some(got)) = (meta.pwm_limit_requested, meta.pwm_limit_measured) else {
        return false;
    };
    got <= req && req <= crate::protocol::XL330_PWM_LIMIT_MAX
}

fn absolute_cage_active(meta: &ProofMeta) -> bool {
    matches!(
        (meta.experiment_min, meta.experiment_max),
        (Some(min), Some(max)) if min <= max
    )
}

fn post_tx_pre_status_no_retransmit(cases: &[CaseRecord]) -> bool {
    cases.iter().any(|c| {
        c.name.contains("after_serial_tx_before_status")
            && c.name.contains("restart")
            && c.serial_tx_delta == 0
    })
}

fn before_prepare_no_retransmit(cases: &[CaseRecord]) -> bool {
    cases
        .iter()
        .any(|c| c.name.contains("crash_restart_before_prepare") && c.serial_tx_delta == 0)
}

fn crash_restarts_have_zero_serial_tx(cases: &[CaseRecord]) -> bool {
    cases
        .iter()
        .filter(|c| {
            c.blocking_layer == BlockingLayer::CrashRecoveryBlocked && c.name.contains("restart")
        })
        .all(|c| c.serial_tx_delta == 0)
}

fn unauthorized_ack_delta(cases: &[CaseRecord]) -> u64 {
    cases
        .iter()
        .filter(|c| !c.expected_authorization)
        .map(|c| c.unauthorized_device_ack_delta)
        .sum()
}

fn eeprom_model_from_identity(identity: &serde_json::Value) -> Option<u16> {
    for key in ["/measured/model", "/model"] {
        if let Some(n) = identity
            .pointer(key)
            .and_then(|v| v.as_u64())
            .and_then(|n| u16::try_from(n).ok())
            .filter(|n| *n != 0)
        {
            return Some(n);
        }
    }
    None
}

/// Refuse a swapped 1190/1200 product name. Unit fixtures without an
/// EEPROM model number skip this check; a live measured.json always has one.
fn hardware_model_matches_eeprom(meta: &ProofMeta) -> Result<(), String> {
    let Some(n) = eeprom_model_from_identity(&meta.real_device_identity) else {
        return Ok(());
    };
    match crate::protocol::xl330_hardware_model(n) {
        Some(want) if meta.hardware_model == want => Ok(()),
        Some(want) => Err(format!(
            "metal_proof_hardware_model_mismatch:label={} eeprom={n} want={want}",
            meta.hardware_model
        )),
        None => Err(format!("metal_proof_unknown_xl330_model:{n}")),
    }
}

fn identity_looks_like_pty_stand_in(id: &serde_json::Value) -> bool {
    let status = id
        .pointer("/hardware_identity/evidence_status")
        .and_then(|v| v.as_str())
        .or_else(|| id.get("evidence_status").and_then(|v| v.as_str()));
    let metal = id
        .pointer("/hardware_identity/metal")
        .and_then(|v| v.as_bool())
        .or_else(|| id.get("metal").and_then(|v| v.as_bool()));
    status == Some("PTY_STAND_IN_NOT_METAL") || metal == Some(false)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetalProof {
    pub schema: String,
    pub hardware_model: String,
    pub controller_model: String,
    pub real_device_identity: serde_json::Value,
    pub software_commit_sha: String,
    pub authority_uid: String,
    pub autonomy_uid: String,
    pub test_date: String,
    pub valid_commands: u64,
    pub valid_physical_device_writes: u64,
    pub hostile_cases: u64,
    pub unauthorized_physical_device_writes: u64,
    pub direct_device_open_attempts: u64,
    pub direct_device_open_successes: u64,
    pub duplicate_writes_after_restart: u64,
    pub identity_mismatch_refusals: u64,
    pub disconnect_refusals: u64,
    pub hardware_present: bool,
    pub used_os_monotonic_clock: bool,
    pub used_hardware_driver_port: bool,
    pub cutoff_mechanism: String,
    pub cutoff_tested: bool,
    #[serde(default)]
    pub cutoff_operator_attested: bool,
    #[serde(default)]
    pub cutoff_live_observed: bool,
    #[serde(default)]
    pub unplug_live_observed: bool,
    #[serde(default)]
    pub pwm_limit_requested: Option<u16>,
    #[serde(default)]
    pub pwm_limit_measured: Option<u16>,
    #[serde(default)]
    pub experiment_min: Option<i32>,
    #[serde(default)]
    pub experiment_max: Option<i32>,
    #[serde(default)]
    pub startup_present: Option<i32>,
    pub sensor_source: String,
    pub device_capture_s: Option<f64>,
    pub authority_receive_s: Option<f64>,
    pub freshness_threshold_s: Option<f64>,
    pub experiment_status: String,
    pub unresolved_assumptions: Vec<String>,
    pub cases: Vec<CaseRecord>,
}

impl MetalProof {
    pub fn from_measured(
        meta: ProofMeta,
        cases: Vec<CaseRecord>,
        unresolved: Vec<String>,
    ) -> Result<Self, String> {
        if !meta.hardware_present {
            return Err("metal_proof_refuses_without_hardware".into());
        }
        if cases.is_empty() {
            return Err("metal_proof_requires_measured_cases".into());
        }
        hardware_model_matches_eeprom(&meta)?;
        let a = aggregates_from_cases(&cases);
        let has_hold = cases.iter().any(|c| valid_hold_measured(c, &meta));
        let has_nudge = cases.iter().any(|c| valid_nudge_measured(c, &meta));
        let freshness_measured =
            meta.device_capture_s.is_some() && meta.authority_receive_s.is_some();
        let not_pty_stand_in = !identity_looks_like_pty_stand_in(&meta.real_device_identity);
        let cutoff_attested = meta.cutoff_operator_attested || meta.cutoff_tested;
        let cutoff_live = meta.cutoff_live_observed;
        let unplug_live = meta.unplug_live_observed;
        let pwm_ok = pwm_cap_configured_and_read_back(&meta);
        let cage_ok = absolute_cage_active(&meta);
        let post_tx_ok = post_tx_pre_status_no_retransmit(&cases);
        let before_prep_ok = before_prepare_no_retransmit(&cases);
        let crash_tx_ok = crash_restarts_have_zero_serial_tx(&cases);
        let unauth_ack = unauthorized_ack_delta(&cases);
        let measured_success = a.unauthorized_physical_writes == 0
            && unauth_ack == 0
            && has_hold
            && has_nudge
            && pwm_ok
            && cage_ok
            && post_tx_ok
            && before_prep_ok
            && crash_tx_ok
            && meta.direct_device_open_successes == 0
            && meta.direct_device_open_attempts > 0
            && cutoff_live
            && unplug_live
            && a.duplicate_writes_after_restart == 0
            && a.identity_mismatch_refusals > 0
            && a.disconnect_refusals > 0
            && freshness_measured
            && not_pty_stand_in
            && meta.used_os_monotonic_clock
            && meta.used_hardware_driver_port;
        Ok(Self {
            schema: PROOF_SCHEMA.into(),
            hardware_model: meta.hardware_model,
            controller_model: meta.controller_model,
            real_device_identity: meta.real_device_identity,
            software_commit_sha: meta.software_commit_sha,
            authority_uid: meta.authority_uid,
            autonomy_uid: meta.autonomy_uid,
            test_date: meta.test_date,
            valid_commands: a.valid_commands,
            valid_physical_device_writes: a.valid_physical_writes,
            hostile_cases: a.hostile_cases,
            unauthorized_physical_device_writes: a.unauthorized_physical_writes,
            direct_device_open_attempts: meta.direct_device_open_attempts,
            direct_device_open_successes: meta.direct_device_open_successes,
            duplicate_writes_after_restart: a.duplicate_writes_after_restart,
            identity_mismatch_refusals: a.identity_mismatch_refusals,
            disconnect_refusals: a.disconnect_refusals,
            hardware_present: true,
            used_os_monotonic_clock: meta.used_os_monotonic_clock,
            used_hardware_driver_port: meta.used_hardware_driver_port,
            cutoff_mechanism: meta.cutoff_mechanism,
            cutoff_tested: cutoff_attested,
            cutoff_operator_attested: cutoff_attested,
            cutoff_live_observed: cutoff_live,
            unplug_live_observed: unplug_live,
            pwm_limit_requested: meta.pwm_limit_requested,
            pwm_limit_measured: meta.pwm_limit_measured,
            experiment_min: meta.experiment_min,
            experiment_max: meta.experiment_max,
            startup_present: meta.startup_present,
            sensor_source: meta.sensor_source,
            device_capture_s: meta.device_capture_s,
            authority_receive_s: meta.authority_receive_s,
            freshness_threshold_s: meta.freshness_threshold_s,
            experiment_status: if measured_success {
                "measured_success".into()
            } else {
                "measured_incomplete_or_failed".into()
            },
            unresolved_assumptions: unresolved,
            cases,
        })
    }

    /// Sixteen-point report derived from this measured proof. Not a certification.
    pub fn sixteen_point_report(&self) -> String {
        let hold = self.cases.iter().find(|c| c.name == "valid_hold");
        let nudge = self.cases.iter().find(|c| c.name == "valid_nudge");
        let crash_retry = self.duplicate_writes_after_restart;
        let id = &self.real_device_identity;
        let verdict = if self.experiment_status == "measured_success" {
            "measured_success (every listed criterion is true on this run)"
        } else {
            "not success: measured_incomplete_or_failed (do not claim the experiment succeeded)"
        };
        format!(
            "# Metal experiment 16-point report\n\
             \n\
             Derived from `{schema}` at {date}. Not ISO/PL/SIL/STO/SS1. Not root protection.\n\
             \n\
             1. **Actuator.** {hw} via {ctrl}. Position-mode PWM Limit cap requested={pwm_req:?} measured={pwm_got:?} (output/PWM cap, not a certified torque limit; raw * 0.113 ≈ percent). Session cage startup={startup:?} min={cage_min:?} max={cage_max:?}. Limits are in `docs/METAL_EXPERIMENT.md`.\n\
             2. **Independent VIN cutoff.** {cutoff}. Operator attested: {cutoff_attested}. Live observed: {cutoff_live}. Only live observation may satisfy measured_success. Not labeled STO/SS1/PL/SIL.\n\
             3. **HardwareDriverPort.** used_hardware_driver_port={port}. One XL330 port: open, sidecar+tty exclusive, probe_identity, sensor, certified write, ack, disconnect, close, torque-off stop.\n\
             4. **Measured identity.** {id}\n\
             5. **Composition.** used_os_monotonic_clock={clock}. `realityos-metal-smoke serve` uses `RuntimeSession<..., OnlineLocked>::start_online` and `OsMonotonicClock`, not HIL `Authority` / `FakeClock`.\n\
             6. **Two-UID attacks.** authority={auth} autonomy={auto}. direct_device_open_attempts={att} successes={succ} (must be attempts>0 and successes==0).\n\
             7. **Zero-motion baseline.** valid_hold serial_tx_delta={hold_tx} ack={hold_ack} present_after={hold_present:?} motion={hold_motion}\n\
             8. **Bounded one-axis motion.** valid_nudge serial_tx_delta={nudge_tx} ack={nudge_ack} present_after={nudge_present:?} motion={nudge_motion}\n\
             9. **Hostile campaign.** hostile_cases={hostile} unauthorized_certified_serial_tx={unauth} unauthorized_device_ack={unauth_ack} (required 0).\n\
             10. **Crash/restart.** duplicate_writes_after_restart={crash} (required 0; after_serial_tx_before_status and other ambiguous restarts must not retransmit).\n\
             11. **Disconnect / identity fail-closed.** identity_mismatch_refusals={idm} disconnect_refusals={disc}. Live USB-UART unplug observed: {unplug_live}. Campaign hooks are not a physical unplug.\n\
             12. **Sensor freshness.** source={src}; device_capture_s={cap:?}; authority_receive_s={recv:?}; freshness_threshold_s={thr:?}. Capture is device Realtime Tick; freshness anchor is authority monotonic receive time.\n\
             13. **Proof artifact.** schema={schema} hardware_present={hp} commit={sha}. Separate from HIL proofs. Aggregates are certified serial-TX deltas, not write attempts.\n\
             14. **All success criteria.** experiment_status={status}\n\
             15. **Unresolved (explicit non-claims).** {assumptions}\n\
             16. **Verdict.** {verdict}\n",
            schema = self.schema,
            date = self.test_date,
            hw = self.hardware_model,
            ctrl = self.controller_model,
            cutoff = self.cutoff_mechanism,
            cutoff_attested = self.cutoff_operator_attested,
            cutoff_live = self.cutoff_live_observed,
            unplug_live = self.unplug_live_observed,
            pwm_req = self.pwm_limit_requested,
            pwm_got = self.pwm_limit_measured,
            startup = self.startup_present,
            cage_min = self.experiment_min,
            cage_max = self.experiment_max,
            port = self.used_hardware_driver_port,
            id = id,
            clock = self.used_os_monotonic_clock,
            auth = self.authority_uid,
            auto = self.autonomy_uid,
            att = self.direct_device_open_attempts,
            succ = self.direct_device_open_successes,
            hold_tx = hold.map(|c| c.serial_tx_delta).unwrap_or(0),
            hold_ack = hold.map(|c| c.device_acknowledgement).unwrap_or(false),
            hold_present = hold.and_then(|c| c.observed_present_after),
            hold_motion = hold
                .and_then(|c| c.observed_motion.clone())
                .unwrap_or_else(|| "missing_valid_hold_case".into()),
            nudge_tx = nudge.map(|c| c.serial_tx_delta).unwrap_or(0),
            nudge_ack = nudge.map(|c| c.device_acknowledgement).unwrap_or(false),
            nudge_present = nudge.and_then(|c| c.observed_present_after),
            nudge_motion = nudge
                .and_then(|c| c.observed_motion.clone())
                .unwrap_or_else(|| "missing_valid_nudge_case".into()),
            hostile = self.hostile_cases,
            unauth = self.unauthorized_physical_device_writes,
            unauth_ack = unauthorized_ack_delta(&self.cases),
            crash = crash_retry,
            idm = self.identity_mismatch_refusals,
            disc = self.disconnect_refusals,
            src = self.sensor_source,
            cap = self.device_capture_s,
            recv = self.authority_receive_s,
            thr = self.freshness_threshold_s,
            hp = self.hardware_present,
            sha = self.software_commit_sha,
            status = self.experiment_status,
            assumptions = self.unresolved_assumptions.join("; "),
            verdict = verdict,
        )
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProofMeta {
    pub hardware_model: String,
    pub controller_model: String,
    pub real_device_identity: serde_json::Value,
    pub software_commit_sha: String,
    pub authority_uid: String,
    pub autonomy_uid: String,
    pub test_date: String,
    pub hardware_present: bool,
    pub used_os_monotonic_clock: bool,
    pub used_hardware_driver_port: bool,
    pub cutoff_mechanism: String,
    pub cutoff_tested: bool,
    #[serde(default)]
    pub cutoff_operator_attested: bool,
    #[serde(default)]
    pub cutoff_live_observed: bool,
    #[serde(default)]
    pub unplug_live_observed: bool,
    #[serde(default)]
    pub pwm_limit_requested: Option<u16>,
    #[serde(default)]
    pub pwm_limit_measured: Option<u16>,
    #[serde(default)]
    pub experiment_min: Option<i32>,
    #[serde(default)]
    pub experiment_max: Option<i32>,
    #[serde(default)]
    pub startup_present: Option<i32>,
    pub direct_device_open_attempts: u64,
    pub direct_device_open_successes: u64,
    pub duplicate_writes_after_restart: u64,
    #[serde(default)]
    pub sensor_source: String,
    #[serde(default)]
    pub device_capture_s: Option<f64>,
    #[serde(default)]
    pub authority_receive_s: Option<f64>,
    #[serde(default)]
    pub freshness_threshold_s: Option<f64>,
}

pub fn default_unresolved() -> Vec<String> {
    vec![
        "root can open any endpoint; root is outside this threat model".into(),
        "same-UID chmod or /proc/<pid>/fd can recover a locked device node".into(),
        "journal+seal is a hash-chain / local seal, not WORM or anti-rollback storage".into(),
        "paired restore of an older journal+seal is indistinguishable from that earlier valid tip"
            .into(),
        "filesystem signing.key is not a hardware root of trust / TPM / HSM".into(),
        "XL330 torque-disable is a register write, not an independent power cutoff".into(),
        "USB-serial adapter serial is not a factory actuator serial; the servo EEPROM has none"
            .into(),
        "XL330 has no useful factory unique actuator serial. An identical-model, identical-firmware servo swapped behind the same USB adapter and bus ID may be indistinguishable. This experiment binds USB adapter identity + bus ID + model + firmware + deployment calibration/design. That is not a claim that arbitrary production robots always provide unique hardware identity".into(),
        "PWM Limit is an output/PWM cap (raw * 0.113 ≈ percent of full PWM; 885 ≈ 100%), not a certified torque limit. Current Limit is configured but is not the Position Mode torque boundary. A conservative cap that cannot move an unloaded horn is incomplete until an operator raises max_pwm_limit_raw; software must not silently restore factory 885".into(),
        "successful software-watchdog pets are in-memory only and are not an independent hardware watchdog; miss/ESTOP remains fail-closed and durably recorded".into(),
        "Realtime Tick is a wrapping 1 ms device counter, not a synchronized clock".into(),
        "hold-still acceptance is |present delta| <= 4 ticks (~0.35°); XL330 quantization is 0.088°/tick and no-load P-gain hunt is not specified as 0. Not certified positioning accuracy".into(),
        "live EEPROM identity re-read is skipped when the motion-block read already took >=15 ms; that cycle keeps the previously latched identity".into(),
        "identity CRC/NAK after a good motion sample keeps the previous latched identity for that cycle".into(),
        "a half-duplex TTL/RS485 adapter that needs more than 1.5 ms after host TX, or more than 500 ms after DTR-RESET, is still a first-contact hole".into(),
        "probe rematches a dangling USB-serial by-id from aliases latched before the first open and the campaign points probe at the live ttyUSB so a 0750 plugdev by-id dir cannot hide the node from authority; a udev rename that also changes the adapter serial still fail-closes".into(),
        "HUPCL is cleared on the live exclusive fd via termios after open; stty after TIOCEXCL is EBUSY on the node and on /proc/<pid>/fd/N, /proc/self/fd/N misses an O_CLOEXEC tty, and a fresh USB-serial session restores kernel-default HUPCL so a pre-open stty is lost".into(),
        "campaign settle treats present inside the hold-still band of the written goal as arrived; Moving=0 alone is not arrived (accel below Moving Threshold)".into(),
        "an XL330 already in Wizard RC-PWM / S.BUS / iBUS mode at boot cannot be identified over Protocol 2.0".into(),
        "a USB-UART with no adapter serial (typical CH340/CP2102) is rebound by vid:pid:bus:devpath / by-path, not KERNEL==ttyUSB0 or a parent hub serial; a living stale ttyUSB0 after re-enum is not kept if its measured serial drifted; campaign waits up to 4s then fails closed instead of handing the stale name to serve; two empty-serial adapters that share dest on different buses used to collide (both usb:vid:pid:1); the same bus+dest is still one port".into(),
        "REALITYOS_METAL_BAUD / SERVO_ID are probe hints; serve keeps the pair probe wrote into metal.json (a 1 Mbps hint on a factory 57600 XL330 used to fail identify)".into(),
        "2 / 3 / 4 Mbps join the probe scan only when hinted, and never ahead of factory 57600 / 115200 / 1 Mbps; a 1 Mbps docs hint or a mistaken 2/3/4 Mbps Wizard hint used to open that rate twice before 57600 and could wedge CH340 so the factory servo was never found".into(),
        "serve measures the USB-adapter serial before open; a recycled living ttyUSB0 whose serial drifted must not reach torque-on".into(),
        "campaign proof-meta reads measured/os-probe/freshness from files; interpolating JSON into python '''...''' dies on an apostrophe in a USB serial".into(),
        "campaign installs docs/metal_proof.json relative to the script's repo, not the caller's working directory; sudo /path/scripts/metal-campaign.sh from another cwd used to write ~/docs after a live run".into(),
        "campaign runs as root with umask 0077; proof_meta.json and the cases file are chmod 0644 and chowned to the authority UID so report can read them. A hardened root umask used to abort mint after the physical run".into(),
        "campaign finds metal binaries in the script repo when REALITYOS_METAL_BIN=$PWD/target/debug points at the caller's cwd; sudo /path/scripts/metal-campaign.sh from another cwd used to exit 2 before probe".into(),
        "first USB prepare fails closed until the UART sysfs node has a non-empty USB serial or busnum:devpath:vid:pid, then waits briefly for iSerial and locks that identity; a dest-only bind still matches after iSerial appears. An empty CH340 serial file, a parent hub serial, inventing 0:nodevpath, waiting for idVendor alone, or preferring a late FTDI serial after a dest-only bind used to miss before hold. FTDI/U2D2 latency_timer must read back 1 after write; a silent failed set used to keep 16 ms and miss the 40 ms live deadline on the first hold. USB power/control on the UART device must read back on after write; a silent failed set used to keep autosuspend auto and miss that deadline after an idle gap. Campaign creates realityos-authority / realityos-autonomy / realityos-ipc and requires python3/timeout before first USB prepare; creating users after udev OWNER= or missing python3 after probe used to fail the first bench run. nscd/sssd can still hide a just-created user so chown/OWNER= fail; campaign flushes those caches and fails closed unless the USB tty inode uid is the authority uid and mode is 0600. Real USB-serial also requires fuser and udevadm; a missing fuser used to skip the holder check and open a UART ModemManager already had, and a missing /run/udev/rules.d used to skip ID_MM_DEVICE_IGNORE. Writing the ignore rule then udevadm control --reload || true used to announce success without loading it; a leftover 99-realityos-metal-*.rules from a SIGKILL'd run used to skip rewrite; fuser ran only before udevadm trigger --action=change, which can wake ModemManager. Reload must succeed, udevadm info must read ID_MM_DEVICE_IGNORE=1, the recorded USB identity is rematched after that trigger (FTDI/U2D2 can come back as ttyUSB1) before metal.json is rewritten, and the holder check runs again after that rematch. First prepare used to run only before journal tmpfs / staging / metal-deploy chown; that chown can emit a udev change that wakes ModemManager and resets FTDI latency_timer, so prepare runs again immediately before probe opens the UART. claim_usb_tty used to chown/chmod on every call even when the inode was already authority 0600, and metal-deploy always chowned the tty; the extra pre-probe claim then emitted another udev change and probe opened while ModemManager could still be waking (or FTDI came back as ttyUSB1 / latency_timer 16 ms). Claim, deploy, latency_timer, and power/control now skip a no-op write (a rewrite of 1/on still emits udev change). After a real claim/latency/power write, prepare settles udev, rematches the recorded USB identity, re-applies owner/latency/power only if they drifted, refuses holders, and fails closed unless latency_timer/power/control still read back 1/on. connect_serial skips a no-op chmod 0600 (that chmod can emit the same udev change and reset FTDI latency_timer before the first live hold). Campaign stty -F -hupcl before probe used to DTR-RESET cheap FTDI/CP2102; the driver clears HUPCL on the exclusive fd. Probe broadcast sniff takes exclusive on a real UART so ModemManager cannot AT-probe during the 500 ms open-settle. After serve open, claim/latency/power used to run inside if without || return so a failed latency_timer write was ignored (set -e is disabled in if) and the first hold could run at 16 ms; the campaign now settles and fails closed unless latency_timer/power/control still read back".into(),
        "after serve open, a udev change can dangle /dev/serial/by-id or rename ttyUSB0 while the exclusive fd is still the live UART; bus_up / probe_identity must not treat that vanished path as unplug (a real unplug fails the next xfer)".into(),
        "force_disconnect and hot_swap.json are campaign hooks, not a physical USB unplug; measured_success requires a live USB-UART unplug and a live VIN drop. If unplug kills serve, the campaign records the drop evidence and serial_tx; it does not invent a disconnect token".into(),
        "no STO/SS1/PLC/SIL/ISO is provided or claimed".into(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aggregates_come_from_deltas_not_hardcoded_zeros() {
        let cases = vec![
            CaseRecord::measure(
                "valid_hold",
                "hold",
                "write:allow",
                BlockingLayer::None,
                0,
                1,
                true,
                true,
                Some("none".into()),
                "consumed",
            )
            .with_certified_transport(
                0,
                1,
                0,
                1,
                0,
                1,
                Some(2048),
                Some(2048),
                Some(2000),
                Some(2096),
            ),
            CaseRecord::measure(
                "valid_nudge",
                "drive",
                "write:allow",
                BlockingLayer::None,
                1,
                2,
                true,
                true,
                Some("ticks=4".into()),
                "consumed",
            )
            .with_certified_transport(
                1,
                2,
                1,
                2,
                1,
                2,
                Some(2080),
                Some(2080),
                Some(2000),
                Some(2096),
            ),
            CaseRecord::measure(
                "unsupported",
                "dance",
                "semantic:refuse",
                BlockingLayer::AuthorizationBlocked,
                2,
                2,
                false,
                false,
                None,
                "no_consume",
            )
            .with_certified_transport(
                2,
                2,
                2,
                2,
                2,
                2,
                None,
                None,
                Some(2000),
                Some(2096),
            ),
        ];
        let a = aggregates_from_cases(&cases);
        assert_eq!(a.valid_physical_writes, 2);
        assert_eq!(a.hostile_cases, 1);
        assert_eq!(a.unauthorized_physical_writes, 0);
    }

    #[test]
    fn identity_and_disconnect_aggregates_use_tokens_not_case_names() {
        let named_only = CaseRecord::measure(
            "firmware_mismatch",
            "hot_swap",
            "authorize:refuse",
            BlockingLayer::AuthorizationBlocked,
            0,
            0,
            false,
            false,
            None,
            "no_consume",
        );
        let a = aggregates_from_cases(&[named_only]);
        assert_eq!(a.identity_mismatch_refusals, 0);
        assert_eq!(a.disconnect_refusals, 0);

        let identity = CaseRecord::measure(
            "unrelated_name",
            "hot_swap",
            "authorize:refuse",
            BlockingLayer::AuthorizationBlocked,
            0,
            0,
            false,
            false,
            None,
            "no_consume",
        )
        .with_violations(vec!["hardware_firmware_mismatch".into()]);
        let disconnect = CaseRecord::measure(
            "also_unrelated",
            "unplug",
            "authorize:refuse",
            BlockingLayer::AuthorizationBlocked,
            0,
            0,
            false,
            false,
            None,
            "no_consume",
        )
        .with_violations(vec!["online_hardware_disconnected".into()]);
        let b = aggregates_from_cases(&[identity, disconnect]);
        assert_eq!(b.identity_mismatch_refusals, 1);
        assert_eq!(b.disconnect_refusals, 1);
    }

    #[test]
    fn proof_refuses_without_hardware() {
        let meta = ProofMeta {
            hardware_model: "none".into(),
            controller_model: "none".into(),
            real_device_identity: serde_json::json!({}),
            software_commit_sha: "x".into(),
            authority_uid: "a".into(),
            autonomy_uid: "b".into(),
            test_date: "t".into(),
            hardware_present: false,
            used_os_monotonic_clock: true,
            used_hardware_driver_port: true,
            cutoff_mechanism: "none".into(),
            cutoff_tested: false,
            cutoff_operator_attested: false,
            cutoff_live_observed: false,
            unplug_live_observed: false,
            pwm_limit_requested: None,
            pwm_limit_measured: None,
            experiment_min: None,
            experiment_max: None,
            startup_present: None,
            direct_device_open_attempts: 0,
            direct_device_open_successes: 0,
            duplicate_writes_after_restart: 0,
            sensor_source: String::new(),
            device_capture_s: None,
            authority_receive_s: None,
            freshness_threshold_s: None,
        };
        assert!(MetalProof::from_measured(meta, vec![], vec![]).is_err());
    }

    fn crash_restart(name: &str) -> CaseRecord {
        CaseRecord::measure(
            name,
            "same command_id after crash/restart",
            "crash:no_auto_retry",
            BlockingLayer::CrashRecoveryBlocked,
            2,
            2,
            false,
            false,
            None,
            "not_retried",
        )
        .with_certified_transport(2, 2, 2, 2, 2, 2, None, None, Some(2000), Some(2096))
    }

    fn ok_cases() -> Vec<CaseRecord> {
        vec![
            CaseRecord::measure(
                "valid_hold",
                "hold",
                "write:allow",
                BlockingLayer::None,
                0,
                1,
                true,
                true,
                Some("present 2048->2048 goal=2048 delta=0".into()),
                "consumed",
            )
            .with_certified_transport(
                0,
                1,
                0,
                1,
                0,
                1,
                Some(2048),
                Some(2048),
                Some(2000),
                Some(2096),
            ),
            CaseRecord::measure(
                "valid_nudge",
                "drive",
                "write:allow",
                BlockingLayer::None,
                1,
                2,
                true,
                true,
                Some("present 2048->2080 goal=2080 delta=32".into()),
                "consumed",
            )
            .with_certified_transport(
                1,
                2,
                1,
                2,
                1,
                2,
                Some(2080),
                Some(2080),
                Some(2000),
                Some(2096),
            ),
            CaseRecord::measure(
                "firmware_mismatch",
                "hot_swap",
                "authorize:refuse",
                BlockingLayer::AuthorizationBlocked,
                2,
                2,
                false,
                false,
                None,
                "no_consume",
            )
            .with_violations(vec!["hardware_firmware_mismatch".into()])
            .with_certified_transport(
                2,
                2,
                2,
                2,
                2,
                2,
                None,
                None,
                Some(2000),
                Some(2096),
            ),
            CaseRecord::measure(
                "device_disconnect",
                "unplug",
                "authorize:refuse",
                BlockingLayer::AuthorizationBlocked,
                2,
                2,
                false,
                false,
                None,
                "no_consume",
            )
            .with_violations(vec!["online_hardware_disconnected".into()])
            .with_certified_transport(
                2,
                2,
                2,
                2,
                2,
                2,
                None,
                None,
                Some(2000),
                Some(2096),
            ),
            crash_restart("crash_restart_before_prepare"),
            crash_restart("crash_restart_after_serial_tx_before_status"),
            crash_restart("crash_restart_during_write"),
            crash_restart("crash_restart_after_prepare_before_write"),
            crash_restart("crash_restart_after_write_before_ack"),
            crash_restart("crash_restart_after_ack"),
        ]
    }

    fn ok_meta(cutoff: bool) -> ProofMeta {
        ProofMeta {
            hardware_model: "XL330-M288-T".into(),
            controller_model: "usb-uart".into(),
            real_device_identity: serde_json::json!({"serial":"FT1:id1"}),
            software_commit_sha: "abc".into(),
            authority_uid: "realityos-authority".into(),
            autonomy_uid: "realityos-autonomy".into(),
            test_date: "t".into(),
            hardware_present: true,
            used_os_monotonic_clock: true,
            used_hardware_driver_port: true,
            cutoff_mechanism: "bench VIN switch".into(),
            cutoff_tested: cutoff,
            cutoff_operator_attested: cutoff,
            cutoff_live_observed: cutoff,
            unplug_live_observed: cutoff,
            pwm_limit_requested: Some(200),
            pwm_limit_measured: Some(200),
            experiment_min: Some(2000),
            experiment_max: Some(2096),
            startup_present: Some(2048),
            direct_device_open_attempts: 1,
            direct_device_open_successes: 0,
            duplicate_writes_after_restart: 0,
            sensor_source: "xl330 tick".into(),
            device_capture_s: Some(1.2),
            authority_receive_s: Some(0.1),
            freshness_threshold_s: Some(2.0),
        }
    }

    #[test]
    fn sixteen_point_report_is_derived_and_does_not_invent_success() {
        let ok =
            MetalProof::from_measured(ok_meta(true), ok_cases(), default_unresolved()).unwrap();
        assert_eq!(ok.experiment_status, "measured_success");
        let text = ok.sixteen_point_report();
        for n in 1..=16 {
            assert!(text.contains(&format!("{n}.")), "missing point {n}: {text}");
        }
        assert!(text.contains("FT1:id1"));
        assert!(text.contains("direct_device_open_attempts=1"));
        assert!(text.contains("measured_success"));
        let incomplete =
            MetalProof::from_measured(ok_meta(false), ok_cases(), default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
        let t = incomplete.sixteen_point_report();
        assert!(t.contains("not success"));
        assert!(!t.contains("every listed criterion is true on this run"));
    }

    #[test]
    fn measured_success_requires_identity_disconnect_and_freshness() {
        let writes_only: Vec<CaseRecord> = ok_cases()
            .into_iter()
            .filter(|c| c.expected_authorization)
            .collect();
        let incomplete =
            MetalProof::from_measured(ok_meta(true), writes_only, default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
        let mut no_fresh = ok_meta(true);
        no_fresh.device_capture_s = None;
        no_fresh.authority_receive_s = None;
        let incomplete =
            MetalProof::from_measured(no_fresh, ok_cases(), default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
    }

    #[test]
    fn measured_success_requires_hold_still_and_nudge_present_delta() {
        assert!(hold_still(Some("present 2048->2048 goal=2048 delta=0")));
        assert!(hold_still(Some("present 2048->2050 goal=2048 delta=2")));
        assert!(!hold_still(Some("present 2048->2080 goal=2080 delta=32")));
        assert!(nudge_moved(Some("present 2048->2080 goal=2080 delta=32")));
        assert!(!nudge_moved(Some("present 2048->2050 goal=2080 delta=2")));

        let mut stuck = ok_cases();
        stuck[1].observed_motion = Some("present 2048->2048 goal=2080 delta=0".into());
        let incomplete =
            MetalProof::from_measured(ok_meta(true), stuck, default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status, "measured_incomplete_or_failed",
            "a goal write without present motion is not a nudge"
        );
        let mut hunt_only = ok_cases();
        hunt_only[1].observed_motion = Some("present 2048->2050 goal=2080 delta=2".into());
        let incomplete =
            MetalProof::from_measured(ok_meta(true), hunt_only, default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status, "measured_incomplete_or_failed",
            "a 2-tick flicker is no-load hunt, not the certified 32-tick nudge"
        );
        let mut yanked = ok_cases();
        yanked[0].observed_motion = Some("present 2048->2080 goal=2080 delta=32".into());
        let incomplete =
            MetalProof::from_measured(ok_meta(true), yanked, default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status, "measured_incomplete_or_failed",
            "a hold that traveled the nudge step is not a zero-motion baseline"
        );
        let mut hunt_hold = ok_cases();
        hunt_hold[0].observed_motion = Some("present 2048->2050 goal=2048 delta=2".into());
        let ok = MetalProof::from_measured(ok_meta(true), hunt_hold, default_unresolved()).unwrap();
        assert_eq!(
            ok.experiment_status, "measured_success",
            "no-load encoder hunt inside {HOLD_STILL_MAX_ABS_TICKS} ticks is still a hold"
        );
    }

    #[test]
    fn proof_refuses_swapped_xl330_hardware_model() {
        let mut swapped = ok_meta(true);
        swapped.hardware_model = "XL330-M077-T".into();
        swapped.real_device_identity = serde_json::json!({"measured": {"model": 1200}});
        let err = MetalProof::from_measured(swapped, ok_cases(), default_unresolved())
            .expect_err("M288 EEPROM must not mint as M077");
        assert!(err.contains("metal_proof_hardware_model_mismatch"), "{err}");
        let mut labeled = ok_meta(true);
        labeled.real_device_identity = serde_json::json!({"measured": {"model": 1200}});
        let ok = MetalProof::from_measured(labeled, ok_cases(), default_unresolved()).unwrap();
        assert_eq!(ok.hardware_model, "XL330-M288-T");
    }

    #[test]
    fn measured_success_refuses_pty_stand_in_identity() {
        let mut pty = ok_meta(true);
        pty.real_device_identity = serde_json::json!({
            "hardware_identity": {
                "metal": false,
                "evidence_status": "PTY_STAND_IN_NOT_METAL",
                "serial": "tty:2:1:id1"
            }
        });
        let incomplete = MetalProof::from_measured(pty, ok_cases(), default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
    }

    #[test]
    fn missing_ack_prevents_measured_success() {
        let mut cases = ok_cases();
        cases[0].device_acknowledgement = false;
        cases[0].device_ack_delta = 0;
        let incomplete =
            MetalProof::from_measured(ok_meta(true), cases, default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
    }

    #[test]
    fn missing_post_motion_prevents_measured_success() {
        let mut cases = ok_cases();
        cases[1].observed_present_after = None;
        cases[1].observed_motion = None;
        let incomplete =
            MetalProof::from_measured(ok_meta(true), cases, default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
    }

    #[test]
    fn unauthorized_serial_tx_fails_proof() {
        let mut cases = ok_cases();
        cases[2] = cases[2].clone().with_certified_transport(
            2,
            3,
            2,
            3,
            2,
            2,
            None,
            None,
            Some(2000),
            Some(2096),
        );
        let incomplete =
            MetalProof::from_measured(ok_meta(true), cases, default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
        assert!(incomplete.unauthorized_physical_device_writes > 0);
    }

    #[test]
    fn duplicate_writes_come_from_cases_not_hardcoded_meta() {
        let mut meta = ok_meta(true);
        meta.duplicate_writes_after_restart = 99;
        let ok = MetalProof::from_measured(meta, ok_cases(), default_unresolved()).unwrap();
        assert_eq!(ok.duplicate_writes_after_restart, 0);
        assert_eq!(ok.experiment_status, "measured_success");
    }

    #[test]
    fn measured_success_requires_before_prepare_crash_restart() {
        let cases: Vec<CaseRecord> = ok_cases()
            .into_iter()
            .filter(|c| !c.name.contains("crash_restart_before_prepare"))
            .collect();
        let incomplete =
            MetalProof::from_measured(ok_meta(true), cases, default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
    }

    #[test]
    fn operator_cutoff_attestation_without_live_observation_prevents_success() {
        let mut meta = ok_meta(true);
        meta.cutoff_tested = true;
        meta.cutoff_operator_attested = true;
        meta.cutoff_live_observed = false;
        let incomplete = MetalProof::from_measured(meta, ok_cases(), default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
        assert!(incomplete.cutoff_operator_attested);
        assert!(!incomplete.cutoff_live_observed);
    }

    #[test]
    fn synthetic_disconnect_without_live_unplug_prevents_success() {
        let mut meta = ok_meta(true);
        meta.unplug_live_observed = false;
        let incomplete = MetalProof::from_measured(meta, ok_cases(), default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
        assert!(!incomplete.unplug_live_observed);
    }

    #[test]
    fn hardcoded_clock_flag_without_os_monotonic_prevents_success() {
        let mut meta = ok_meta(true);
        meta.used_os_monotonic_clock = false;
        let incomplete = MetalProof::from_measured(meta, ok_cases(), default_unresolved()).unwrap();
        assert_eq!(
            incomplete.experiment_status,
            "measured_incomplete_or_failed"
        );
        assert!(!incomplete.used_os_monotonic_clock);
    }
}
