use std::path::PathBuf;
use std::sync::Arc;

use realityos_core::{lifecycle, CertifiedCommand, IssuedCommand};
use realityos_governor::{
    DriverEnvelopePack, Hil, OnlineLocked, Rail, RuntimeGovernor, RuntimeIdentity, RuntimeTrace,
    SafeState, Simulation, UnlockedRail,
};
use realityos_kernel::{AuthorityClock, OsMonotonicClock};
use realityos_kernel::{
    CalibrationId, DesignContentHash, FirmwareId, ReleaseHash, SerialOrAsBuilt,
};
use realityos_plant::{ActionParams, Plant, PlantRealized, SimPlant};
use serde_json::json;

use crate::mode::{RuntimeMode, SessionStartError};

#[derive(Debug, Clone)]
pub struct StartArgs {
    pub mode: RuntimeMode,
    pub release_hash: String,
    pub release_class: String,
    pub design_content_hash: String,
    pub serial_or_as_built: String,
    pub firmware_id: String,
    pub calibration_id: String,
    pub require_verified_release: Option<bool>,
    pub require_command_signature: Option<bool>,
    pub require_driver_envelope: Option<bool>,
    pub max_action_abs: f64,
    pub journal_path: Option<PathBuf>,
    pub signing_key: Option<Vec<u8>>,
    pub first_online: bool,
    pub actuator_ids: Vec<String>,
}

impl StartArgs {
    pub fn simulation(release_hash: impl Into<String>) -> Self {
        Self {
            mode: RuntimeMode::Simulation,
            release_hash: release_hash.into(),
            release_class: "SCREENING".into(),
            design_content_hash: String::new(),
            serial_or_as_built: "SIM_SERIAL".into(),
            firmware_id: "SIM_FW".into(),
            calibration_id: "SIM_CAL".into(),
            require_verified_release: None,
            require_command_signature: None,
            require_driver_envelope: None,
            max_action_abs: 10.0,
            journal_path: None,
            signing_key: None,
            first_online: false,
            actuator_ids: Vec::new(),
        }
    }
}

pub struct RuntimeSession<P: Plant, R: Rail = Simulation> {
    pub mode: RuntimeMode,
    pub governor: RuntimeGovernor<P, R>,
    last_sequence: i64,
    acknowledged_ids: std::collections::HashSet<String>,
    safe_state: SafeState,
    last_sensor_hash: Option<String>,
}

impl RuntimeSession<SimPlant, Simulation> {
    pub fn start_sim(
        args: StartArgs,
        plant: SimPlant,
        now_s: f64,
    ) -> Result<Self, SessionStartError> {
        Self::start(args, plant, now_s)
    }
}

impl<P: Plant> RuntimeSession<P, Simulation> {
    pub fn start(args: StartArgs, plant: P, now_s: f64) -> Result<Self, SessionStartError> {
        if args.mode == RuntimeMode::Online {
            return Err(SessionStartError(
                "online_requires_start_online_typestate".into(),
            ));
        }
        if args.mode == RuntimeMode::Hil {
            return Err(SessionStartError("hil_requires_start_hil_typestate".into()));
        }
        if args.require_verified_release == Some(false)
            || args.require_command_signature == Some(false)
            || args.require_driver_envelope == Some(false)
        {
            // SIM may opt out; recorded only.
        }

        let identity = identity_from_args(&args)?;
        let mut governor = RuntimeGovernor::new(identity, plant);
        governor.config_mut().require_online_identity = false;
        governor.config_mut().require_sensor_before_write = false;
        governor.config_mut().require_monotonic_sequence = false;
        governor.config_mut().require_command_signature = false;
        governor.config_mut().require_sensor_packet_hash = false;
        let mut env =
            DriverEnvelopePack::from_max_action(&governor.plant().caps().max_action, false);
        if env.max_action_abs <= 0.0 {
            env.max_action_abs = args.max_action_abs;
        }
        governor.set_envelope(env);
        governor.heartbeat(now_s);
        let _ = governor.watchdog_tick(now_s);
        Ok(Self {
            mode: RuntimeMode::Simulation,
            governor,
            last_sequence: 0,
            acknowledged_ids: std::collections::HashSet::new(),
            safe_state: SafeState::Running,
            last_sensor_hash: None,
        })
    }
}

impl<P: Plant> RuntimeSession<P, Hil> {
    pub fn start_hil(args: StartArgs, plant: P, now_s: f64) -> Result<Self, SessionStartError> {
        let identity = identity_from_args(&args)?;
        let mut governor = RuntimeGovernor::new_hil(identity, plant);
        let mut env =
            DriverEnvelopePack::from_max_action(&governor.plant().caps().max_action, true);
        if env.max_action_abs <= 0.0 {
            env.max_action_abs = args.max_action_abs;
        }
        governor.set_envelope(env);
        governor.heartbeat(now_s);
        let _ = governor.watchdog_tick(now_s);
        Ok(Self {
            mode: RuntimeMode::Hil,
            governor,
            last_sequence: 0,
            acknowledged_ids: std::collections::HashSet::new(),
            safe_state: SafeState::Running,
            last_sensor_hash: None,
        })
    }
}

impl<P: Plant> RuntimeSession<P, OnlineLocked> {
    /// Production ONLINE start. Uses OS monotonic time. Tests/HIL inject a clock
    /// via [`Self::start_online_with_clock`].
    pub fn start_online(args: StartArgs, plant: P) -> Result<Self, SessionStartError> {
        Self::start_online_with_clock(args, plant, Arc::new(OsMonotonicClock::new()))
    }

    pub fn start_online_with_clock(
        args: StartArgs,
        plant: P,
        clock: Arc<dyn AuthorityClock>,
    ) -> Result<Self, SessionStartError> {
        let mut opted = Vec::new();
        if args.require_verified_release == Some(false) {
            opted.push("verified_release");
        }
        if args.require_command_signature == Some(false) {
            opted.push("command_signature");
        }
        if args.require_driver_envelope == Some(false) {
            opted.push("driver_envelope");
        }
        if !opted.is_empty() {
            return Err(SessionStartError(format!(
                "online_refuses_safety_rail_opt_out:{}",
                opted.join(",")
            )));
        }
        if args.release_class != "MFG_CANDIDATE" {
            return Err(SessionStartError(format!(
                "online_requires_MFG_CANDIDATE_got:{}",
                args.release_class
            )));
        }
        if args.serial_or_as_built.is_empty() || args.serial_or_as_built.starts_with("SIM_") {
            return Err(SessionStartError(
                "online_requires_real_serial_or_as_built".into(),
            ));
        }
        if args.firmware_id.is_empty() || args.firmware_id.starts_with("SIM_") {
            return Err(SessionStartError("online_requires_real_firmware_id".into()));
        }
        if args.calibration_id.is_empty() || args.calibration_id == "SIM_CAL" {
            return Err(SessionStartError(
                "online_requires_real_calibration_id".into(),
            ));
        }
        if !plant.is_online() {
            return Err(SessionStartError("online_rejects_dry_run_plant".into()));
        }
        let journal = args
            .journal_path
            .clone()
            .ok_or_else(|| SessionStartError("online_requires_durable_journal".into()))?;
        let key = args
            .signing_key
            .clone()
            .ok_or_else(|| SessionStartError("online_requires_signing_key".into()))?;
        if args.actuator_ids.is_empty() {
            return Err(SessionStartError("online_requires_actuator_ids".into()));
        }
        let identity = identity_from_args(&args)?;
        if !identity.complete_online() {
            return Err(SessionStartError(
                "incomplete_online_runtime_identity".into(),
            ));
        }
        let governor = RuntimeGovernor::new_online(
            identity,
            plant,
            journal,
            key,
            args.first_online,
            args.actuator_ids.clone(),
            clock,
        )
        .map_err(|e| SessionStartError(e.0))?;
        Ok(Self {
            mode: RuntimeMode::Online,
            governor,
            last_sequence: 0,
            acknowledged_ids: std::collections::HashSet::new(),
            safe_state: SafeState::Running,
            last_sensor_hash: None,
        })
    }
}

fn identity_from_args(args: &StartArgs) -> Result<RuntimeIdentity, SessionStartError> {
    Ok(RuntimeIdentity {
        release_hash: ReleaseHash::new(&args.release_hash)
            .map_err(|e| SessionStartError(e.to_string()))?,
        design_content_hash: if args.design_content_hash.is_empty() {
            None
        } else {
            Some(
                DesignContentHash::new(&args.design_content_hash)
                    .map_err(|e| SessionStartError(e.to_string()))?,
            )
        },
        serial_or_as_built: SerialOrAsBuilt::new(&args.serial_or_as_built).ok(),
        firmware_id: FirmwareId::new(&args.firmware_id).ok(),
        calibration_id: CalibrationId::new(&args.calibration_id).ok(),
    })
}

impl<P: Plant, R: UnlockedRail> RuntimeSession<P, R> {
    pub fn bind_and_dispatch(
        &mut self,
        command: CertifiedCommand,
        params: &ActionParams,
        now_s: f64,
    ) -> DispatchResult {
        if self.safe_state.blocks_actuation() {
            return DispatchResult::refused(
                self.mode,
                vec![format!(
                    "dispatch_safe_state_latched:{}",
                    self.safe_state.as_str()
                )],
            );
        }
        let bound = match lifecycle::CertifiedIntent::from_command(command).bind_identity(
            self.governor.identity().release_hash.as_str(),
            self.governor.identity().design_str(),
            self.governor.identity().calibration_id_str(),
        ) {
            Ok(c) => c,
            Err(errs) => return DispatchResult::refused(self.mode, errs),
        };

        let mut cmd = bound.into_command();
        if let Some(h) = &self.last_sensor_hash {
            if cmd.sensor_packet_hash().is_empty() {
                cmd = cmd.with_sensor_packet_hash(h.clone());
            } else if cmd.sensor_packet_hash() != *h {
                return DispatchResult::refused(
                    self.mode,
                    vec!["online_refuses_forged_sensor_packet_hash".into()],
                );
            }
        }
        let command = lifecycle::CertifiedIntent::from_command(cmd)
            .acknowledge_sim()
            .into_command();
        self.acknowledged_ids
            .insert(command.command_id().to_string());
        self.last_sequence = command.sequence_value().max(self.last_sequence);
        let _ = self.governor.watchdog_tick(now_s);
        let trace = self.governor.write_driver(&command, params, now_s);
        DispatchResult {
            ok: trace.ok,
            executed: trace.ok,
            mode: self.mode,
            violations: trace.violations.clone(),
            trace: Some(trace),
            realized: None,
            metal: false,
        }
    }
}

impl<P: Plant> RuntimeSession<P, OnlineLocked> {
    pub fn dispatch_issued(
        &mut self,
        command: IssuedCommand,
        params: &ActionParams,
    ) -> DispatchResult {
        if self.safe_state.blocks_actuation() {
            return DispatchResult::refused(
                self.mode,
                vec![format!(
                    "dispatch_safe_state_latched:{}",
                    self.safe_state.as_str()
                )],
            );
        }
        let write = match self.governor.authorize_issued(command) {
            Ok(w) => w,
            Err(errs) => return DispatchResult::refused(self.mode, errs),
        };
        self.acknowledged_ids.insert(write.command_id().to_string());
        self.last_sequence = write.as_command().sequence_value().max(self.last_sequence);
        let _ = self.governor.watchdog_tick_now();
        let trace = self.governor.write_online_now(&write, params);
        DispatchResult {
            ok: trace.ok,
            executed: trace.ok,
            mode: self.mode,
            violations: trace.violations.clone(),
            trace: Some(trace),
            realized: None,
            metal: false,
        }
    }

    pub fn acquire_sensor(&mut self) -> Result<String, String> {
        let hash = self.governor.acquire_sensor()?;
        self.last_sensor_hash = Some(hash.clone());
        Ok(hash)
    }

    pub fn ingest_sensor_packet(
        &mut self,
        packet: realityos_plant::SensorPacket,
    ) -> Result<String, String> {
        let hash = self.governor.ingest_sensor_packet(packet)?;
        self.last_sensor_hash = Some(hash.clone());
        Ok(hash)
    }
}

impl<P: Plant, R: Rail> RuntimeSession<P, R> {
    pub fn latch_safe_state(&mut self, state: SafeState, _reason: &str) {
        if R::ONLINE_LOCKED {
            self.safe_state = self.safe_state.tighten(state);
        } else {
            self.safe_state = state;
        }
        self.governor.latch_safe_state(state);
    }

    pub fn last_sensor_hash(&self) -> Option<&str> {
        self.last_sensor_hash.as_deref()
    }

    pub fn refuse_bare_action(&self, _action: &[f64]) -> DispatchResult {
        DispatchResult::refused(
            self.mode,
            vec!["bind_and_dispatch_requires_CertifiedCommand".into()],
        )
    }
}

impl<P: Plant, R: UnlockedRail> RuntimeSession<P, R> {
    pub fn ingest_sensor(
        &mut self,
        samples: &[(String, f64)],
        timestamp_s: Option<f64>,
        now_s: f64,
    ) -> Result<String, String> {
        if samples.is_empty() {
            return Err("sensor_reading_empty".into());
        }
        if samples.iter().any(|(_, v)| !v.is_finite()) {
            return Err("sensor_sample_non_finite".into());
        }
        if self.mode == RuntimeMode::Hil && timestamp_s.is_none() {
            return Err("sensor_timestamp_required".into());
        }
        let ts = timestamp_s.unwrap_or(now_s);
        if !ts.is_finite() || !now_s.is_finite() {
            return Err("sensor_timestamp_non_finite".into());
        }
        // ONLINE freshness is authority receive time; device capture is informative.
        if self.mode != RuntimeMode::Online
            && timestamp_s.is_some()
            && (ts - now_s).abs() > self.governor.config().sensor_stale_s
        {
            return Err("sensor_timestamp_stale_vs_now".into());
        }
        self.last_sequence = self.last_sequence.saturating_add(1);
        let hash = self.governor.record_sensor(
            samples,
            ts,
            self.last_sequence as u64,
            "session/sensor",
            "session",
        )?;
        self.last_sensor_hash = Some(hash.clone());
        Ok(hash)
    }
}

#[derive(Debug, Clone)]
pub struct DispatchResult {
    pub ok: bool,
    pub executed: bool,
    pub mode: RuntimeMode,
    pub violations: Vec<String>,
    pub trace: Option<RuntimeTrace>,
    pub realized: Option<PlantRealized>,
    pub metal: bool,
}

impl DispatchResult {
    fn refused(mode: RuntimeMode, violations: Vec<String>) -> Self {
        Self {
            ok: false,
            executed: false,
            mode,
            violations,
            trace: None,
            realized: None,
            metal: false,
        }
    }

    pub fn to_json(&self) -> serde_json::Value {
        json!({
            "ok": self.ok,
            "executed": self.executed,
            "mode": self.mode.as_str(),
            "violations": self.violations,
            "metal": false,
            "learned_actuator_authority": false,
        })
    }
}
