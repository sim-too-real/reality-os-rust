use realityos_core::CertifiedCommand;
use realityos_governor::{
    DriverEnvelopePack, RuntimeGovernor, RuntimeIdentity, RuntimeTrace, SafeState,
};
use realityos_kernel::{
    CalibrationId, DesignContentHash, FirmwareId, ReleaseHash, SerialOrAsBuilt,
};
use realityos_plant::{ActionParams, Plant, PlantRealized, SimPlant};
use serde_json::json;
use sha2::{Digest, Sha256};

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
        }
    }
}

pub struct RuntimeSession<P: Plant> {
    pub mode: RuntimeMode,
    pub governor: RuntimeGovernor<P>,
    last_sequence: i64,
    acknowledged_ids: std::collections::HashSet<String>,
    safe_state: SafeState,
    last_sensor_hash: Option<String>,
}

impl RuntimeSession<SimPlant> {
    pub fn start_sim(
        args: StartArgs,
        plant: SimPlant,
        now_s: f64,
    ) -> Result<Self, SessionStartError> {
        Self::start(args, plant, now_s)
    }
}

impl<P: Plant> RuntimeSession<P> {
    pub fn start(args: StartArgs, plant: P, now_s: f64) -> Result<Self, SessionStartError> {
        if args.mode == RuntimeMode::Online {
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
        }

        let identity = RuntimeIdentity {
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
        };
        if args.mode == RuntimeMode::Online && !identity.complete_online() {
            return Err(SessionStartError(
                "incomplete_online_runtime_identity".into(),
            ));
        }

        let mut governor = RuntimeGovernor::new(identity, plant);
        governor.config.require_online_identity = args.mode == RuntimeMode::Online;
        governor.config.require_sensor_before_write = args.mode != RuntimeMode::Simulation;
        governor.config.require_monotonic_sequence =
            matches!(args.mode, RuntimeMode::Online | RuntimeMode::Hil);
        governor.config.require_command_signature = args.mode == RuntimeMode::Online;
        governor.config.require_sensor_packet_hash = args.mode == RuntimeMode::Online;
        let mut env = DriverEnvelopePack::from_max_action(
            &governor.plant.caps().max_action,
            args.mode == RuntimeMode::Online,
        );
        if env.max_action_abs <= 0.0 {
            env.max_action_abs = args.max_action_abs;
        }
        governor.envelope = Some(env);
        governor.heartbeat(now_s);

        Ok(Self {
            mode: args.mode,
            governor,
            last_sequence: 0,
            acknowledged_ids: std::collections::HashSet::new(),
            safe_state: SafeState::Running,
            last_sensor_hash: None,
        })
    }

    pub fn start_online(args: StartArgs, plant: P, now_s: f64) -> Result<Self, SessionStartError> {
        let mut args = args;
        args.mode = RuntimeMode::Online;
        args.require_verified_release = Some(true);
        args.require_command_signature = Some(true);
        args.require_driver_envelope = Some(true);
        Self::start(args, plant, now_s)
    }

    pub fn latch_safe_state(&mut self, state: SafeState, _reason: &str) {
        self.safe_state = state;
    }

    pub fn last_sensor_hash(&self) -> Option<&str> {
        self.last_sensor_hash.as_deref()
    }

    pub fn ingest_sensor(
        &mut self,
        samples: &[(String, f64)],
        timestamp_s: Option<f64>,
        now_s: f64,
    ) -> Result<String, String> {
        if samples.is_empty() {
            return Err("sensor_reading_empty".into());
        }
        if self.mode != RuntimeMode::Simulation && timestamp_s.is_none() {
            return Err("sensor_timestamp_required".into());
        }
        let ts = timestamp_s.unwrap_or(now_s);
        if timestamp_s.is_some() && (ts - now_s).abs() > self.governor.config.sensor_stale_s {
            return Err("sensor_timestamp_stale_vs_now".into());
        }
        let canon = serde_json::to_string(samples).unwrap_or_default();
        let hash = hex::encode(Sha256::digest(canon.as_bytes()));
        self.governor.mark_sensor(ts, Some(hash.clone()));
        self.last_sensor_hash = Some(hash.clone());
        Ok(hash)
    }

    pub fn bind_and_dispatch(
        &mut self,
        mut command: CertifiedCommand,
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
        if let Err(errs) = command.bind_identity(
            self.governor.identity.release_hash.as_str(),
            self.governor.identity.design_str(),
            self.governor.identity.calibration_id_str(),
        ) {
            return DispatchResult::refused(self.mode, errs);
        }
        if self.mode == RuntimeMode::Online {
            if command.actuator_ids.is_empty() {
                return DispatchResult::refused(
                    self.mode,
                    vec!["dispatch_requires_actuator_ids".into()],
                );
            }
            if let Some(h) = &self.last_sensor_hash {
                if command.sensor_packet_hash.is_empty() {
                    command.sensor_packet_hash = h.clone();
                } else if command.sensor_packet_hash != *h {
                    return DispatchResult::refused(
                        self.mode,
                        vec!["online_refuses_forged_sensor_packet_hash".into()],
                    );
                }
            }
        }
        command.acknowledge();
        self.acknowledged_ids.insert(command.command_id.clone());
        self.last_sequence = command.sequence.max(self.last_sequence);
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

    pub fn refuse_bare_action(&self, _action: &[f64]) -> DispatchResult {
        DispatchResult::refused(
            self.mode,
            vec!["bind_and_dispatch_requires_CertifiedCommand".into()],
        )
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
