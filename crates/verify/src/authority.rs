//! Reality OS simulation authority boundary.
//! Policy → ActionProposal → decide → IssuedCommand → session write → plant.
//! No simulator-only bypass. metal remains false.

use crate::honesty::SIMULATION_ONLY;
use crate::normalize::RobotManifest;
use crate::observation::PolicyObservation;
use crate::policy::ActionProposal;
use crate::task::TaskSpec;
use realityos_core::{DecideRequest, Intent, KernelDecision, PolicyProposal, RealityOs, WorldView};
use realityos_kernel::DecisionStatus;
use realityos_plant::{
    ActionParams, ActuationCommand, CommandLedger, HardwareBackedPlant, HardwareDriverPort, Plant,
};
use realityos_session::{DispatchResult, RuntimeSession, StartArgs};
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum AuthorityOutcome {
    Allowed,
    Refused,
    Aborted,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthorityRecord {
    pub outcome: AuthorityOutcome,
    pub status: String,
    pub physical_reason: String,
    pub command_id: Option<String>,
    pub violations: Vec<String>,
    pub executed: bool,
    pub metal: bool,
    pub evidence_status: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CommandLease {
    pub command_id: String,
    pub control_mode: String,
    pub issued_at: f64,
    pub expires_at: f64,
    pub authorized_ctrl: Vec<f64>,
}

struct LedgerCmd {
    id: String,
    sequence: i64,
    issued_at_s: f64,
    expires_at_s: f64,
    action: Vec<f64>,
    payload_hash: String,
}

impl ActuationCommand for LedgerCmd {
    fn command_id(&self) -> &str {
        &self.id
    }
    fn sequence(&self) -> i64 {
        self.sequence
    }
    fn issued_at_s(&self) -> f64 {
        self.issued_at_s
    }
    fn expires_at_s(&self) -> f64 {
        self.expires_at_s
    }
    fn allowed_action(&self) -> &[f64] {
        &self.action
    }
    fn issuer_allowed_action(&self) -> &[f64] {
        &self.action
    }
    fn certificate_status(&self) -> DecisionStatus {
        DecisionStatus::Allow
    }
    fn issuer_certificate_status(&self) -> &str {
        "allow"
    }
    fn issuer_physical_reason(&self) -> &str {
        ""
    }
    fn release_hash(&self) -> &str {
        "sim-verify"
    }
    fn as_built_hash(&self) -> &str {
        ""
    }
    fn calibration_ids(&self) -> &[String] {
        &[]
    }
    fn acknowledged(&self) -> bool {
        true
    }
    fn sensor_snapshot_id(&self) -> &str {
        ""
    }
    fn sensor_packet_hash(&self) -> &str {
        ""
    }
    fn belief_snapshot_id(&self) -> &str {
        ""
    }
    fn payload_hash(&self) -> &str {
        &self.payload_hash
    }
    fn signature(&self) -> &str {
        ""
    }
    fn signer(&self) -> &str {
        ""
    }
    fn signing_scheme(&self) -> &str {
        ""
    }
    fn actuator_ids(&self) -> &[String] {
        &[]
    }
}

pub struct SimAuthority<P: HardwareDriverPort> {
    ros: RealityOs,
    pub session: RuntimeSession<HardwareBackedPlant<P>>,
    consume: CommandLedger,
    journal_path: PathBuf,
    seen_observation_ids: std::collections::HashSet<String>,
    last_obs_ts: Option<f64>,
    sequence: i64,
    pub freshness_s: f64,
    pub command_lifetime_s: f64,
    pub lease: Option<CommandLease>,
}

impl<P: HardwareDriverPort> SimAuthority<P> {
    pub fn open(
        plant: HardwareBackedPlant<P>,
        manifest: &RobotManifest,
        now_s: f64,
    ) -> Result<Self, String> {
        Self::open_with_journal(plant, manifest, now_s, None)
    }

    pub fn open_with_journal(
        plant: HardwareBackedPlant<P>,
        manifest: &RobotManifest,
        now_s: f64,
        journal: Option<PathBuf>,
    ) -> Result<Self, String> {
        let mut args = StartArgs::simulation(format!("sim-{}", manifest.model_hash));
        args.design_content_hash = manifest.model_hash.clone();
        args.max_action_abs = manifest.tau_max().into_iter().fold(1.0, f64::max);
        let start_s = if now_s <= 0.0 { 10.0 } else { now_s };
        let session = RuntimeSession::start(args, plant, start_s).map_err(|e| e.0)?;
        let journal_path = journal.unwrap_or_else(|| {
            std::env::temp_dir().join(format!(
                "realityos-sim-consume-{}-{}-{}.jsonl",
                manifest.robot_id,
                &manifest.model_hash[..8.min(manifest.model_hash.len())],
                std::process::id()
            ))
        });
        let consume =
            CommandLedger::with_journal(&journal_path, false).map_err(|e| e.to_string())?;
        let lease = restore_lease_from_path(&journal_path).or_else(|| restore_lease(&consume));
        Ok(Self {
            ros: RealityOs::new(),
            session,
            consume,
            journal_path,
            seen_observation_ids: std::collections::HashSet::new(),
            last_obs_ts: None,
            sequence: 0,
            freshness_s: 0.25,
            command_lifetime_s: 1.0,
            lease,
        })
    }

    pub fn plant_write_count(&self) -> u32 {
        self.session.governor.plant().write_count()
    }

    pub fn journal_path(&self) -> &PathBuf {
        &self.journal_path
    }

    pub fn restart(
        &mut self,
        plant: HardwareBackedPlant<P>,
        manifest: &RobotManifest,
        now_s: f64,
    ) -> Result<(), String> {
        let journal = self.journal_path.clone();
        let freshness = self.freshness_s;
        let lifetime = self.command_lifetime_s;
        let next = Self::open_with_journal(plant, manifest, now_s, Some(journal))?;
        self.ros = next.ros;
        self.session = next.session;
        self.consume = next.consume;
        self.journal_path = next.journal_path;
        self.lease = next.lease;
        self.freshness_s = freshness;
        self.command_lifetime_s = lifetime;
        Ok(())
    }

    pub fn lease_expired(&self, now_s: f64) -> bool {
        self.lease
            .as_ref()
            .map(|l| now_s >= l.expires_at)
            .unwrap_or(false)
    }

    pub fn safe_ctrl_after_expiry(&self, manifest: &RobotManifest) -> Option<Vec<f64>> {
        let lease = self.lease.as_ref()?;
        let nu = manifest.nu.max(0) as usize;
        let mode = lease.control_mode.as_str();
        if mode == "position" || manifest.position_mask().iter().any(|p| *p) {
            let mut hold = lease.authorized_ctrl.clone();
            hold.resize(nu, 0.0);
            Some(hold)
        } else {
            Some(vec![0.0; nu])
        }
    }

    pub fn screen_proposal(
        &self,
        proposal: &ActionProposal,
        manifest: &RobotManifest,
        obs: &PolicyObservation,
        now_s: f64,
        replay_command_id: Option<&str>,
    ) -> Result<(), Vec<String>> {
        let mut v = Vec::new();
        if proposal.robot_id != manifest.robot_id {
            v.push("WRONG_ROBOT_IDENTITY".into());
        }
        if proposal.model_hash != manifest.model_hash {
            v.push("WRONG_ROBOT_IDENTITY".into());
        }
        if proposal.episode_id != obs.episode_id {
            v.push("episode_mismatch".into());
        }
        if now_s - proposal.observation_timestamp > self.freshness_s {
            v.push("STALE_ACTION".into());
            v.push("observation_freshness".into());
        }
        if self.seen_observation_ids.contains(&proposal.observation_id) {
            v.push("STALE_OBSERVATION".into());
        }
        if obs.qpos.is_empty() && proposal.action.iter().any(|x| x.abs() > 0.0) {
            v.push("SENSOR_DROPOUT".into());
            v.push("INCOMPLETE_OBSERVATION".into());
        }
        let cid = replay_command_id
            .map(|s| s.to_string())
            .unwrap_or_else(|| format!("{}:{}", proposal.episode_id, proposal.observation_id));
        if !self.consume.command_phase(&cid).may_attempt_write() {
            v.push("COMMAND_REPLAY".into());
            v.push("DUPLICATE_COMMAND".into());
        }
        if proposal.action.iter().any(|x| !x.is_finite()) {
            v.push("non_finite_action".into());
        }
        if v.is_empty() {
            Ok(())
        } else {
            Err(v)
        }
    }

    pub fn decide_and_maybe_write(
        &mut self,
        proposal: &ActionProposal,
        manifest: &RobotManifest,
        task: &TaskSpec,
        obs: &PolicyObservation,
        now_s: f64,
        forced_command_id: Option<String>,
        force_verb: Option<&str>,
    ) -> AuthorityRecord {
        let command_id = forced_command_id
            .unwrap_or_else(|| format!("{}:{}", proposal.episode_id, proposal.observation_id));
        if let Err(violations) =
            self.screen_proposal(proposal, manifest, obs, now_s, Some(&command_id))
        {
            return AuthorityRecord {
                outcome: AuthorityOutcome::Refused,
                status: DecisionStatus::Refuse.as_str().into(),
                physical_reason: violations.join(","),
                command_id: Some(command_id),
                violations,
                executed: false,
                metal: false,
                evidence_status: SIMULATION_ONLY.into(),
            };
        }

        let now_s = if now_s <= 0.0 { 10.0 } else { now_s };
        self.session.governor.heartbeat(now_s);
        let _ = self.session.governor.watchdog_tick(now_s);
        self.sequence += 1;
        let verb = force_verb.unwrap_or_else(|| task.verb());
        let mut intent = Intent::language(task.id(), verb);
        if verb == "place" || verb == "pick" {
            intent.require_scene = true;
        }
        let world = WorldView {
            tau_max: manifest.tau_max(),
            q: obs.qpos.clone(),
            q_min: manifest.q_min(),
            q_max: manifest.q_max(),
            observation: None,
            ..WorldView::default()
        };
        let mut req = DecideRequest::new(intent, world, now_s);
        req.command_id = command_id.clone();
        req.sequence = self.sequence;
        req.ttl_s = self.command_lifetime_s;
        req.proposal = Some(PolicyProposal::external_deterministic(
            proposal.action.clone(),
            &proposal.policy_id,
        ));
        let decision: KernelDecision = self.ros.decide(req);
        if !decision.allowed {
            return AuthorityRecord {
                outcome: if decision.status == DecisionStatus::Abort {
                    AuthorityOutcome::Aborted
                } else {
                    AuthorityOutcome::Refused
                },
                status: decision.status.as_str().into(),
                physical_reason: decision.physical_reason,
                command_id: None,
                violations: vec!["AUTHORITY_VIOLATION".into()],
                executed: false,
                metal: false,
                evidence_status: SIMULATION_ONLY.into(),
            };
        }
        let Some(issued) = decision.command else {
            return AuthorityRecord {
                outcome: AuthorityOutcome::Refused,
                status: "refuse".into(),
                physical_reason: "no_issued_command".into(),
                command_id: None,
                violations: vec!["AUTHORITY_VIOLATION".into()],
                executed: false,
                metal: false,
                evidence_status: SIMULATION_ONLY.into(),
            };
        };
        let cmd = issued.into_command();
        let horizon = if proposal.requested_horizon_s > 0.0 {
            proposal.requested_horizon_s
        } else {
            self.command_lifetime_s
        };
        let ledger_cmd = LedgerCmd {
            id: command_id.clone(),
            sequence: self.sequence,
            issued_at_s: now_s,
            expires_at_s: now_s + horizon,
            action: proposal.action.clone(),
            payload_hash: command_id.clone(),
        };
        if let Err(e) = self.consume.prepare(&ledger_cmd) {
            return AuthorityRecord {
                outcome: AuthorityOutcome::Refused,
                status: DecisionStatus::Refuse.as_str().into(),
                physical_reason: e.to_string(),
                command_id: Some(command_id),
                violations: vec!["COMMAND_REPLAY".into()],
                executed: false,
                metal: false,
                evidence_status: SIMULATION_ONLY.into(),
            };
        }
        let writes_before = self.plant_write_count();
        let dispatch: DispatchResult =
            self.session
                .bind_and_dispatch(cmd, &ActionParams::empty(), now_s);
        let executed = dispatch.ok && self.plant_write_count() > writes_before;
        if executed {
            let _ = self.consume.ack(&ledger_cmd);
            self.seen_observation_ids
                .insert(proposal.observation_id.clone());
            self.last_obs_ts = Some(proposal.observation_timestamp);
            self.lease = Some(CommandLease {
                command_id: command_id.clone(),
                control_mode: proposal.control_mode.clone(),
                issued_at: now_s,
                expires_at: now_s + horizon,
                authorized_ctrl: proposal.action.clone(),
            });
            persist_lease(&self.journal_path, self.lease.as_ref());
        } else {
            let _ = self.consume.mark_unknown(&ledger_cmd);
        }
        AuthorityRecord {
            outcome: if executed {
                AuthorityOutcome::Allowed
            } else {
                AuthorityOutcome::Refused
            },
            status: if executed {
                "allow".into()
            } else {
                "refuse".into()
            },
            physical_reason: dispatch.violations.join(","),
            command_id: Some(command_id),
            violations: dispatch.violations,
            executed,
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
        }
    }
}

/// Replay/restart injection may only reuse a command that actually executed.
/// A refused first write must not mint a fresh drive under a never-spent id.
pub fn spent_command_for_replay(executed: bool, command_id: Option<&str>) -> Option<String> {
    if executed {
        command_id.filter(|s| !s.is_empty()).map(str::to_string)
    } else {
        None
    }
}

fn persist_lease(journal: &Path, lease: Option<&CommandLease>) {
    if let Some(lease) = lease {
        if let Ok(s) = serde_json::to_string(lease) {
            let sidecar = journal.with_extension("lease.json");
            let _ = std::fs::write(sidecar, s);
        }
    }
}

fn restore_lease(consume: &CommandLedger) -> Option<CommandLease> {
    let _ = consume;
    None
}

fn restore_lease_from_path(journal: &Path) -> Option<CommandLease> {
    let sidecar = journal.with_extension("lease.json");
    let raw = std::fs::read_to_string(sidecar).ok()?;
    serde_json::from_str(&raw).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::driver::RecordingSimPort;
    use crate::normalize::{DerivedInterface, RobotManifest};
    use crate::observation::{PolicyObservation, VisionMode};
    use crate::task::TaskSpec;

    fn manifest() -> RobotManifest {
        RobotManifest {
            robot_id: "arm".into(),
            nq: 1,
            nv: 1,
            nu: 1,
            nbody: 2,
            njoint: 1,
            nactuator: 1,
            nsensor: 0,
            ncamera: 0,
            timestep: 0.002,
            joints: vec![],
            actuators: vec![crate::normalize::ActuatorRecord {
                name: "a0".into(),
                transmission_target: "j0".into(),
                control_dimensions: 1,
                ctrlrange: [-1.0, 1.0],
                ctrllimited: true,
                force_range: None,
                actuator_type: "fixed".into(),
                transmission_kind: "joint".into(),
            }],
            sensors: vec![],
            cameras: vec![],
            bodies: vec![],
            sites: vec![],
            site_records: vec![],
            derived: DerivedInterface {
                base_type: crate::bundle::BaseType::Fixed,
                actuated_dofs: vec!["j0".into()],
                passive_dofs: vec![],
                end_effector_chains: vec![],
                end_effector_joint_chains: vec![],
                actuator_coverage: 1.0,
                potentially_uncontrollable_joints: vec![],
            },
            model_hash: "hash-arm".into(),
            source_hash: "src".into(),
            mujoco_version: "3".into(),
            source_format: "mjcf".into(),
            lost_features: vec![],
            support_bodies: vec![],
            collision_groups: Default::default(),
            geoms: Vec::new(),
            metal: false,
            evidence_status: SIMULATION_ONLY.into(),
        }
    }

    fn obs(m: &RobotManifest) -> PolicyObservation {
        PolicyObservation {
            robot_id: m.robot_id.clone(),
            model_hash: m.model_hash.clone(),
            episode_id: "ep1".into(),
            observation_id: "obs1".into(),
            timestamp_s: 1.0,
            task_id: "hold".into(),
            task_instruction: "hold".into(),
            mode: VisionMode::State,
            qpos: vec![0.0],
            qvel: vec![0.0],
            action_dim: 1,
            detections: vec![],
            contact_pairs: vec![],
            rgb: None,
            depth: None,
            camera_status: None,
            goal_xyz: None,
            goal_q: None,
        }
    }

    #[test]
    fn refuse_does_not_actuate_recording_plant() {
        let m = manifest();
        let port = RecordingSimPort::new(1);
        let probe = port.probe.clone();
        let plant = HardwareBackedPlant::new(port, "arm", 1, 1.0);
        let mut auth = SimAuthority::open(plant, &m, 1.0).unwrap();
        let o = obs(&m);
        let mut p = ActionProposal {
            robot_id: "other_robot".into(),
            model_hash: m.model_hash.clone(),
            episode_id: o.episode_id.clone(),
            observation_id: o.observation_id.clone(),
            observation_timestamp: 1.0,
            task_id: "hold".into(),
            action: vec![0.1],
            control_mode: "effort".into(),
            requested_horizon_s: 0.02,
            confidence: None,
            policy_id: "scripted".into(),
            policy_version: "1".into(),
        };
        let rec = auth.decide_and_maybe_write(
            &p,
            &m,
            &TaskSpec::Hold { duration_s: 0.1 },
            &o,
            1.0,
            None,
            None,
        );
        assert_eq!(rec.outcome, AuthorityOutcome::Refused);
        assert!(!rec.executed);
        assert_eq!(probe.snapshot().write_count, 0);
        assert_eq!(auth.plant_write_count(), 0);

        p.robot_id = m.robot_id.clone();
        let rec = auth.decide_and_maybe_write(
            &p,
            &m,
            &TaskSpec::Hold { duration_s: 0.1 },
            &o,
            1.0,
            None,
            None,
        );
        assert!(rec.executed, "{:?}", rec);
        assert_eq!(probe.snapshot().write_count, 1);

        let rec = auth.decide_and_maybe_write(
            &p,
            &m,
            &TaskSpec::Hold { duration_s: 0.1 },
            &o,
            1.0,
            None,
            None,
        );
        assert_eq!(rec.outcome, AuthorityOutcome::Refused);
        assert!(rec
            .violations
            .iter()
            .any(|v| v.contains("REPLAY") || v.contains("DUPLICATE")));
        assert_eq!(probe.snapshot().write_count, 1);
    }

    #[test]
    fn refused_write_is_not_a_spent_command_for_replay() {
        assert!(spent_command_for_replay(false, Some("ep:obs-0")).is_none());
        assert!(spent_command_for_replay(true, None).is_none());
        assert_eq!(
            spent_command_for_replay(true, Some("ep:obs-0")).as_deref(),
            Some("ep:obs-0")
        );
    }

    #[test]
    fn wrong_task_authority_place_without_scene_refuses() {
        let m = manifest();
        let port = RecordingSimPort::new(1);
        let probe = port.probe.clone();
        let plant = HardwareBackedPlant::new(port, "arm", 1, 1.0);
        let mut auth = SimAuthority::open(plant, &m, 1.0).unwrap();
        let o = obs(&m);
        let p = ActionProposal {
            robot_id: m.robot_id.clone(),
            model_hash: m.model_hash.clone(),
            episode_id: o.episode_id.clone(),
            observation_id: "obs-place".into(),
            observation_timestamp: 1.0,
            task_id: "place".into(),
            action: vec![0.1],
            control_mode: "effort".into(),
            requested_horizon_s: 0.02,
            confidence: None,
            policy_id: "scripted".into(),
            policy_version: "1".into(),
        };
        let rec = auth.decide_and_maybe_write(
            &p,
            &m,
            &TaskSpec::Place {
                object: "cube_1".into(),
                target_region: "tray_A".into(),
            },
            &o,
            1.0,
            None,
            Some("place"),
        );
        assert_eq!(rec.outcome, AuthorityOutcome::Refused);
        assert_eq!(probe.snapshot().write_count, 0);
    }
}
