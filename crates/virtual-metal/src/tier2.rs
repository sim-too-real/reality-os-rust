//! Production-path Tier 2: decide → ONLINE governor → Xl330Driver → PTY → device.
//! Privileged device truth is test-oracle only.

use serde::{Deserialize, Serialize};

use crate::faults::FaultSchedule;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tier1Counterexample {
    pub seed: u64,
    pub fault_sequence: FaultSchedule,
    pub realization: serde_json::Value,
    pub production_path_relevant: bool,
    pub note: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Tier2ReplaySpec {
    pub seed: u64,
    pub fault_sequence: FaultSchedule,
    pub realization: serde_json::Value,
}

/// Promote a minimized Tier-1 case onto the production serial/driver path
/// only when the fault is production-path relevant.
pub fn promote_to_tier2(cx: &Tier1Counterexample) -> Option<Tier2ReplaySpec> {
    if !cx.production_path_relevant {
        return None;
    }
    Some(Tier2ReplaySpec {
        seed: cx.seed,
        fault_sequence: cx.fault_sequence.clone(),
        realization: cx.realization.clone(),
    })
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Tier2Trace {
    pub authority_verdict: String,
    pub driver_verdict: String,
    pub integrity_aborted: bool,
    pub physical_actions_before: u64,
    pub physical_actions_after: u64,
    pub present_before: i32,
    pub present_after: i32,
    pub applied_target: Option<i32>,
    pub tx_count: usize,
    pub rx_count: usize,
    pub outcome_class: String,
}

#[cfg(unix)]
mod unix {
    use super::Tier2Trace;
    use std::path::{Path, PathBuf};
    use std::sync::{Arc, Mutex};

    use realityos_governor::RuntimeTrace;
    use realityos_governor::{OnlineLocked, RuntimeGovernor, RuntimeIdentity};
    use realityos_kernel::{
        AuthorityClock, CalibrationId, DesignContentHash, FakeClock, FirmwareId, ReleaseHash,
        SerialOrAsBuilt,
    };
    use realityos_metal::config::MetalConfig;
    use realityos_metal::xl330::Xl330Driver;
    use realityos_plant::{ActionParams, HardwareBackedPlant, HardwareDriverPort};

    use crate::campaign::decide_hold;
    use crate::device::VirtualXl330;
    use crate::faults::FaultSchedule;
    use crate::oracle::{belief_from_trace, check_belief_vs_truth, class_from_trace, DeviceTruth};
    use crate::peer::{SharedXl330, VirtualSerialPeer};
    use crate::pty::VirtualXl330Pty;

    type Plant = HardwareBackedPlant<Xl330Driver>;
    type Gov = RuntimeGovernor<Plant, OnlineLocked>;

    fn metal_root(name: &str) -> PathBuf {
        let base = if Path::new("/dev/shm").is_dir() {
            PathBuf::from("/dev/shm")
        } else {
            std::env::temp_dir()
        };
        let dir = base.join(format!("realityos-t2-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let _ = std::fs::create_dir_all(&dir);
        dir
    }

    pub struct Tier2Session {
        pub gov: Gov,
        pub device: SharedXl330,
        pub peer: Arc<Mutex<VirtualSerialPeer>>,
        pty: Option<VirtualXl330Pty>,
        root: PathBuf,
        tag: String,
        seed: u64,
    }

    impl Tier2Session {
        pub fn open(device: VirtualXl330, tag: &str, seed: u64) -> Result<Self, String> {
            Self::open_with_faults(device, tag, seed, FaultSchedule::empty())
        }

        pub fn open_with_faults(
            device: VirtualXl330,
            tag: &str,
            seed: u64,
            faults: FaultSchedule,
        ) -> Result<Self, String> {
            let device = Arc::new(Mutex::new(device));
            let (pty, peer) = VirtualXl330Pty::spawn_shared(device.clone(), true, faults)
                .map_err(|e| format!("pty:{e}"))?;
            let root = metal_root(tag);
            let mut cfg = MetalConfig::example(pty.slave_path());
            cfg.campaign_hooks = false;
            let driver = Xl330Driver::open(cfg, &root).map_err(|e| format!("open:{e}"))?;
            let measured = driver.probe_identity();
            if !measured.connected {
                return Err("tier2_identity_disconnected".into());
            }
            let identity = RuntimeIdentity {
                release_hash: ReleaseHash::new("rel-metal-xl330-1").map_err(|e| e.to_string())?,
                design_content_hash: Some(
                    DesignContentHash::new(measured.design_content_hash.clone())
                        .map_err(|e| e.to_string())?,
                ),
                serial_or_as_built: Some(
                    SerialOrAsBuilt::new(measured.serial.clone()).map_err(|e| e.to_string())?,
                ),
                firmware_id: Some(
                    FirmwareId::new(measured.firmware_id.clone()).map_err(|e| e.to_string())?,
                ),
                calibration_id: Some(
                    CalibrationId::new(measured.calibration_id.clone())
                        .map_err(|e| e.to_string())?,
                ),
            };
            let acts = if measured.actuator_ids.is_empty() {
                vec!["xl330:1".into()]
            } else {
                measured.actuator_ids.clone()
            };
            let plant = Plant::new(driver, "xl330-t2", 1, 0.2);
            let clock: Arc<dyn AuthorityClock> = FakeClock::arc(10.0);
            let journal = root.join("driver.jsonl");
            let mut gov = Gov::new_online(
                identity,
                plant,
                journal,
                b"virtual-metal-key".to_vec(),
                true,
                acts,
                clock,
            )
            .map_err(|e| format!("online:{e:?}"))?;
            gov.acquire_sensor().map_err(|e| format!("sensor:{e}"))?;
            Ok(Self {
                gov,
                device,
                peer,
                pty: Some(pty),
                root,
                tag: tag.into(),
                seed,
            })
        }

        pub fn truth(&self) -> DeviceTruth {
            let d = self.device.lock().expect("oracle");
            let p = self.peer.lock().expect("peer");
            DeviceTruth::from_device_peer(&d, Some(&p))
        }

        pub fn inject(&self, faults: FaultSchedule) {
            self.peer
                .lock()
                .expect("peer")
                .set_transport_schedule(faults);
        }

        pub fn inject_next(&self, kind: crate::faults::FaultKind) {
            let n = self
                .peer
                .lock()
                .expect("peer")
                .packet_count()
                .saturating_add(1);
            self.inject(crate::faults::FaultSchedule {
                events: vec![crate::faults::FaultEvent {
                    after_packet: n,
                    kind,
                }],
            });
        }

        pub fn hold(&mut self) -> (RuntimeTrace, DeviceTruth, DeviceTruth) {
            let (r, before, after) = self.try_hold(1);
            (r.expect("authorize hold"), before, after)
        }

        pub fn try_hold(
            &mut self,
            seq: i64,
        ) -> (Result<RuntimeTrace, Vec<String>>, DeviceTruth, DeviceTruth) {
            let before = self.truth();
            let issued = decide_hold(seq, 10.0);
            let after_auth = match self.gov.authorize_issued(issued) {
                Ok(w) => Ok(self.gov.write_online_now(&w, &ActionParams::empty())),
                Err(e) => Err(e),
            };
            let after = self.truth();
            (after_auth, before, after)
        }

        pub fn restart(&mut self) -> Result<(), String> {
            drop(self.pty.take());
            let (pty, peer) =
                VirtualXl330Pty::spawn_shared(self.device.clone(), true, FaultSchedule::empty())
                    .map_err(|e| format!("pty2:{e}"))?;
            let new_root = metal_root(&format!("{}-rst", self.tag));
            if let Ok(entries) = std::fs::read_dir(&self.root) {
                for e in entries.flatten() {
                    let from = e.path();
                    let to = new_root.join(e.file_name());
                    let _ = std::fs::copy(&from, &to);
                }
            }
            let dst = new_root.join("driver.jsonl");
            let mut cfg = MetalConfig::example(pty.slave_path());
            cfg.campaign_hooks = false;
            let driver = Xl330Driver::open(cfg, &new_root).map_err(|e| format!("reopen:{e}"))?;
            let measured = driver.probe_identity();
            let identity = RuntimeIdentity {
                release_hash: ReleaseHash::new("rel-metal-xl330-1").map_err(|e| e.to_string())?,
                design_content_hash: Some(
                    DesignContentHash::new(measured.design_content_hash.clone())
                        .map_err(|e| e.to_string())?,
                ),
                serial_or_as_built: Some(
                    SerialOrAsBuilt::new(measured.serial.clone()).map_err(|e| e.to_string())?,
                ),
                firmware_id: Some(
                    FirmwareId::new(measured.firmware_id.clone()).map_err(|e| e.to_string())?,
                ),
                calibration_id: Some(
                    CalibrationId::new(measured.calibration_id.clone())
                        .map_err(|e| e.to_string())?,
                ),
            };
            let acts = if measured.actuator_ids.is_empty() {
                vec!["xl330:1".into()]
            } else {
                measured.actuator_ids.clone()
            };
            let plant = Plant::new(driver, "xl330-t2", 1, 0.2);
            let clock: Arc<dyn AuthorityClock> = FakeClock::arc(10.0);
            let mut gov = Gov::new_online(
                identity,
                plant,
                dst,
                b"virtual-metal-key".to_vec(),
                false,
                acts,
                clock,
            )
            .map_err(|e| format!("restart_online:{e:?}"))?;
            let _ = gov.acquire_sensor();
            let old_root = std::mem::replace(&mut self.root, new_root);
            self.gov = gov;
            self.peer = peer;
            self.pty = Some(pty);
            let _ = std::fs::remove_dir_all(old_root);
            Ok(())
        }

        pub fn trace_row(
            &self,
            t: &RuntimeTrace,
            before: &DeviceTruth,
            after: &DeviceTruth,
        ) -> Tier2Trace {
            let connected = self.peer.lock().expect("peer").is_connected();
            Tier2Trace {
                authority_verdict: t.event.clone(),
                driver_verdict: t.event.clone(),
                integrity_aborted: self.gov.integrity_aborted(),
                physical_actions_before: before.physical_actions,
                physical_actions_after: after.physical_actions,
                present_before: before.present,
                present_after: after.present,
                applied_target: Some(after.goal),
                tx_count: after.tx_count,
                rx_count: after.rx_count,
                outcome_class: class_from_trace(t.ok, &t.event, connected).as_str().into(),
            }
        }

        pub fn a1(
            &self,
            t: &RuntimeTrace,
            before: &DeviceTruth,
            after: &DeviceTruth,
        ) -> Result<(), String> {
            check_belief_vs_truth(belief_from_trace(t.ok, &t.event), before, after)
        }
    }

    impl Drop for Tier2Session {
        fn drop(&mut self) {
            if let Some(pty) = self.pty.take() {
                pty.stop();
            }
            let _ = std::fs::remove_dir_all(&self.root);
            let _ = self.tag;
            let _ = self.seed;
        }
    }

    pub use Tier2Session as Session;
}

#[cfg(unix)]
pub use unix::Session as Tier2Session;
