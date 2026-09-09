//! Authority-owned deployment configuration. Not device EEPROM.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Factory XL330 is 57 600. U2D2 benches often use 1 Mbps.
pub const CANDIDATE_BAUDS: &[u32] = &[57_600, 115_200, 1_000_000];

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MetalConfig {
    pub device: PathBuf,
    #[serde(default = "default_servo_id")]
    pub servo_id: u8,
    #[serde(default = "default_baud")]
    pub baud: u32,
    pub release_hash: String,
    /// Deployment/configuration identity. XL330 EEPROM has no calibration record.
    pub calibration_id: String,
    /// Measured USB+servo identity recorded by `probe` / `bind-measured`.
    #[serde(default)]
    pub expected_serial: String,
    /// Measured `xl330:<model>:<fw>` recorded by `probe` / `bind-measured`.
    #[serde(default)]
    pub expected_firmware: String,
    /// Profile velocity register units (1 unit ≈ 0.229 rpm). Keep tiny.
    #[serde(default = "default_profile_velocity")]
    pub max_profile_velocity: u32,
    #[serde(default = "default_profile_accel")]
    pub max_profile_acceleration: u32,
    /// Maximum authorized goal step, in XL330 position ticks.
    #[serde(default = "default_delta_ticks")]
    pub max_position_delta_ticks: i32,
    /// EEPROM current limit (XL330 unit ≈ 1 mA). Keep well below stall.
    #[serde(default = "default_current_limit")]
    pub current_limit_milli: u16,
    /// Reality OS actuator envelope (not a device unit).
    #[serde(default = "default_tau_max")]
    pub tau_max: f64,
    #[serde(default = "default_freshness")]
    pub freshness_threshold_s: f64,
    /// Campaign-only overlays (force_disconnect / hot_swap). Off in production.
    #[serde(default)]
    pub campaign_hooks: bool,
}

fn default_servo_id() -> u8 {
    1
}
fn default_baud() -> u32 {
    57600
}
fn default_profile_velocity() -> u32 {
    20
}
fn default_profile_accel() -> u32 {
    10
}
fn default_delta_ticks() -> i32 {
    8
}
fn default_current_limit() -> u16 {
    200
}
fn default_tau_max() -> f64 {
    0.2
}
fn default_freshness() -> f64 {
    2.0
}

impl MetalConfig {
    pub fn example(device: impl Into<PathBuf>) -> Self {
        Self {
            device: device.into(),
            servo_id: 1,
            baud: 57600,
            release_hash: "rel-metal-xl330-1".into(),
            calibration_id: "xl330-bench-cal-1".into(),
            expected_serial: String::new(),
            expected_firmware: String::new(),
            max_profile_velocity: 20,
            max_profile_acceleration: 10,
            max_position_delta_ticks: 8,
            current_limit_milli: 200,
            tau_max: 0.2,
            freshness_threshold_s: 2.0,
            campaign_hooks: false,
        }
    }

    pub fn apply_process_env(&mut self) {
        if let Ok(d) = std::env::var("REALITYOS_METAL_DEVICE") {
            if !d.trim().is_empty() {
                self.device = PathBuf::from(d);
            }
        }
        if let Ok(b) = std::env::var("REALITYOS_METAL_BAUD") {
            if let Ok(n) = b.parse::<u32>() {
                if n > 0 {
                    self.baud = n;
                }
            }
        }
        if let Ok(id) = std::env::var("REALITYOS_METAL_SERVO_ID") {
            if let Ok(n) = id.parse::<u8>() {
                if n != 0 && n != 254 {
                    self.servo_id = n;
                }
            }
        }
    }

    pub fn load(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let raw = std::fs::read_to_string(path)?;
        Ok(serde_json::from_str(&raw)?)
    }

    pub fn save(&self, path: impl AsRef<Path>) -> anyhow::Result<()> {
        if let Some(dir) = path.as_ref().parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, serde_json::to_string_pretty(self)?)?;
        Ok(())
    }

    /// Authority-owned configuration artifact. Not read from EEPROM.
    pub fn design_content_hash(&self) -> String {
        let canon = serde_json::json!({
            "schema": "realityos.metal_design/1",
            "device_kind": "xl330",
            "servo_id": self.servo_id,
            "baud": self.baud,
            "calibration_id": self.calibration_id,
            "max_profile_velocity": self.max_profile_velocity,
            "max_profile_acceleration": self.max_profile_acceleration,
            "max_position_delta_ticks": self.max_position_delta_ticks,
            "current_limit_milli": self.current_limit_milli,
            "tau_max": self.tau_max,
        });
        hex::encode(Sha256::digest(
            serde_json::to_string(&canon).unwrap_or_default().as_bytes(),
        ))
    }

    pub fn actuator_id(&self) -> String {
        format!("xl330:{}", self.servo_id)
    }

    pub fn expected_ready(&self) -> bool {
        !self.expected_serial.trim().is_empty() && !self.expected_firmware.trim().is_empty()
    }
}

/// Device path for autonomy os-probe. Env wins so a 0700 `metal.json` still records attempts.
pub fn resolve_probe_device(root: impl AsRef<Path>) -> PathBuf {
    if let Ok(d) = std::env::var("REALITYOS_METAL_DEVICE") {
        if !d.trim().is_empty() {
            return PathBuf::from(d);
        }
    }
    MetalConfig::load(root.as_ref().join(CONFIG_FILE))
        .ok()
        .map(|c| c.device)
        .unwrap_or_default()
}

pub const CONFIG_FILE: &str = "metal.json";
pub const MEASURED_FILE: &str = "measured.json";
pub const SIGNING_KEY_FILE: &str = "signing.key";
pub const JOURNAL: &str = "driver.jsonl";
pub const IPC_SOCK: &str = "ipc.sock";
pub const BUS_DIR: &str = "bus";
pub const WRITES_FILE: &str = "writes";
pub const ACKS_FILE: &str = "acks";
pub const EGRESS_LOG: &str = "egress.jsonl";
pub const LOCK_FILE: &str = "actuator.lock";
pub const PRESENT_FILE: &str = "present";
pub const GOAL_FILE: &str = "goal";
pub const VIN_FILE: &str = "vin";
pub const FRESHNESS_FILE: &str = "sensor_freshness.json";
pub const IPC_SOCKET_MODE: u32 = 0o660;

pub fn candidate_bauds(configured: u32, extra: Option<u32>) -> Vec<u32> {
    let mut out = Vec::new();
    for b in std::iter::once(configured)
        .chain(extra)
        .chain(CANDIDATE_BAUDS.iter().copied())
    {
        if b > 0 && !out.contains(&b) {
            out.push(b);
        }
    }
    out
}

pub fn candidate_servo_ids(configured: u8, extra: Option<u8>) -> Vec<u8> {
    let mut out = Vec::new();
    for id in std::iter::once(configured).chain(extra).chain([1_u8, 2]) {
        if id != 0 && id != 254 && !out.contains(&id) {
            out.push(id);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn candidate_bauds_keep_configured_first_and_dedup() {
        let b = candidate_bauds(1_000_000, Some(57_600));
        assert_eq!(b[0], 1_000_000);
        assert_eq!(b.iter().filter(|x| **x == 1_000_000).count(), 1);
        assert!(b.contains(&57_600));
        assert!(b.contains(&115_200));
    }

    #[test]
    fn resolve_probe_device_empty_without_config() {
        if std::env::var("REALITYOS_METAL_DEVICE")
            .ok()
            .filter(|s| !s.trim().is_empty())
            .is_some()
        {
            return;
        }
        let p = resolve_probe_device("/tmp/realityos-metal-no-such-root");
        assert!(p.as_os_str().is_empty());
    }

    #[test]
    fn resolve_probe_device_reads_metal_json_when_env_unset() {
        if std::env::var("REALITYOS_METAL_DEVICE")
            .ok()
            .filter(|s| !s.trim().is_empty())
            .is_some()
        {
            return;
        }
        let dir =
            std::env::temp_dir().join(format!("realityos-metal-probe-cfg-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let cfg = MetalConfig::example("/dev/ttyUSB9");
        cfg.save(dir.join(CONFIG_FILE)).unwrap();
        assert_eq!(resolve_probe_device(&dir), PathBuf::from("/dev/ttyUSB9"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn candidate_ids_skip_broadcast_and_zero() {
        let ids = candidate_servo_ids(7, Some(0));
        assert_eq!(ids[0], 7);
        assert!(!ids.contains(&0));
        assert!(!ids.contains(&254));
        assert!(ids.contains(&1));
    }
}
