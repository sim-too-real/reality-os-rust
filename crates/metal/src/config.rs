//! Authority-owned deployment configuration. Not device EEPROM.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Factory XL330 is 57 600. Automatic scan also tries 115 200, 1 Mbps, and
/// Wizard 9 600 last. 1 Mbps and Wizard 2 / 3 / 4 Mbps can wedge a
/// CH340/CP2102 (datasheet max ~2 Mbps) after a DTR-RESET miss, so those
/// opens must not follow a factory miss without another 57 600 retry.
pub const CANDIDATE_BAUDS: &[u32] = &[57_600, 115_200, 1_000_000, 9_600];
/// Only added when configured or `REALITYOS_METAL_BAUD` is already 2 Mbps.
pub const FAST_WIZARD_BAUDS: &[u32] = &[2_000_000];
/// Only added when configured or `REALITYOS_METAL_BAUD` is already 3 or 4 Mbps.
pub const HIGH_WIZARD_BAUDS: &[u32] = &[3_000_000, 4_000_000];

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
    /// Session absolute cage around startup Present Position (ticks).
    /// Per-command delta is insufficient: repeated valid nudges accumulate.
    #[serde(default = "default_total_excursion_ticks")]
    pub max_total_excursion_ticks: i32,
    /// Position-mode PWM Limit(36) cap. Raw 0..=885; percent ≈ raw * 0.113.
    /// Output/PWM cap, not a certified torque limit. Current Limit is not
    /// the Position Mode torque boundary.
    #[serde(default = "default_max_pwm_limit_raw")]
    pub max_pwm_limit_raw: u16,
    /// EEPROM current limit (XL330 unit ≈ 1 mA). Keep well below stall.
    /// Configured, but not the Position Mode output/torque boundary.
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
    32
}
/// First experiment: one 32-tick nudge fits; 100 accumulated nudges cannot.
fn default_total_excursion_ticks() -> i32 {
    48
}
fn default_max_pwm_limit_raw() -> u16 {
    crate::protocol::CONSERVATIVE_PWM_LIMIT
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
            max_position_delta_ticks: 32,
            max_total_excursion_ticks: 48,
            max_pwm_limit_raw: crate::protocol::CONSERVATIVE_PWM_LIMIT,
            current_limit_milli: 200,
            tau_max: 0.2,
            freshness_threshold_s: 2.0,
            campaign_hooks: false,
        }
    }

    /// Device path plus optional baud/id hints. Do **not** use this on
    /// `init` / `probe` persist: hints must stay scan extras until
    /// `open_discovering` writes the measured pair.
    pub fn apply_process_env(&mut self) {
        self.apply_device_env();
        self.apply_bus_hint_env();
    }

    /// udev rematch after bind. Must not clobber the discovered baud/id:
    /// `REALITYOS_METAL_BAUD` is a probe hint (2/3/4 Mbps); a factory
    /// XL330 is 57 600, and serve used to reopen at a 1 Mbps docs hint.
    pub fn apply_device_env(&mut self) {
        self.apply_device_path(std::env::var("REALITYOS_METAL_DEVICE").ok().as_deref());
    }

    pub fn apply_device_path(&mut self, device: Option<&str>) {
        if let Some(d) = device.map(str::trim).filter(|s| !s.is_empty()) {
            self.device = PathBuf::from(d);
        }
    }

    pub fn apply_bus_hint_env(&mut self) {
        let baud = std::env::var("REALITYOS_METAL_BAUD")
            .ok()
            .and_then(|s| s.parse().ok());
        let id = std::env::var("REALITYOS_METAL_SERVO_ID")
            .ok()
            .and_then(|s| s.parse().ok());
        self.apply_bus_hints(baud, id);
    }

    pub fn apply_bus_hints(&mut self, baud: Option<u32>, servo_id: Option<u8>) {
        if let Some(n) = baud.filter(|n| *n > 0) {
            self.baud = n;
        }
        if let Some(n) = servo_id.filter(|n| *n != 254) {
            self.servo_id = n;
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
            "max_total_excursion_ticks": self.max_total_excursion_ticks,
            "max_pwm_limit_raw": self.max_pwm_limit_raw,
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

    /// Fail closed on PWM/cage values outside XL330 legal ranges.
    pub fn validate_xl330_limits(&self) -> Result<(), String> {
        if self.max_pwm_limit_raw > crate::protocol::XL330_PWM_LIMIT_MAX {
            return Err(format!(
                "metal_pwm_limit_raw_out_of_range:{} max={}",
                self.max_pwm_limit_raw,
                crate::protocol::XL330_PWM_LIMIT_MAX
            ));
        }
        if self.max_total_excursion_ticks < 0 {
            return Err(format!(
                "metal_max_total_excursion_ticks_negative:{}",
                self.max_total_excursion_ticks
            ));
        }
        if !self.freshness_threshold_s.is_finite() || self.freshness_threshold_s <= 0.0 {
            return Err(format!(
                "metal_freshness_threshold_invalid:{}",
                self.freshness_threshold_s
            ));
        }
        Ok(())
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
/// Certified-command egress attempts (not a physical device write).
pub const WRITES_FILE: &str = "writes";
pub const EGRESS_ATTEMPTS_FILE: &str = "egress_attempts";
/// Certified command frames that passed write_all+flush. Not setup/sensor.
pub const SERIAL_TX_FILE: &str = "serial_tx";
pub const ACKS_FILE: &str = "acks";
pub const EGRESS_LOG: &str = "egress.jsonl";
pub const PWM_EVIDENCE_FILE: &str = "pwm_limit.json";
pub const CAGE_EVIDENCE_FILE: &str = "position_cage.json";
pub const LOCK_FILE: &str = "actuator.lock";
pub const PRESENT_FILE: &str = "present";
pub const GOAL_FILE: &str = "goal";
pub const VIN_FILE: &str = "vin";
/// XL330 Moving (addr 122). Campaign waits for 0 before sampling present.
pub const MOVING_FILE: &str = "moving";
pub const FRESHNESS_FILE: &str = "sensor_freshness.json";
pub const IPC_SOCKET_MODE: u32 = 0o660;

pub fn candidate_bauds(configured: u32, extra: Option<u32>) -> Vec<u32> {
    let mut out = Vec::new();
    let push = |out: &mut Vec<u32>, b: u32| {
        if b > 0 && !out.contains(&b) {
            out.push(b);
        }
    };
    // Factory/common rates first. The docs example used to export
    // REALITYOS_METAL_BAUD=1000000; that put 1 Mbps (twice, via the
    // cold-ping retry) ahead of 57 600. A mistaken 2/3/4 Mbps Wizard
    // hint did the same. Either open can wedge a CH340 so the factory
    // servo is never found. 1 Mbps stays in the automatic scan; it
    // just cannot lead. Hinted 2/3/4 Mbps still join, after 1 Mbps.
    for b in CANDIDATE_BAUDS.iter().copied().filter(|b| *b != 9_600) {
        push(&mut out, b);
    }
    let mut hints = Vec::new();
    if configured > 0 {
        hints.push(configured);
    }
    if let Some(b) = extra.filter(|b| *b > 0) {
        hints.push(b);
    }
    if hints.iter().any(|b| FAST_WIZARD_BAUDS.contains(b)) {
        for b in FAST_WIZARD_BAUDS {
            push(&mut out, *b);
        }
    }
    if hints.iter().any(|b| HIGH_WIZARD_BAUDS.contains(b)) {
        for b in HIGH_WIZARD_BAUDS {
            push(&mut out, *b);
        }
    }
    for b in hints {
        if b != 9_600
            && !CANDIDATE_BAUDS.contains(&b)
            && !FAST_WIZARD_BAUDS.contains(&b)
            && !HIGH_WIZARD_BAUDS.contains(&b)
        {
            push(&mut out, b);
        }
    }
    push(&mut out, 9_600);
    out
}

/// 1 Mbps is in the automatic scan. 2 / 3 / 4 Mbps join only when hinted.
/// A CH340/CP2102 can fail to leave any of those rates after a DTR-RESET
/// identify miss, so the factory servo is never found.
fn is_ch340_wedge_baud(b: u32) -> bool {
    b == 1_000_000 || FAST_WIZARD_BAUDS.contains(&b) || HIGH_WIZARD_BAUDS.contains(&b)
}

/// Repeat the first baud immediately. That first rate is factory 57 600
/// (`candidate_bauds` does not let a 1 Mbps docs hint or a 2/3/4 Mbps
/// Wizard hint lead). U2D2/FTDI often drop a cold first ping; the
/// after-scan retry used to run only after 2 Mbps had already opened
/// (and could wedge) a CH340.
///
/// 1 Mbps still joins every scan after 115 200. If DTR-RESET hides the
/// factory servo during the first 57 600 / 115 200 windows, the next
/// open used to be 1 Mbps and can wedge CH340 before any later factory
/// retry. A leftover 2/3/4 Mbps env still joins after that. Retry
/// factory 57 600 immediately before each of those rates (a 4 Mbps hint
/// also opens 3 Mbps first; inserting only before that 3 Mbps leaves
/// 4 Mbps immediately after a high-rate miss).
pub fn discover_baud_attempts(bauds: &[u32]) -> Vec<u32> {
    let mut out = Vec::new();
    for (i, b) in bauds.iter().copied().enumerate() {
        if is_ch340_wedge_baud(b) && out.last().copied() != Some(CANDIDATE_BAUDS[0]) {
            out.push(CANDIDATE_BAUDS[0]);
        }
        out.push(b);
        if i == 0 {
            out.push(b);
        }
    }
    out
}

pub fn candidate_servo_ids(configured: u8, extra: Option<u8>) -> Vec<u8> {
    let mut out = Vec::new();
    for id in std::iter::once(configured).chain(extra).chain([1_u8, 2]) {
        if id != 254 && !out.contains(&id) {
            out.push(id);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_governor::GovernorConfig;

    #[test]
    fn candidate_bauds_keep_factory_first_and_dedup() {
        let b = candidate_bauds(1_000_000, Some(57_600));
        assert_eq!(b[0], 57_600, "a 1 Mbps docs hint must not lead the scan");
        assert_eq!(b.iter().filter(|x| **x == 1_000_000).count(), 1);
        assert!(b.contains(&115_200));
        assert!(
            !b.contains(&2_000_000),
            "2 Mbps is a Wizard hint, not an automatic CH340 scan"
        );
        assert!(
            !b.contains(&3_000_000),
            "3 Mbps is a U2D2 hint, not an automatic CH340 scan"
        );
        assert!(!b.contains(&4_000_000));
        assert!(b.contains(&9_600));
        assert_eq!(*b.last().unwrap(), 9_600);
    }

    #[test]
    fn factory_baud_stays_first_when_hint_is_one_or_two_megabit() {
        let one = candidate_bauds(1_000_000, Some(1_000_000));
        assert_eq!(one[0], 57_600);
        let attempts = discover_baud_attempts(&one);
        assert_eq!(attempts[0], 57_600);
        assert_eq!(attempts[1], 57_600);
        let one_pos = attempts.iter().position(|&x| x == 1_000_000).unwrap();
        assert_eq!(attempts[one_pos - 1], 57_600);
        assert!(!attempts.contains(&2_000_000));

        let two = candidate_bauds(2_000_000, None);
        assert_eq!(two[0], 57_600);
        assert!(two.contains(&2_000_000));
        let one_pos = two.iter().position(|&x| x == 1_000_000).unwrap();
        let two_pos = two.iter().position(|&x| x == 2_000_000).unwrap();
        assert!(
            one_pos < two_pos,
            "must finish factory/1 Mbps before a 2 Mbps hint that can wedge CH340"
        );

        let four = candidate_bauds(4_000_000, None);
        assert_eq!(four[0], 57_600);
        assert!(four.contains(&3_000_000) && four.contains(&4_000_000));
        let one_m = four.iter().position(|&x| x == 1_000_000).unwrap();
        let four_m = four.iter().position(|&x| x == 4_000_000).unwrap();
        assert!(one_m < four_m);
    }

    #[test]
    fn high_wizard_bauds_join_scan_only_when_hinted() {
        let hi = candidate_bauds(57_600, Some(4_000_000));
        assert_eq!(hi[0], 57_600);
        assert!(hi.contains(&3_000_000));
        assert!(hi.contains(&4_000_000));
        assert!(
            !hi.contains(&2_000_000),
            "a 4 Mbps hint must not also open 2 Mbps"
        );
        let four = hi.iter().position(|&x| x == 4_000_000).unwrap();
        let slow = hi.iter().position(|&x| x == 9_600).unwrap();
        assert!(four < slow, "4 Mbps must be tried before Wizard 9600");
        assert_eq!(*hi.last().unwrap(), 9_600);
    }

    #[test]
    fn fast_wizard_baud_joins_scan_only_when_hinted() {
        let hi = candidate_bauds(57_600, Some(2_000_000));
        assert_eq!(hi[0], 57_600);
        assert!(hi.contains(&2_000_000));
        assert!(
            !hi.contains(&3_000_000),
            "a 2 Mbps hint must not also open 3/4 Mbps"
        );
        assert!(!hi.contains(&4_000_000));
        let two = hi.iter().position(|&x| x == 2_000_000).unwrap();
        let slow = hi.iter().position(|&x| x == 9_600).unwrap();
        assert!(two < slow, "2 Mbps must be tried before Wizard 9600");
    }

    #[test]
    fn discover_retries_configured_baud_before_other_rates() {
        let bauds = candidate_bauds(57_600, None);
        let attempts = discover_baud_attempts(&bauds);
        assert_eq!(attempts[0], 57_600);
        assert_eq!(attempts[1], 57_600);
        assert!(attempts[2..].contains(&115_200));
        assert!(!attempts.contains(&2_000_000));
        let one_m = attempts.iter().position(|&x| x == 1_000_000).unwrap();
        assert_eq!(
            attempts[one_m - 1],
            57_600,
            "DTR-RESET can miss the first factory/115200 opens; do not wedge CH340 at 1 Mbps next"
        );
    }

    #[test]
    fn discover_retries_factory_again_before_one_megabit_open() {
        let bauds = candidate_bauds(57_600, None);
        let attempts = discover_baud_attempts(&bauds);
        let one = attempts
            .iter()
            .position(|&x| x == 1_000_000)
            .expect("1 Mbps stays in the automatic scan");
        assert_eq!(attempts[one - 1], 57_600);
        assert!(
            !attempts.contains(&2_000_000),
            "unhinted scan must not open Wizard 2/3/4 Mbps"
        );
    }

    #[test]
    fn discover_retries_factory_again_before_high_wizard_open() {
        let bauds = candidate_bauds(57_600, Some(4_000_000));
        let attempts = discover_baud_attempts(&bauds);
        assert_eq!(attempts[0], 57_600);
        assert_eq!(attempts[1], 57_600);
        let three = attempts.iter().position(|&x| x == 3_000_000).unwrap();
        let four = attempts.iter().position(|&x| x == 4_000_000).unwrap();
        assert_eq!(
            attempts[three - 1],
            57_600,
            "DTR-RESET can miss the first factory opens; do not wedge CH340 at 3 Mbps next"
        );
        assert_eq!(
            attempts[four - 1],
            57_600,
            "a 4 Mbps hint also opens 3 Mbps; factory must precede 4 Mbps too"
        );
        let two = candidate_bauds(57_600, Some(2_000_000));
        let two_a = discover_baud_attempts(&two);
        let two_pos = two_a.iter().position(|&x| x == 2_000_000).unwrap();
        assert_eq!(two_a[two_pos - 1], 57_600);
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
    fn candidate_ids_skip_broadcast_and_keep_id_zero() {
        let ids = candidate_servo_ids(7, Some(0));
        assert_eq!(ids[0], 7);
        assert!(ids.contains(&0), "Protocol 2.0 ID 0 is a valid Wizard ID");
        assert!(!ids.contains(&254));
        assert!(ids.contains(&1));
    }

    #[test]
    fn apply_device_env_does_not_persist_bus_hints() {
        let mut cfg = MetalConfig::example("/dev/ttyUSB0");
        cfg.apply_device_path(Some("/dev/ttyUSB9"));
        assert_eq!(cfg.baud, 57_600, "init/probe must not write a baud hint");
        assert_eq!(cfg.servo_id, 1, "init/probe must not write a servo-id hint");
        assert_eq!(cfg.device, PathBuf::from("/dev/ttyUSB9"));
    }

    #[test]
    fn device_remap_does_not_clobber_discovered_baud_or_id() {
        let mut cfg = MetalConfig::example("/dev/ttyUSB0");
        cfg.baud = 115_200;
        cfg.servo_id = 2;
        cfg.apply_device_path(Some("/dev/ttyUSB1"));
        cfg.apply_bus_hints(None, None);
        assert_eq!(cfg.device, PathBuf::from("/dev/ttyUSB1"));
        assert_eq!(cfg.baud, 115_200, "serve must keep the pair probe wrote");
        assert_eq!(cfg.servo_id, 2);
    }

    #[test]
    fn bus_hints_are_probe_only_and_ignore_broadcast() {
        let mut cfg = MetalConfig::example("/dev/ttyUSB0");
        cfg.apply_bus_hints(Some(1_000_000), Some(0));
        assert_eq!(cfg.baud, 1_000_000);
        assert_eq!(cfg.servo_id, 0);
        cfg.apply_bus_hints(Some(0), Some(254));
        assert_eq!(cfg.baud, 1_000_000);
        assert_eq!(cfg.servo_id, 0);
    }

    #[test]
    fn design_hash_includes_pwm_cap_and_position_cage() {
        let mut a = MetalConfig::example("/dev/ttyUSB0");
        let h1 = a.design_content_hash();
        a.max_pwm_limit_raw = 199;
        assert_ne!(h1, a.design_content_hash());
        a.max_pwm_limit_raw = crate::protocol::CONSERVATIVE_PWM_LIMIT;
        a.max_total_excursion_ticks = 16;
        assert_ne!(h1, a.design_content_hash());
    }

    #[test]
    fn freshness_threshold_must_be_positive_and_finite() {
        let mut cfg = MetalConfig::example("/dev/ttyUSB0");
        assert!(cfg.validate_xl330_limits().is_ok());
        cfg.freshness_threshold_s = 0.0;
        assert!(cfg
            .validate_xl330_limits()
            .unwrap_err()
            .contains("metal_freshness_threshold_invalid"));
        cfg.freshness_threshold_s = f64::NAN;
        assert!(cfg
            .validate_xl330_limits()
            .unwrap_err()
            .contains("metal_freshness_threshold_invalid"));
    }

    #[test]
    fn freshness_default_matches_online_locked_sensor_stale() {
        let cfg = MetalConfig::example("/dev/ttyUSB0");
        let gov = GovernorConfig::online_locked();
        assert_eq!(
            cfg.freshness_threshold_s, gov.sensor_stale_s,
            "metal.json freshness must match the locked ONLINE window; serve cannot config_mut after start_online"
        );
    }

    #[test]
    fn pwm_cap_rejects_raw_above_xl330_range() {
        let mut cfg = MetalConfig::example("/dev/ttyUSB0");
        cfg.max_pwm_limit_raw = 886;
        let err = cfg.validate_xl330_limits().unwrap_err();
        assert!(err.contains("metal_pwm_limit_raw_out_of_range"), "{err}");
    }
}
