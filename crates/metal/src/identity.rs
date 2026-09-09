//! Hardware vs deployment identity. Do not silently treat one as the other.

use std::path::{Path, PathBuf};

use realityos_plant::HardwareIdentity;
use serde::{Deserialize, Serialize};

use crate::config::MetalConfig;
use crate::protocol::{is_xl330_model, XL330_M077_MODEL, XL330_M288_MODEL};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IdentitySource {
    MeasuredFromHardware,
    DeploymentConfiguration,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IdentityField {
    pub field: String,
    pub value: String,
    pub source: IdentitySource,
    pub note: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MeasuredIdentity {
    pub usb_serial: Option<String>,
    pub usb_fallback: Option<String>,
    pub servo_id: u8,
    pub model: u16,
    pub firmware_version: u8,
    pub connected: bool,
    pub serial: String,
    pub firmware_id: String,
    pub actuator_id: String,
    pub provenance: Vec<IdentityField>,
}

impl MeasuredIdentity {
    pub fn from_hardware(
        cfg: &MetalConfig,
        usb_serial: Option<String>,
        usb_fallback: Option<String>,
        model: u16,
        firmware_version: u8,
        connected: bool,
    ) -> Self {
        let node = device_node_identity(&cfg.device);
        let serial = match (
            usb_serial.as_deref(),
            usb_fallback.as_deref(),
            node.as_deref(),
        ) {
            (Some(s), _, _) if !s.trim().is_empty() => format!("{s}:id{}", cfg.servo_id),
            (_, Some(f), _) if !f.trim().is_empty() => format!("{f}:id{}", cfg.servo_id),
            (_, _, Some(n)) if !n.trim().is_empty() => format!("{n}:id{}", cfg.servo_id),
            _ => String::new(),
        };
        let model_name = match model {
            XL330_M288_MODEL => "xl330-m288",
            XL330_M077_MODEL => "xl330-m077",
            _ => "unknown",
        };
        let firmware_id = if model == 0 {
            String::new()
        } else {
            format!("{model_name}:{model}:{firmware_version}")
        };
        let actuator_id = format!("xl330:{}", cfg.servo_id);
        let provenance = vec![
            IdentityField {
                field: "serial".into(),
                value: serial.clone(),
                source: IdentitySource::MeasuredFromHardware,
                note: "XL330 EEPROM has no factory serial. Preference: USB adapter serial, then USB vid:pid:devpath, then measured tty name+rdev (UART/GPIO adapters). Plus the servo bus ID.".into(),
            },
            IdentityField {
                field: "firmware_id".into(),
                value: firmware_id.clone(),
                source: IdentitySource::MeasuredFromHardware,
                note: "Model number register 0 and firmware version register 6.".into(),
            },
            IdentityField {
                field: "actuator_ids".into(),
                value: actuator_id.clone(),
                source: IdentitySource::MeasuredFromHardware,
                note: "Single XL330 on the configured bus ID.".into(),
            },
            IdentityField {
                field: "calibration_id".into(),
                value: cfg.calibration_id.clone(),
                source: IdentitySource::DeploymentConfiguration,
                note: "Not an EEPROM field. Authority-owned bench calibration/limit set.".into(),
            },
            IdentityField {
                field: "design_content_hash".into(),
                value: cfg.design_content_hash(),
                source: IdentitySource::DeploymentConfiguration,
                note: "SHA-256 of realityos.metal_design/1 (limits, baud, servo id). Not device EEPROM.".into(),
            },
        ];
        Self {
            usb_serial,
            usb_fallback,
            servo_id: cfg.servo_id,
            model,
            firmware_version,
            connected: connected
                && is_xl330_model(model)
                && !serial.is_empty()
                && !firmware_id.is_empty(),
            serial,
            firmware_id,
            actuator_id,
            provenance,
        }
    }

    pub fn hardware_identity(&self, cfg: &MetalConfig) -> HardwareIdentity {
        let pty = is_pty_path(&cfg.device);
        HardwareIdentity {
            serial: self.serial.clone(),
            firmware_id: self.firmware_id.clone(),
            calibration_id: cfg.calibration_id.clone(),
            design_content_hash: cfg.design_content_hash(),
            connected: self.connected,
            metal: !pty,
            evidence_status: if pty {
                "PTY_STAND_IN_NOT_METAL".into()
            } else {
                "MEASURED_XL330_PROTOCOL2".into()
            },
            actuator_ids: vec![self.actuator_id.clone()],
        }
    }
}

/// Prefer the real tty name so `/dev/serial/by-id/...` still walks sysfs.
pub fn tty_sysfs_name(tty: &Path) -> Option<String> {
    let resolved = std::fs::canonicalize(tty).unwrap_or_else(|_| tty.to_path_buf());
    resolved.file_name()?.to_str().map(str::to_string)
}

/// Character-device identity from the OS node (major/minor). Not a factory serial.
pub fn device_node_identity(tty: &Path) -> Option<String> {
    use std::os::unix::fs::MetadataExt;
    let meta = std::fs::metadata(tty).ok()?;
    let name = tty_sysfs_name(tty)?;
    Some(format!("tty:{name}:{:x}", meta.rdev()))
}

pub fn is_pty_path(tty: &Path) -> bool {
    let canon = std::fs::canonicalize(tty).unwrap_or_else(|_| tty.to_path_buf());
    canon.starts_with("/dev/pts") || tty_sysfs_name(&canon).is_some_and(|n| n.starts_with("pts"))
}

/// Walk sysfs for a USB serial, then a vendor:product:devpath fallback.
pub fn usb_identity_for_tty(tty: &Path) -> (Option<String>, Option<String>) {
    let Some(name) = tty_sysfs_name(tty) else {
        return (None, None);
    };
    usb_identity_from_sysfs_node(&Path::new("/sys/class/tty").join(name).join("device"))
}

/// Stop at the first node with idVendor+idProduct (the USB device).
/// A CH340/CP2102 with an empty serial must not inherit a parent hub serial:
/// `ATTRS{serial}==<hub>` would udev-match every tty on that hub, and
/// `find_tty_for_expected_serial` could rebind the wrong node after rename.
pub(crate) fn usb_identity_from_sysfs_node(start: &Path) -> (Option<String>, Option<String>) {
    let mut cur = start.to_path_buf();
    for _ in 0..8 {
        let vid = std::fs::read_to_string(cur.join("idVendor"))
            .ok()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty());
        let pid = std::fs::read_to_string(cur.join("idProduct"))
            .ok()
            .map(|s| s.trim().to_string())
            .filter(|s| !s.is_empty());
        if let (Some(v), Some(p)) = (vid, pid) {
            let serial = std::fs::read_to_string(cur.join("serial"))
                .ok()
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty());
            let devpath = std::fs::read_to_string(cur.join("devpath"))
                .ok()
                .map(|s| s.trim().to_string())
                .filter(|s| !s.is_empty())
                .unwrap_or_else(|| "nodevpath".into());
            return (serial, Some(format!("usb:{v}:{p}:{devpath}")));
        }
        match std::fs::canonicalize(&cur) {
            Ok(p) => {
                if let Some(parent) = p.parent() {
                    cur = parent.to_path_buf();
                } else {
                    break;
                }
            }
            Err(_) => {
                if let Some(parent) = cur.parent() {
                    cur = parent.to_path_buf();
                } else {
                    break;
                }
            }
        }
        if cur == Path::new("/") {
            break;
        }
    }
    (None, None)
}

/// USB-UART nodes that may replace a vanished `ttyUSB*` after udev rename.
pub fn iter_usb_uart_candidates() -> Vec<PathBuf> {
    let mut out = Vec::new();
    for dir in ["/dev/serial/by-id", "/dev/serial/by-path"] {
        let Ok(rd) = std::fs::read_dir(dir) else {
            continue;
        };
        for e in rd.flatten() {
            let p = e.path();
            if p.exists() {
                out.push(p);
            }
        }
    }
    if let Ok(rd) = std::fs::read_dir("/dev") {
        for e in rd.flatten() {
            let name = e.file_name();
            let name = name.to_string_lossy();
            if name.starts_with("ttyUSB")
                || name.starts_with("ttyACM")
                || name.starts_with("ttyCH341")
            {
                let p = e.path();
                if p.exists() {
                    out.push(p);
                }
            }
        }
    }
    out
}

/// Adapter+servo-id serial string for a live tty. Model/firmware are not used.
pub fn adapter_serial_for_tty(tty: &Path, servo_id: u8) -> String {
    let (usb, fb) = usb_identity_for_tty(tty);
    let mut cfg = MetalConfig::example(tty);
    cfg.servo_id = servo_id;
    MeasuredIdentity::from_hardware(&cfg, usb, fb, 0, 0, false).serial
}

/// Find a live USB-UART whose measured adapter serial matches `probe` bind.
pub fn find_tty_for_expected_serial(expected_serial: &str, servo_id: u8) -> Option<PathBuf> {
    let want = expected_serial.trim();
    if want.is_empty() {
        return None;
    }
    iter_usb_uart_candidates()
        .into_iter()
        .find(|p| adapter_serial_for_tty(p, servo_id) == want)
}

/// Sentinel when the bound adapter is gone. `serve` must not open a living
/// recycled `ttyUSB0` — `Xl330Driver::open` applies torque before the
/// post-open identity compare.
pub const MISSING_ADAPTER_PATH: &str = "/dev/realityos-metal-missing-adapter";

/// Prefer a USB-UART whose measured adapter serial matches the bind, then an
/// existing env/config path, then `metal.json`. CH340/CP2102 often have no
/// USB serial; udev `change` can rename `ttyUSB0` → `ttyUSB1`. A living
/// preferred path is not enough: after crash close / USB re-enum the old
/// name can be a *different* adapter on the same bench. Opening that node
/// would identify-fail (or command the wrong UART if identity fell back to
/// tty name+rdev). When `expected_serial` is known, keep a path only if its
/// measured adapter serial matches. If the bound adapter is gone, return a
/// missing path instead of a living preferred whose serial drifted.
pub fn pick_live_device(
    preferred: PathBuf,
    fallback: PathBuf,
    expected_serial: &str,
    servo_id: u8,
) -> PathBuf {
    let want = expected_serial.trim();
    if !want.is_empty() {
        if preferred.exists() && adapter_serial_for_tty(&preferred, servo_id) == want {
            return preferred;
        }
        if fallback.exists() && adapter_serial_for_tty(&fallback, servo_id) == want {
            return fallback;
        }
        if let Some(found) = find_tty_for_expected_serial(want, servo_id) {
            return found;
        }
        if !preferred.exists() {
            return preferred;
        }
        if !fallback.exists() {
            return fallback;
        }
        return PathBuf::from(MISSING_ADAPTER_PATH);
    }
    if preferred.exists() {
        return preferred;
    }
    if fallback.exists() {
        return fallback;
    }
    preferred
}

/// USB-adapter serial on this node, before any servo open / torque-on.
pub fn adapter_serial_matches(device: &Path, expected_serial: &str, servo_id: u8) -> bool {
    let want = expected_serial.trim();
    !want.is_empty() && device.exists() && adapter_serial_for_tty(device, servo_id) == want
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn empty_usb_identity_is_not_invented() {
        let cfg = MetalConfig::example(PathBuf::from("/dev/missing"));
        let m = MeasuredIdentity::from_hardware(&cfg, None, None, 1190, 46, true);
        assert!(m.serial.is_empty());
        assert!(!m.connected);
        assert!(device_node_identity(Path::new("/dev/missing")).is_none());
        assert_eq!(
            m.provenance
                .iter()
                .find(|p| p.field == "calibration_id")
                .unwrap()
                .source,
            IdentitySource::DeploymentConfiguration
        );
        assert_eq!(
            m.provenance
                .iter()
                .find(|p| p.field == "firmware_id")
                .unwrap()
                .source,
            IdentitySource::MeasuredFromHardware
        );
    }

    #[test]
    fn usb_serial_plus_id_is_measured() {
        let cfg = MetalConfig::example(PathBuf::from("/dev/ttyUSB0"));
        let m = MeasuredIdentity::from_hardware(&cfg, Some("FT123".into()), None, 1190, 46, true);
        assert_eq!(m.serial, "FT123:id1");
        assert_eq!(m.firmware_id, "xl330-m288:1190:46");
        assert!(m.connected);
    }

    #[test]
    fn firmware_zero_is_not_the_measured_firmware() {
        let cfg = MetalConfig::example(PathBuf::from("/dev/ttyUSB0"));
        let measured =
            MeasuredIdentity::from_hardware(&cfg, Some("FT123".into()), None, 1190, 46, true);
        let after_sensor_clobber =
            MeasuredIdentity::from_hardware(&cfg, Some("FT123".into()), None, 1190, 0, true);
        assert_eq!(measured.firmware_id, "xl330-m288:1190:46");
        assert_ne!(measured.firmware_id, after_sensor_clobber.firmware_id);
    }

    #[test]
    fn char_device_rdev_is_measured_not_invented() {
        let cfg = MetalConfig::example(PathBuf::from("/dev/zero"));
        let node = device_node_identity(&cfg.device).expect("/dev/zero is a char device");
        assert!(node.starts_with("tty:zero:"), "{node}");
        let m = MeasuredIdentity::from_hardware(&cfg, None, None, 1190, 46, true);
        assert!(m.serial.starts_with("tty:zero:"));
        assert!(m.serial.ends_with(":id1"));
        assert!(m.connected);
        assert!(!is_pty_path(Path::new("/dev/zero")));
    }

    #[test]
    fn by_id_path_uses_basename_when_unresolved() {
        assert_eq!(
            tty_sysfs_name(Path::new("/dev/serial/by-id/usb-FTDI_FT123-if00-port0")).as_deref(),
            Some("usb-FTDI_FT123-if00-port0")
        );
        assert_eq!(
            tty_sysfs_name(Path::new("/dev/ttyUSB0")).as_deref(),
            Some("ttyUSB0")
        );
    }

    #[test]
    fn pick_live_device_keeps_existing_preferred() {
        assert_eq!(
            pick_live_device(
                PathBuf::from("/dev/null"),
                PathBuf::from("/dev/zero"),
                "",
                1
            ),
            PathBuf::from("/dev/null")
        );
    }

    #[test]
    fn pick_live_device_falls_back_when_preferred_vanished() {
        assert_eq!(
            pick_live_device(
                PathBuf::from("/dev/missing-metal-tty"),
                PathBuf::from("/dev/null"),
                "",
                1
            ),
            PathBuf::from("/dev/null")
        );
    }

    #[test]
    fn pick_live_device_skips_living_preferred_with_wrong_adapter_serial() {
        let expected = adapter_serial_for_tty(Path::new("/dev/zero"), 1);
        assert!(
            expected.starts_with("tty:zero:"),
            "char-device rdev must be measurable: {expected}"
        );
        assert_ne!(
            adapter_serial_for_tty(Path::new("/dev/null"), 1),
            expected,
            "null and zero must not share an identity"
        );
        assert_eq!(
            pick_live_device(
                PathBuf::from("/dev/null"),
                PathBuf::from("/dev/zero"),
                &expected,
                1
            ),
            PathBuf::from("/dev/zero")
        );
    }

    #[test]
    fn pick_live_device_keeps_preferred_when_adapter_serial_matches() {
        let expected = adapter_serial_for_tty(Path::new("/dev/null"), 1);
        assert!(!expected.is_empty());
        assert_eq!(
            pick_live_device(
                PathBuf::from("/dev/null"),
                PathBuf::from("/dev/zero"),
                &expected,
                1
            ),
            PathBuf::from("/dev/null")
        );
    }

    #[test]
    fn pick_live_device_refuses_living_wrong_adapter_when_bound_is_gone() {
        let expected = "usb:dead:beef:missing:id1";
        let got = pick_live_device(
            PathBuf::from("/dev/null"),
            PathBuf::from("/dev/zero"),
            expected,
            1,
        );
        assert_ne!(got, PathBuf::from("/dev/null"));
        assert_ne!(got, PathBuf::from("/dev/zero"));
        assert!(!got.exists() || adapter_serial_for_tty(&got, 1) == expected);
        assert_eq!(got, PathBuf::from(MISSING_ADAPTER_PATH));
    }

    #[test]
    fn adapter_serial_matches_requires_live_node() {
        let expected = adapter_serial_for_tty(Path::new("/dev/null"), 1);
        assert!(adapter_serial_matches(Path::new("/dev/null"), &expected, 1));
        assert!(!adapter_serial_matches(
            Path::new("/dev/zero"),
            &expected,
            1
        ));
        assert!(!adapter_serial_matches(
            Path::new("/dev/missing-metal-tty"),
            &expected,
            1
        ));
    }

    #[test]
    fn find_tty_for_expected_serial_none_on_empty_or_unknown() {
        assert!(find_tty_for_expected_serial("", 1).is_none());
        assert!(find_tty_for_expected_serial("no-such-adapter:id1", 1).is_none());
    }

    fn write_attr(dir: &Path, name: &str, value: &str) {
        std::fs::create_dir_all(dir).expect("sysfs dir");
        std::fs::write(dir.join(name), value).expect("sysfs attr");
    }

    #[test]
    fn usb_identity_stops_at_uart_device_not_parent_hub_serial() {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock")
            .as_nanos();
        let root = std::env::temp_dir().join(format!(
            "realityos-metal-usb-id-{}-{stamp}",
            std::process::id()
        ));
        let hub = root.join("hub");
        let uart = hub.join("1-1.3");
        let iface = uart.join("1-1.3:1.0");
        write_attr(&hub, "idVendor", "1d6b\n");
        write_attr(&hub, "idProduct", "0002\n");
        write_attr(&hub, "serial", "HUBSERIAL\n");
        write_attr(&uart, "idVendor", "1a86\n");
        write_attr(&uart, "idProduct", "7523\n");
        write_attr(&uart, "devpath", "1.3\n");
        write_attr(&uart, "busnum", "1\n");
        std::fs::create_dir_all(&iface).expect("iface");
        let (serial, fallback) = usb_identity_from_sysfs_node(&iface);
        let _ = std::fs::remove_dir_all(&root);
        assert_eq!(serial, None, "CH340 must not inherit hub serial");
        assert_eq!(fallback.as_deref(), Some("usb:1a86:7523:1.3"));
    }

    #[test]
    fn usb_identity_uses_serial_on_the_uart_device() {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock")
            .as_nanos();
        let root = std::env::temp_dir().join(format!(
            "realityos-metal-usb-id-ftdi-{}-{stamp}",
            std::process::id()
        ));
        let uart = root.join("1-1.2");
        let iface = uart.join("1-1.2:1.0");
        write_attr(&uart, "idVendor", "0403\n");
        write_attr(&uart, "idProduct", "6001\n");
        write_attr(&uart, "devpath", "1.2\n");
        write_attr(&uart, "serial", "FT123456\n");
        std::fs::create_dir_all(&iface).expect("iface");
        let (serial, fallback) = usb_identity_from_sysfs_node(&iface);
        let _ = std::fs::remove_dir_all(&root);
        assert_eq!(serial.as_deref(), Some("FT123456"));
        assert_eq!(fallback.as_deref(), Some("usb:0403:6001:1.2"));
    }
}
