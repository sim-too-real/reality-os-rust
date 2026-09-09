use std::env;
use std::path::PathBuf;

use realityos_metal::config::{MetalConfig, CONFIG_FILE, MEASURED_FILE};
use realityos_metal::identity::usb_identity_for_tty;
use realityos_metal::proof::{default_unresolved, CaseRecord, MetalProof, ProofMeta, PROOF_SCHEMA};
use realityos_metal::{serve_forever, Xl330Driver};

fn main() -> anyhow::Result<()> {
    let mut root = PathBuf::from("/tmp/realityos-metal");
    let mut device: Option<PathBuf> = None;
    let mut first = true;
    let mut cmd = String::new();
    let mut cases = PathBuf::from("docs/metal_cases.json");
    let mut out = PathBuf::from("docs/metal_proof.json");
    let mut args = env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--root" => root = PathBuf::from(args.next().unwrap_or_default()),
            "--device" => device = Some(PathBuf::from(args.next().unwrap_or_default())),
            "--first-online" => first = true,
            "--restart" => first = false,
            "--cases" => cases = PathBuf::from(args.next().unwrap_or_default()),
            "--out" => out = PathBuf::from(args.next().unwrap_or_default()),
            "init" | "probe" | "bind-measured" | "serve" | "report" => cmd = a,
            other => anyhow::bail!("unknown_arg:{other}"),
        }
    }
    match cmd.as_str() {
        "init" => {
            std::fs::create_dir_all(&root)?;
            let mut cfg = MetalConfig::example(device.unwrap_or_else(|| "/dev/ttyUSB0".into()));
            if let Ok(env_dev) = env::var("REALITYOS_METAL_DEVICE") {
                cfg.device = PathBuf::from(env_dev);
            }
            cfg.save(root.join(CONFIG_FILE))?;
            println!("{}", serde_json::to_string_pretty(&cfg)?);
        }
        "probe" => {
            std::fs::create_dir_all(&root)?;
            let cfg_path = root.join(CONFIG_FILE);
            let mut cfg = if cfg_path.exists() {
                MetalConfig::load(&cfg_path)?
            } else {
                MetalConfig::example(device.clone().unwrap_or_else(|| "/dev/ttyUSB0".into()))
            };
            if let Some(d) = device {
                cfg.device = d;
            }
            if let Ok(env_dev) = env::var("REALITYOS_METAL_DEVICE") {
                cfg.device = PathBuf::from(env_dev);
            }
            cfg.save(&cfg_path)?;
            if !cfg.device.exists() {
                let (usb, fb) = usb_identity_for_tty(&cfg.device);
                println!(
                    "{}",
                    serde_json::json!({
                        "ok": false,
                        "error": "metal_device_missing",
                        "device": cfg.device,
                        "usb_serial": usb,
                        "usb_fallback": fb,
                    })
                );
                std::process::exit(2);
            }
            let driver = Xl330Driver::open(cfg.clone(), &root)?;
            let measured = driver.measured();
            let id = measured.hardware_identity(&cfg);
            let report = serde_json::json!({
                "ok": measured.connected,
                "measured": measured,
                "hardware_identity": {
                    "serial": id.serial,
                    "firmware_id": id.firmware_id,
                    "calibration_id": id.calibration_id,
                    "design_content_hash": id.design_content_hash,
                    "connected": id.connected,
                    "metal": id.metal,
                    "evidence_status": id.evidence_status,
                    "actuator_ids": id.actuator_ids,
                },
                "design_content_hash_source": "deployment_configuration",
                "calibration_id_source": "deployment_configuration",
                "clock": "OsMonotonicClock_on_start_online",
            });
            std::fs::write(
                root.join(MEASURED_FILE),
                serde_json::to_string_pretty(&report)?,
            )?;
            println!("{}", serde_json::to_string_pretty(&report)?);
        }
        "bind-measured" => {
            let cfg_path = root.join(CONFIG_FILE);
            let mut cfg = MetalConfig::load(&cfg_path)?;
            let measured_raw = std::fs::read_to_string(root.join(MEASURED_FILE))?;
            let v: serde_json::Value = serde_json::from_str(&measured_raw)?;
            let serial = v
                .pointer("/measured/serial")
                .and_then(|x| x.as_str())
                .unwrap_or("")
                .to_string();
            let fw = v
                .pointer("/measured/firmware_id")
                .and_then(|x| x.as_str())
                .unwrap_or("")
                .to_string();
            if serial.is_empty() || fw.is_empty() {
                anyhow::bail!("bind_measured_requires_probe");
            }
            cfg.expected_serial = serial;
            cfg.expected_firmware = fw;
            cfg.save(&cfg_path)?;
            println!("{}", serde_json::to_string_pretty(&cfg)?);
        }
        "serve" => {
            if let Ok(v) = env::var("REALITYOS_METAL_CAMPAIGN") {
                if v == "1" {
                    let cfg_path = root.join(CONFIG_FILE);
                    if cfg_path.exists() {
                        let mut cfg = MetalConfig::load(&cfg_path)?;
                        cfg.campaign_hooks = true;
                        cfg.save(&cfg_path)?;
                    }
                }
            }
            serve_forever(&root, first)?;
        }
        "report" => {
            let raw = std::fs::read_to_string(&cases)?;
            let recs: Vec<CaseRecord> = serde_json::from_str(&raw)?;
            let meta_path = root.join("proof_meta.json");
            let meta: ProofMeta = if meta_path.exists() {
                serde_json::from_str(&std::fs::read_to_string(meta_path)?)?
            } else {
                anyhow::bail!("proof_meta.json missing; campaign must write measured metadata");
            };
            match MetalProof::from_measured(meta, recs, default_unresolved()) {
                Ok(p) => {
                    assert_eq!(p.schema, PROOF_SCHEMA);
                    if let Some(dir) = out.parent() {
                        std::fs::create_dir_all(dir)?;
                    }
                    std::fs::write(&out, serde_json::to_string_pretty(&p)?)?;
                    println!("{}", serde_json::to_string_pretty(&p)?);
                }
                Err(e) => {
                    eprintln!("error:{e}");
                    std::process::exit(2);
                }
            }
        }
        _ => {
            eprintln!(
                "usage: realityos-metal-smoke init|probe|bind-measured|serve|report --root DIR"
            );
            std::process::exit(2);
        }
    }
    Ok(())
}
