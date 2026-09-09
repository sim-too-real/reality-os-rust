use std::env;
use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
use std::path::PathBuf;

use realityos_metal::config::{JOURNAL, SIGNING_KEY_FILE};
use realityos_metal::ipc::{call, call_raw, MetalRequest};

fn main() -> anyhow::Result<()> {
    let mut root = PathBuf::from("/tmp/realityos-metal");
    let mut cmd = String::new();
    let mut verb = "hold".to_string();
    let mut id = env::var("METAL_CMD_ID").unwrap_or_else(|_| "metal-1".into());
    let mut action: Option<Vec<f64>> = None;
    let mut raw: Option<String> = None;
    let mut args = env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--root" => root = PathBuf::from(args.next().unwrap_or_default()),
            "--verb" => verb = args.next().unwrap_or_default(),
            "--id" => id = args.next().unwrap_or_default(),
            "--action" => {
                let s = args.next().unwrap_or_default();
                action = Some(
                    s.split(',')
                        .filter(|x| !x.is_empty())
                        .map(|x| x.parse().unwrap_or(f64::NAN))
                        .collect(),
                );
            }
            "--authority-pid" => {
                let _ = args.next();
            }
            "propose" | "propose-id" | "unsupported" | "oversized" | "replay" | "raw"
            | "hil_fault" | "caller_time" | "os-probe" | "status" | "recover" | "sensor"
            | "forged-sensor" => cmd = a,
            other if other.starts_with('{') => raw = Some(other.to_string()),
            other => anyhow::bail!("unknown_arg:{other}"),
        }
    }
    if env::var("METAL_CMD_ID").is_ok() {
        id = env::var("METAL_CMD_ID")?;
    }
    match cmd.as_str() {
        "status" => {
            let mut r = MetalRequest::propose(&id, "hold");
            r.op = "status".into();
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "sensor" => {
            let mut r = MetalRequest::propose(&id, "hold");
            r.op = "sensor".into();
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "propose" | "propose-id" => {
            let mut r = MetalRequest::propose(&id, &verb);
            r.action = action;
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "unsupported" => {
            let r = MetalRequest::propose("metal-dance", "dance");
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "oversized" => {
            let mut r = MetalRequest::propose("metal-big", "drive");
            r.action = Some(vec![1_000_000.0]);
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "replay" => {
            let r = MetalRequest::propose(&id, "hold");
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "raw" => {
            let line = raw.unwrap_or_else(|| "{not-json".into());
            match call_raw(&root, &line) {
                Ok(resp) => println!("{}", serde_json::to_string(&resp)?),
                Err(_) => println!(
                    "{}",
                    serde_json::json!({
                        "ok": false,
                        "executed": false,
                        "stage": "protocol",
                        "status": "refuse",
                        "violations": ["raw_ipc_error"],
                        "physical_writes": 0,
                        "metal": true,
                    })
                ),
            }
        }
        "hil_fault" => {
            let mut r = MetalRequest::propose("metal-fault", "hold");
            r.op = "hil_fault".into();
            r.fault = Some(serde_json::json!({"now_s": 1.0}));
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "caller_time" => {
            let mut r = MetalRequest::propose("metal-time", "hold");
            r.now_s = Some(99.0);
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "recover" => {
            let mut r = MetalRequest::propose(&id, "hold");
            r.op = "recover".into();
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "forged-sensor" => {
            let mut r = MetalRequest::propose("metal-forge-sensor", "hold");
            r.sensor_samples = Some(serde_json::json!([{"q0": 0.0, "ts": 1e9}]));
            println!("{}", serde_json::to_string(&call(&root, &r)?)?);
        }
        "os-probe" => {
            println!("{}", serde_json::to_string_pretty(&os_probe(&root)?)?);
        }
        _ => {
            eprintln!("usage: realityos-metal-propose --root DIR propose|os-probe|...");
            std::process::exit(2);
        }
    }
    Ok(())
}

fn os_probe(root: &std::path::Path) -> anyhow::Result<serde_json::Value> {
    let uid = rust_uid();
    let euid = rust_euid();
    let ran_as_root = euid == 0;
    let device = realityos_metal::config::MetalConfig::load(root.join("metal.json"))
        .ok()
        .map(|c| c.device)
        .unwrap_or_default();

    let mut device_open_attempts: u64 = 0;
    let mut device_open_successes: u64 = 0;
    let mut device_write_successes: u64 = 0;

    if !device.as_os_str().is_empty() {
        device_open_attempts += 1;
        if let Ok(mut f) = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(&device)
        {
            device_open_successes += 1;
            use std::io::Write;
            if writeln!(f, "HOSTILE").is_ok() {
                device_write_successes += 1;
            }
        }
    }

    let lock = root.join("bus/actuator.lock");
    let lock_open = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(&lock);
    let (open_actuator_lock, take_actuator_lock) = match lock_open {
        Ok(f) => (true, fs2::FileExt::try_lock_exclusive(&f).is_ok()),
        Err(_) => (false, false),
    };

    let key = root.join(SIGNING_KEY_FILE);
    let read_signing_key = std::fs::read(&key).is_ok();
    let write_signing_key = std::fs::OpenOptions::new()
        .write(true)
        .custom_flags(0)
        .open(&key)
        .map(|mut f| {
            use std::io::Write;
            writeln!(f, "x").is_ok()
        })
        .unwrap_or(false);
    let chmod_signing_key =
        std::fs::set_permissions(&key, std::fs::Permissions::from_mode(0o666)).is_ok();

    let journal = root.join(JOURNAL);
    let modify_journal = std::fs::OpenOptions::new()
        .append(true)
        .open(&journal)
        .map(|mut f| {
            use std::io::Write;
            writeln!(f, "hostile").is_ok()
        })
        .unwrap_or(false);

    let mut ipc_connect = false;
    let mut status_reached_authority = false;
    if let Ok(resp) = {
        let mut r = MetalRequest::propose("probe-status", "hold");
        r.op = "status".into();
        call(root, &r)
    } {
        ipc_connect = true;
        status_reached_authority = resp.ok;
    }

    let auth_pid = env::var("METAL_AUTHORITY_PID")
        .ok()
        .and_then(|s| s.parse::<i32>().ok());
    let mut proc_fd_device = false;
    if let Some(pid) = auth_pid {
        if let Ok(rd) = std::fs::read_dir(format!("/proc/{pid}/fd")) {
            for e in rd.flatten() {
                if let Ok(link) = std::fs::read_link(e.path()) {
                    if link == device {
                        proc_fd_device = true;
                    }
                }
            }
        }
    }

    Ok(serde_json::json!({
        "ran_as_root": ran_as_root,
        "uid": uid,
        "euid": euid,
        "ipc_connect": ipc_connect,
        "status_reached_authority": status_reached_authority,
        "read_signing_key": read_signing_key,
        "write_signing_key": write_signing_key,
        "chmod_signing_key": chmod_signing_key,
        "modify_journal": modify_journal,
        "open_actuator_lock": open_actuator_lock,
        "take_actuator_lock": take_actuator_lock,
        "direct_device_open_attempts": device_open_attempts,
        "direct_device_open_successes": device_open_successes,
        "direct_device_write_successes": device_write_successes,
        "proc_fd_device": proc_fd_device,
    }))
}

fn rust_uid() -> u32 {
    std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("Uid:"))
                .and_then(|l| l.split_whitespace().nth(1))
                .and_then(|x| x.parse().ok())
        })
        .unwrap_or(0)
}

fn rust_euid() -> u32 {
    std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("Uid:"))
                .and_then(|l| l.split_whitespace().nth(2))
                .and_then(|x| x.parse().ok())
        })
        .unwrap_or(0)
}
