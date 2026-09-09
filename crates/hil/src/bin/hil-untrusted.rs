use std::env;
use std::path::PathBuf;

use realityos_hil::{call, HilRequest};
use realityos_vport::try_hostile_open;

/// Untrusted autonomy process. Never receives keys, OnlineWrite, or Plant.
fn main() -> anyhow::Result<()> {
    let mut root = PathBuf::from("/tmp/realityos-hil");
    let mut op = "propose".to_string();
    let mut verb = "hold".to_string();
    let mut id = "u1".to_string();
    let mut action: Option<Vec<f64>> = None;
    let mut skip_sensor = false;
    let mut args = env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--root" => root = PathBuf::from(args.next().unwrap_or_default()),
            "--op" => op = args.next().unwrap_or_default(),
            "--verb" => verb = args.next().unwrap_or_default(),
            "--now" | "--seq" => {
                // Production proposal cannot set authority time or sequence.
                let _ = args.next();
            }
            "--id" => id = args.next().unwrap_or_default(),
            "--action" => {
                let raw = args.next().unwrap_or_default();
                action = Some(raw.split(',').filter_map(|s| s.parse().ok()).collect());
            }
            "--nan" => action = Some(vec![f64::NAN]),
            "--inf" => action = Some(vec![f64::INFINITY]),
            "--skip-sensor" => skip_sensor = true,
            "attack-open" => {
                let r = try_hostile_open(root.join("bus"));
                println!("{}", serde_json::to_string(&r)?);
                return Ok(());
            }
            _ => {}
        }
    }
    let req = if skip_sensor || op == "hil_fault" {
        let mut r = HilRequest::hil_fault(
            &id,
            &verb,
            realityos_hil::HilFaultInjectionRequest {
                skip_sensor,
                ..Default::default()
            },
        );
        r.op = op;
        r.action = action;
        r
    } else {
        let mut r = HilRequest::propose(&id, &verb);
        r.op = op;
        r.action = action;
        r
    };
    let resp = call(&root, &req)?;
    println!("{}", serde_json::to_string(&resp)?);
    Ok(())
}
