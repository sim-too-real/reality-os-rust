use std::env;
use std::path::PathBuf;

use realityos_hil::{call, HilRequest};
use realityos_vport::try_hostile_open;

/// Untrusted autonomy process. Never receives keys, OnlineWrite, or Plant.
fn main() -> anyhow::Result<()> {
    let mut root = PathBuf::from("/tmp/realityos-hil");
    let mut op = "propose".to_string();
    let mut verb = "hold".to_string();
    let mut now_s = 10.0;
    let mut seq = 1i64;
    let mut id = "u1".to_string();
    let mut action: Option<Vec<f64>> = None;
    let mut skip_sensor = false;
    let mut args = env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--root" => root = PathBuf::from(args.next().unwrap_or_default()),
            "--op" => op = args.next().unwrap_or_default(),
            "--verb" => verb = args.next().unwrap_or_default(),
            "--now" => now_s = args.next().unwrap_or_default().parse().unwrap_or(10.0),
            "--seq" => seq = args.next().unwrap_or_default().parse().unwrap_or(1),
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
    let req = HilRequest {
        op,
        verb,
        now_s,
        sequence: seq,
        command_id: id,
        action,
        ttl_s: 30.0,
        skip_sensor,
        write_now_s: None,
        ttl_override: None,
        skip_heartbeat: false,
    };
    let resp = call(&root, &req)?;
    println!("{}", serde_json::to_string(&resp)?);
    Ok(())
}
