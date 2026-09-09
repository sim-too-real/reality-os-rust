use std::env;
use std::path::PathBuf;

use realityos_hil::{serve_with_opts, Authority, HilRequest, ServeOpts};

fn main() -> anyhow::Result<()> {
    let mut root = PathBuf::from("/tmp/realityos-hil");
    let mut first = true;
    let mut serve = false;
    let mut production = false;
    let mut once: Option<HilRequest> = None;
    let mut args = env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--root" => root = PathBuf::from(args.next().unwrap_or_default()),
            "--first-online" => first = true,
            "--restart" => first = false,
            "--production" => production = true,
            "serve" => serve = true,
            "once" => {
                let mut req = HilRequest::propose("once", "hold");
                while let Some(b) = args.next() {
                    match b.as_str() {
                        "--verb" => req.verb = args.next().unwrap_or_default(),
                        "--now" => {
                            req.now_s = args.next().unwrap_or_default().parse().unwrap_or(20.0)
                        }
                        "--seq" => {
                            req.sequence = args.next().unwrap_or_default().parse().unwrap_or(1)
                        }
                        "--id" => req.command_id = args.next().unwrap_or_default(),
                        "--skip-sensor" => req.skip_sensor = true,
                        _ => {}
                    }
                }
                req.op = "propose".into();
                once = Some(req);
            }
            _ => {}
        }
    }
    if serve {
        return serve_with_opts(
            &root,
            ServeOpts {
                first_online: first,
                now_s: 10.0,
                production_ipc: production,
            },
        );
    }
    if let Some(req) = once {
        let mut auth = if production {
            Authority::start_deploy(&root, first, req.now_s.max(1.0))?
        } else {
            Authority::start(&root, first, req.now_s.max(1.0))?
        };
        let resp = if production {
            auth.handle_production(req)
        } else {
            auth.handle(req)
        };
        println!("{}", serde_json::to_string(&resp)?);
    }
    Ok(())
}
