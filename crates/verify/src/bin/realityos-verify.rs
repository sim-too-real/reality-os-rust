//! CLI for the MuJoCo verification harness. SIMULATION_ONLY. Never metal.

use realityos_verify::bundle::RobotBundle;
use realityos_verify::corpus::{milestone_robots, robot_dir};
use realityos_verify::evidence::{aggregate, render_markdown};
use realityos_verify::families::{family_spec, MILESTONE_FAMILIES};
use realityos_verify::honesty::{refuse_physical_proof_origin, SIMULATION_ONLY};
use realityos_verify::reduce::minimize;
use realityos_verify::runner::{qualify_bundle, run_episode, run_matrix, run_resolved};
use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let mut args = env::args().skip(1).collect::<Vec<_>>();
    if args.is_empty() {
        eprintln!(
            "usage: realityos-verify [inspect <id>|qualify <id>|episode <id> <family> <seed>|matrix|milestone]"
        );
        std::process::exit(2);
    }
    let cmd = args.remove(0);
    match cmd.as_str() {
        "inspect" => inspect(&args[0]),
        "qualify" => qualify(&args[0]),
        "episode" => episode(&args),
        "matrix" | "milestone" => milestone(),
        other => {
            eprintln!("unknown {other}");
            std::process::exit(2);
        }
    }
}

fn inspect(id: &str) {
    let b = RobotBundle::load(robot_dir(id)).expect("bundle");
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "robot_id": b.manifest.robot_id,
            "format": b.format,
            "source_hash": b.source_hash,
            "metal": false,
            "evidence_status": SIMULATION_ONLY,
        }))
        .unwrap()
    );
}

fn qualify(id: &str) {
    let b = RobotBundle::load(robot_dir(id)).expect("bundle");
    let q = qualify_bundle(&b).expect("qualify");
    println!("{}", serde_json::to_string_pretty(&q).unwrap());
}

fn episode(args: &[String]) {
    let id = &args[0];
    let family = args.get(1).map(String::as_str).unwrap_or("JOINT_TRACKING");
    let seed = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(1);
    let b = RobotBundle::load(robot_dir(id)).expect("bundle");
    let ep = run_episode(&b, family, seed, "pd").expect("episode");
    refuse_physical_proof_origin(&ep.to_json_value()).expect("sim evidence");
    println!("{}", serde_json::to_string_pretty(&ep).unwrap());
}

fn milestone() {
    let out = PathBuf::from("verify-out");
    fs::create_dir_all(&out).ok();
    let robots = milestone_robots();
    for path in &robots {
        if let Ok(b) = RobotBundle::load(path) {
            if let Ok(q) = qualify_bundle(&b) {
                let _ = fs::write(
                    out.join(format!(
                        "control_qualification_{}.json",
                        b.manifest.robot_id
                    )),
                    serde_json::to_string_pretty(&q).unwrap(),
                );
            }
        }
    }
    let n_seeds: u64 = env::var("REALITYOS_VERIFY_SEEDS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(100);
    let seeds: Vec<u64> = (0..n_seeds).collect();
    let episodes = run_matrix(&robots, MILESTONE_FAMILIES, seeds, &["pd"], true);
    let report = aggregate(&episodes);
    refuse_physical_proof_origin(&serde_json::to_value(&report).unwrap()).expect("report");
    fs::write(
        out.join("verification_report.json"),
        serde_json::to_string_pretty(&report).unwrap(),
    )
    .unwrap();
    fs::write(out.join("VERIFICATION_REPORT.md"), render_markdown(&report)).unwrap();
    for (i, ep) in episodes.iter().enumerate() {
        if i < 8 || !ep.task_success {
            let _ = ep.write(out.join(format!(
                "episode_{}_{}_{}.json",
                ep.robot_id, ep.family, ep.seed
            )));
        }
    }
    if let Some(fail) = episodes.iter().find(|e| {
        !e.task_success
            && !e.not_applicable
            && !matches!(
                e.family.as_str(),
                "COMMAND_REPLAY"
                    | "DUPLICATE_COMMAND"
                    | "WRONG_ROBOT_IDENTITY"
                    | "WRONG_TASK_AUTHORITY"
                    | "POLICY_CRASH"
                    | "AUTHORITY_RESTART"
                    | "STALE_OBSERVATION"
            )
    }) {
        if let Ok(bundle) = RobotBundle::load(robot_dir(&fail.robot_id)) {
            let ee = bundle
                .manifest
                .end_effectors
                .first()
                .map(|e| e.name.as_str())
                .unwrap_or("ee");
            let spec = family_spec(&fail.family, 3, ee);
            let reduced = minimize(&spec, fail, |resolved| {
                run_resolved(&bundle, resolved.clone(), "pd").ok()
            });
            let _ = fs::write(
                out.join("counterexample.json"),
                serde_json::to_string_pretty(&reduced).unwrap(),
            );
        }
    }
    println!(
        "SIMULATION VERIFIED episodes={} metal=false evidence_status={}",
        episodes.len(),
        SIMULATION_ONLY
    );
    println!("wrote verify-out/verification_report.json");
}
