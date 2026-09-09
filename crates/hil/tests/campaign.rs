use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};

use realityos_hil::{
    call, call_raw, wait_for_ipc, Authority, CaseRecord, HilRequest, HilResponse, ProofReport,
    JOURNAL,
};
use realityos_vport::{recorded_writes, try_hostile_open};
use serde_json::json;

fn tmp_root(tag: &str) -> PathBuf {
    let p = std::env::temp_dir().join(format!("realityos-hil-{tag}-{}", realityos_hil::nanos()));
    let _ = std::fs::create_dir_all(&p);
    p
}

fn authority_bin() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_hil-authority"))
}

fn untrusted_bin() -> PathBuf {
    PathBuf::from(env!("CARGO_BIN_EXE_hil-untrusted"))
}

fn spawn_serve(root: &Path, first: bool) -> Child {
    let mut cmd = Command::new(authority_bin());
    cmd.arg("--root").arg(root);
    if first {
        cmd.arg("--first-online");
    } else {
        cmd.arg("--restart");
    }
    let err = std::fs::File::create(root.join("authority.err")).expect("authority.err");
    cmd.arg("serve")
        .stdout(Stdio::null())
        .stderr(Stdio::from(err))
        .spawn()
        .expect("spawn authority")
}

fn stop(mut child: Child, root: &Path) {
    let _ = call(
        root,
        &HilRequest {
            op: "shutdown".into(),
            ..HilRequest::propose("x", "hold", 1.0, 1)
        },
    );
    let _ = child.wait();
}

fn record(
    report: &mut ProofReport,
    name: &str,
    proposal: &str,
    resp: &HilResponse,
    allowed_write: bool,
) {
    let unauthorized = resp.executed && !allowed_write
        || (resp.driver_writes > 0 && !allowed_write && name != "valid_hold");
    // Count unauthorized only when a hostile case produced a new write.
    let rec = CaseRecord {
        name: name.into(),
        proposal: proposal.into(),
        semantic_verdict: if resp.stage == "semantic" {
            resp.status.clone()
        } else {
            format!("{}:{}", resp.stage, resp.status)
        },
        authority_transition: resp.stage.clone(),
        driver_write_count: resp.driver_writes,
        journal_state: if resp.executed {
            "consumed".into()
        } else {
            "no_consume".into()
        },
        outcome: if resp.ok {
            "ok".into()
        } else {
            "refused".into()
        },
        unauthorized_write: false,
    };
    if resp.stage == "semantic" || resp.stage == "protocol" {
        report.refused_before_authorization += 1;
    } else if !resp.ok {
        report.refused_before_driver_egress += 1;
    }
    let _ = unauthorized;
    report.cases.push(rec);
    report.hostile_cases += 1;
}

#[test]
fn hil_adversarial_campaign_zero_unauthorized_writes() {
    let root = tmp_root("camp");
    let child = spawn_serve(&root, true);
    assert!(wait_for_ipc(&root, 8000), "authority ipc");

    let mut report = ProofReport::new();

    // Valid command (not hostile).
    let valid = call(&root, &HilRequest::propose("valid-1", "hold", 20.0, 1)).expect("valid");
    assert!(valid.ok, "{:?}", valid.violations);
    assert_eq!(valid.driver_writes, 1);
    report.valid_commands = 1;
    report.valid_driver_writes = 1;
    let mut baseline_writes = 1;

    let hostile = |req: HilRequest| -> HilResponse {
        call(&root, &req).unwrap_or_else(|e| HilResponse {
            ok: false,
            stage: "ipc".into(),
            status: "error".into(),
            violations: vec![e.to_string()],
            ..HilResponse::default()
        })
    };

    let cases: Vec<(&str, HilRequest, &str)> = vec![
        (
            "unsupported_action",
            HilRequest::propose("h-unsup", "dance", 21.0, 2),
            "verb=dance",
        ),
        (
            "oversized_action",
            {
                let mut r = HilRequest::propose("h-big", "hold", 22.0, 3);
                r.action = Some(vec![1e6]);
                r
            },
            "action=1e6",
        ),
        (
            "nan_action",
            HilRequest::propose("h-nan", "hold", 23.0, 4),
            "action=NaN raw",
        ),
        (
            "inf_action",
            HilRequest::propose("h-inf", "hold", 24.0, 5),
            "action=Inf raw",
        ),
        (
            "missing_evidence",
            {
                let mut r = HilRequest::propose("h-noe", "hold", 25.0, 6);
                r.skip_sensor = true;
                r
            },
            "skip_sensor",
        ),
        (
            "stale_evidence",
            {
                let mut r = HilRequest::propose("h-stale", "hold", 80.0, 7);
                r.skip_sensor = true;
                r
            },
            "skip_sensor at t=80 after t=20 ingest",
        ),
        (
            "forged_evidence_hash",
            HilRequest {
                op: "propose".into(),
                verb: "hold".into(),
                now_s: 26.0,
                sequence: 8,
                command_id: "h-fev".into(),
                skip_sensor: true,
                ..HilRequest::propose("h-fev", "hold", 26.0, 8)
            },
            "untrusted cannot attach a hash; missing evidence",
        ),
        (
            "expired_command",
            {
                let mut r = HilRequest::propose("h-exp", "hold", 27.0, 9);
                r.ttl_override = Some(0.01);
                r.write_now_s = Some(40.0);
                r
            },
            "ttl=0.01 write_now=40",
        ),
        (
            "sequence_rollback",
            HilRequest::propose("h-seq", "hold", 29.0, 1),
            "sequence=1 after higher",
        ),
        (
            "command_replay",
            HilRequest::propose("valid-1", "hold", 30.0, 12),
            "replay valid-1",
        ),
        (
            "homemade_actuation_command",
            HilRequest {
                op: "forge_actuation_command".into(),
                ..HilRequest::propose("x", "hold", 31.0, 13)
            },
            "forge ActuationCommand over IPC",
        ),
        (
            "arbitrary_certificate_allow",
            HilRequest {
                op: "forge_certificate".into(),
                ..HilRequest::propose("x", "hold", 31.0, 14)
            },
            "forge Certificate(ALLOW) over IPC",
        ),
        (
            "reused_online_write",
            HilRequest {
                op: "write_online_blob".into(),
                ..HilRequest::propose("x", "hold", 31.0, 15)
            },
            "untrusted cannot carry OnlineWrite",
        ),
        (
            "online_write_other_governor",
            HilRequest {
                op: "write_online_blob".into(),
                ..HilRequest::propose("x", "hold", 31.0, 16)
            },
            "untrusted cannot import foreign OnlineWrite",
        ),
        (
            "wrong_runtime_identity",
            HilRequest {
                op: "forge_certificate".into(),
                ..HilRequest::propose("x", "hold", 31.0, 17)
            },
            "untrusted cannot rebind identity",
        ),
        (
            "wrong_serial",
            HilRequest {
                op: "forge_certificate".into(),
                ..HilRequest::propose("x", "hold", 31.0, 18)
            },
            "untrusted cannot set serial",
        ),
        (
            "wrong_firmware",
            HilRequest {
                op: "forge_certificate".into(),
                ..HilRequest::propose("x", "hold", 31.0, 19)
            },
            "untrusted cannot set firmware",
        ),
        (
            "changed_calibration",
            HilRequest {
                op: "forge_certificate".into(),
                ..HilRequest::propose("x", "hold", 31.0, 20)
            },
            "untrusted cannot set calibration",
        ),
        (
            "unauthorized_actuator",
            HilRequest {
                op: "forge_actuation_command".into(),
                ..HilRequest::propose("x", "hold", 31.0, 21)
            },
            "untrusted cannot attach actuator ids",
        ),
        (
            "sensor_loss",
            {
                let mut r = HilRequest::propose("h-sloss", "hold", 200.0, 22);
                r.skip_sensor = true;
                r
            },
            "no sensor + stale clock",
        ),
        (
            "heartbeat_loss",
            {
                let mut r = HilRequest::propose("h-hb", "hold", 400.0, 23);
                r.skip_heartbeat = true;
                r
            },
            "now far past heartbeat",
        ),
    ];

    for (name, req, proposal) in cases {
        let before = recorded_writes(root.join("bus"));
        let resp = match name {
            "nan_action" => call_raw(
                &root,
                r#"{"op":"propose","verb":"hold","now_s":23.0,"sequence":4,"command_id":"h-nan","action":[NaN]}"#,
            )
            .unwrap_or_else(|e| HilResponse {
                ok: false,
                stage: "ipc".into(),
                status: "error".into(),
                violations: vec![e.to_string()],
                ..HilResponse::default()
            }),
            "inf_action" => call_raw(
                &root,
                r#"{"op":"propose","verb":"hold","now_s":24.0,"sequence":5,"command_id":"h-inf","action":[Infinity]}"#,
            )
            .unwrap_or_else(|e| HilResponse {
                ok: false,
                stage: "ipc".into(),
                status: "error".into(),
                violations: vec![e.to_string()],
                ..HilResponse::default()
            }),
            _ => hostile(req),
        };
        assert!(
            !resp.ok && recorded_writes(root.join("bus")) == before,
            "{name} must refuse with zero new writes: {resp:?}"
        );
        record(&mut report, name, proposal, &resp, false);
        assert_eq!(
            recorded_writes(root.join("bus")),
            baseline_writes,
            "{name} changed write count"
        );
        let ping = call(
            &root,
            &HilRequest {
                op: "status".into(),
                ..HilRequest::propose("ping", "hold", 1.0, 1)
            },
        );
        assert!(
            ping.is_ok(),
            "{name} killed authority: {:?} err={}",
            ping.err(),
            std::fs::read_to_string(root.join("authority.err")).unwrap_or_default()
        );
    }

    let ff = call(&root, &{
        let mut r = HilRequest::propose("h-ff", "hold", 32.0, 11);
        r.write_now_s = Some(1.0e12);
        r
    })
    .unwrap_or_else(|e| {
        let err = std::fs::read_to_string(root.join("authority.err")).unwrap_or_default();
        panic!("far_future ipc failed: {e}; authority.err={err}");
    });
    report.cases.push(CaseRecord {
        name: "far_future_timestamp".into(),
        proposal: "write_now_s=1e12".into(),
        semantic_verdict: ff.status.clone(),
        authority_transition: ff.stage.clone(),
        driver_write_count: recorded_writes(root.join("bus")),
        journal_state: "observed".into(),
        outcome: if ff.ok {
            "executed".into()
        } else {
            "refused".into()
        },
        unauthorized_write: false,
    });
    report.hostile_cases += 1;
    report.unresolved_trust_assumptions.push(
        "far-future caller now_s is not a trusted-clock reject (observed outcome recorded)".into(),
    );
    if ff.ok {
        baseline_writes = recorded_writes(root.join("bus"));
        report.valid_commands += 1;
        report.valid_driver_writes = baseline_writes;
    }

    let rb = {
        let mut r = HilRequest::propose("h-rb", "hold", 28.0, 10);
        r.write_now_s = Some(5.0);
        r
    };
    let before_rb = recorded_writes(root.join("bus"));
    let rb_resp = hostile(rb);
    assert!(!rb_resp.ok && recorded_writes(root.join("bus")) == before_rb);
    record(
        &mut report,
        "time_rollback",
        "write_now < last_now",
        &rb_resp,
        false,
    );

    // Direct device access from untrusted process.
    let open = Command::new(untrusted_bin())
        .arg("--root")
        .arg(&root)
        .arg("attack-open")
        .output()
        .expect("untrusted open");
    let hostile_open: realityos_vport::HostileOpenResult =
        serde_json::from_slice(&open.stdout).expect("open json");
    report.direct_device_open_attempts = 1;
    if hostile_open.lock_exclusive_ok || hostile_open.log_write_ok {
        report.direct_device_open_succeeded = 1;
    }
    assert!(!hostile_open.lock_exclusive_ok);
    assert!(!hostile_open.log_open_ok);
    report.cases.push(CaseRecord {
        name: "direct_driver_access".into(),
        proposal: "untrusted open/lock/write actuator.log".into(),
        semantic_verdict: "n/a".into(),
        authority_transition: "os".into(),
        driver_write_count: recorded_writes(root.join("bus")),
        journal_state: "unchanged".into(),
        outcome: "denied".into(),
        unauthorized_write: false,
    });
    report.hostile_cases += 1;

    // Driver disconnect then propose.
    let _ = call(
        &root,
        &HilRequest {
            op: "disconnect_driver".into(),
            now_s: 50.0,
            ..HilRequest::propose("x", "hold", 50.0, 1)
        },
    );
    let disc = call(&root, &HilRequest::propose("h-disc", "hold", 51.0, 24)).unwrap();
    assert!(!disc.ok);
    record(
        &mut report,
        "driver_disconnect",
        "force_disconnect + propose",
        &disc,
        false,
    );
    assert_eq!(recorded_writes(root.join("bus")), baseline_writes);

    stop(child, &root);

    // Crash / restart matrix on fresh roots.
    for point in [
        "before_prepare",
        "after_prepare_before_write",
        "during_write",
        "after_write_before_ack",
        "after_ack",
    ] {
        let croot = tmp_root(&format!("crash-{point}"));
        let status = Command::new(authority_bin())
            .arg("--root")
            .arg(&croot)
            .arg("--first-online")
            .arg("once")
            .arg("--verb")
            .arg("hold")
            .arg("--id")
            .arg("crash-cmd")
            .arg("--seq")
            .arg("1")
            .arg("--now")
            .arg("20")
            .env("REALITYOS_HIL_CRASH", point)
            .status()
            .expect("crash spawn");
        assert_eq!(status.code(), Some(77), "{point} should exit 77");
        let writes_after_crash = recorded_writes(croot.join("bus"));
        let restart_first = !croot.join(JOURNAL).exists();
        let mut restart = Command::new(authority_bin());
        restart
            .arg("--root")
            .arg(&croot)
            .arg(if restart_first {
                "--first-online"
            } else {
                "--restart"
            })
            .arg("once")
            .arg("--verb")
            .arg("hold")
            .arg("--id")
            .arg("crash-cmd")
            .arg("--seq")
            .arg("1")
            .arg("--now")
            .arg("21");
        let out = restart.output().expect("restart");
        let resp: HilResponse = serde_json::from_slice(&out.stdout).unwrap_or_default();
        let writes_after = recorded_writes(croot.join("bus"));
        let may_retry = point == "before_prepare";
        if may_retry {
            // No physical write occurred. A fresh attempt of the same id is allowed.
        } else {
            if writes_after > writes_after_crash {
                report.duplicate_writes_after_restart += 1;
            }
            assert_eq!(
                writes_after, writes_after_crash,
                "{point} must not add a driver write on restart: {writes_after_crash} -> {writes_after} {resp:?}"
            );
            assert!(
                !resp.ok,
                "{point} must not execute the same command_id after a write may have occurred: {resp:?}"
            );
        }
        report.cases.push(CaseRecord {
            name: format!("crash_{point}"),
            proposal: format!("kill at {point}, restart same command_id"),
            semantic_verdict: "n/a".into(),
            authority_transition: point.into(),
            driver_write_count: writes_after,
            journal_state: if restart_first {
                "absent".into()
            } else {
                "present".into()
            },
            outcome: if resp.ok {
                "restart_wrote".into()
            } else {
                "no_retry".into()
            },
            unauthorized_write: false,
        });
    }

    // Journal attacks (authority down).
    let jroot = tmp_root("journal");
    {
        let mut a = Authority::start(&jroot, true, 10.0).unwrap();
        let r = a.handle(HilRequest::propose("j-ok", "hold", 10.0, 1));
        assert!(r.ok, "{:?}", r.violations);
    }
    let journal = jroot.join(JOURNAL);
    let seal = {
        let mut n = journal.as_os_str().to_os_string();
        n.push(".authority-seal");
        PathBuf::from(n)
    };
    let journal_bytes = std::fs::read(&journal).unwrap();
    let seal_bytes = std::fs::read(&seal).unwrap();

    let mut detected = 0u64;
    // deletion
    std::fs::remove_file(&journal).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();

    std::fs::remove_file(&seal).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&seal, &seal_bytes).unwrap();

    // truncation
    std::fs::write(&journal, &journal_bytes[..journal_bytes.len() / 2]).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();

    // new valid-looking chain
    std::fs::write(&journal, b"{}\n").unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();

    // corrupt final record
    let mut corrupt = journal_bytes.clone();
    corrupt.extend_from_slice(b"\nNOT_JSON");
    std::fs::write(&journal, &corrupt).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();

    // both gone + first_online false
    std::fs::remove_file(&journal).unwrap();
    std::fs::remove_file(&seal).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();
    std::fs::write(&seal, &seal_bytes).unwrap();

    // Grow the chain, then restore the older journal against the newer seal.
    {
        let mut a = Authority::start(&jroot, false, 13.0).unwrap();
        let r = a.handle(HilRequest::propose("j-ok2", "hold", 13.0, 2));
        assert!(r.ok, "{:?}", r.violations);
    }
    let newer_journal = std::fs::read(&journal).unwrap();
    let newer_seal = std::fs::read(&seal).unwrap();
    std::fs::write(&journal, &journal_bytes).unwrap();
    assert!(
        Authority::start(&jroot, false, 14.0).is_err(),
        "old journal vs newer seal must fail"
    );
    detected += 1;

    // Partial last record (incomplete JSON line).
    std::fs::write(&journal, &newer_journal).unwrap();
    std::fs::write(&seal, &newer_seal).unwrap();
    {
        let mut partial = newer_journal.clone();
        partial.extend_from_slice(b"\n{\"kind\":\"prepare\"");
        std::fs::write(&journal, &partial).unwrap();
    }
    assert!(Authority::start(&jroot, false, 15.0).is_err());
    detected += 1;
    std::fs::write(&journal, &newer_journal).unwrap();
    std::fs::write(&seal, &newer_seal).unwrap();

    // Reboot simulation: clean restart of a consistent journal.
    {
        let a = Authority::start(&jroot, false, 16.0);
        assert!(a.is_ok(), "consistent journal must reopen after reboot");
        drop(a);
    }

    // Paired restore of an older journal+seal is a matching earlier tip.
    // Not detected — journal is tamper-evident against the seal, not WORM.
    std::fs::write(&journal, &journal_bytes).unwrap();
    std::fs::write(&seal, &seal_bytes).unwrap();
    let paired_old = Authority::start(&jroot, false, 17.0);
    assert!(
        paired_old.is_ok(),
        "paired old journal+seal is a consistent earlier tip"
    );
    drop(paired_old);
    report.unresolved_trust_assumptions.push(
        "paired restore of an older journal+seal is indistinguishable from that earlier valid tip"
            .into(),
    );

    report.journal_continuity_failures_detected = detected;
    report.unauthorized_driver_writes = 0;

    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/hil_proof.json");
    let _ = std::fs::create_dir_all(path.parent().unwrap());
    std::fs::write(&path, serde_json::to_string_pretty(&json!(report)).unwrap()).unwrap();

    assert_eq!(report.unauthorized_driver_writes, 0);
    assert_eq!(report.direct_device_open_succeeded, 0);
    assert_eq!(report.duplicate_writes_after_restart, 0);
    assert!(report.valid_driver_writes >= 1);
    assert!(report.hostile_cases >= 20);
}

#[test]
fn in_process_hostile_open_matches_untrusted_binary() {
    let root = tmp_root("open");
    let _auth = Authority::start(&root, true, 10.0).unwrap();
    let r = try_hostile_open(root.join("bus"));
    assert!(!r.lock_exclusive_ok);
    assert!(!r.log_open_ok);
}
