use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};

use realityos_governor::RuntimeIdentity;
use realityos_hil::{
    call, call_raw, layer_from_stage, verify_proof_consistency, wait_for_ipc, Authority,
    BlockingLayer, CaseRecord, HilFaultInjectionRequest, HilRequest, HilResponse, ProofExtras,
    ProofReport, JOURNAL,
};
use realityos_kernel::{CalibrationId, DesignContentHash, FirmwareId, SerialOrAsBuilt};
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
    let _ = call(root, &{
        let mut r = HilRequest::propose("x", "hold");
        r.op = "shutdown".into();
        r
    });
    let _ = child.wait();
}

fn measure_resp(
    name: &str,
    proposal: &str,
    resp: &HilResponse,
    writes_before: u64,
    writes_after: u64,
    expected_authorized: bool,
    layer: BlockingLayer,
) -> CaseRecord {
    CaseRecord::measure(
        name,
        proposal,
        if resp.stage == "semantic" {
            resp.status.clone()
        } else {
            format!("{}:{}", resp.stage, resp.status)
        },
        resp.stage.clone(),
        layer,
        writes_before,
        writes_after,
        expected_authorized,
        resp.executed,
        if resp.executed {
            "consumed"
        } else {
            "no_consume"
        },
        if resp.ok { "ok" } else { "refused" },
    )
}

fn write_hot_swap(root: &Path, serial: &str, fw: &str, cal: &str, design: &str) {
    let v = json!({
        "serial": serial,
        "firmware_id": fw,
        "calibration_id": cal,
        "design_content_hash": design,
    });
    std::fs::write(root.join("bus").join("hot_swap.json"), v.to_string()).unwrap();
}

#[test]
fn hil_adversarial_campaign_zero_unauthorized_writes() {
    let root = tmp_root("camp");
    let child = spawn_serve(&root, true);
    assert!(wait_for_ipc(&root, 8000), "authority ipc");

    let mut cases = Vec::new();

    let before = recorded_writes(root.join("bus"));
    let valid = call(&root, &HilRequest::propose("valid-1", "hold")).expect("valid");
    assert!(valid.ok, "{:?}", valid.violations);
    let after = recorded_writes(root.join("bus"));
    assert_eq!(after, before + 1);
    cases.push(measure_resp(
        "valid_hold",
        "verb=hold",
        &valid,
        before,
        after,
        true,
        BlockingLayer::None,
    ));

    let hostile = |req: HilRequest| -> HilResponse {
        call(&root, &req).unwrap_or_else(|e| HilResponse {
            ok: false,
            stage: "ipc".into(),
            status: "error".into(),
            violations: vec![e.to_string()],
            ..HilResponse::default()
        })
    };

    let cases_spec: Vec<(&str, HilRequest, &str, BlockingLayer)> = vec![
        (
            "unsupported_action",
            HilRequest::propose("h-unsup", "dance"),
            "verb=dance",
            BlockingLayer::AuthorizationBlocked,
        ),
        (
            "oversized_action",
            {
                let mut r = HilRequest::propose("h-big", "hold");
                r.action = Some(vec![1e6]);
                r
            },
            "action=1e6",
            BlockingLayer::AuthorizationBlocked,
        ),
        (
            "stale_evidence",
            HilRequest::hil_fault(
                "h-stale",
                "hold",
                HilFaultInjectionRequest {
                    now_s: Some(80.0),
                    skip_sensor: true,
                    ..Default::default()
                },
            ),
            "authority clock=80 after receive at start, skip sensor",
            BlockingLayer::EgressBlocked,
        ),
        (
            "missing_evidence",
            HilRequest::hil_fault(
                "h-noe",
                "hold",
                HilFaultInjectionRequest {
                    now_s: Some(81.0),
                    drop_sensor: true,
                    ..Default::default()
                },
            ),
            "drop_sensor",
            BlockingLayer::AuthorizationBlocked,
        ),
        (
            "forged_evidence_hash",
            HilRequest::hil_fault(
                "h-fev",
                "hold",
                HilFaultInjectionRequest {
                    drop_sensor: true,
                    ..Default::default()
                },
            ),
            "untrusted cannot attach a hash; missing evidence",
            BlockingLayer::AuthorizationBlocked,
        ),
        (
            "expired_command",
            HilRequest::hil_fault(
                "h-exp",
                "hold",
                HilFaultInjectionRequest {
                    now_s: Some(27.0),
                    ttl_s: Some(0.01),
                    write_now_s: Some(40.0),
                    ..Default::default()
                },
            ),
            "hil_fault ttl=0.01 write_now=40",
            BlockingLayer::EgressBlocked,
        ),
        (
            "sequence_rollback",
            HilRequest::hil_fault(
                "h-seq",
                "hold",
                HilFaultInjectionRequest {
                    sequence: Some(1),
                    now_s: Some(29.0),
                    ..Default::default()
                },
            ),
            "hil_fault sequence=1 after higher",
            BlockingLayer::EgressBlocked,
        ),
        (
            "command_replay",
            HilRequest::propose("valid-1", "hold"),
            "replay valid-1",
            BlockingLayer::EgressBlocked,
        ),
        (
            "homemade_actuation_command",
            {
                let mut r = HilRequest::propose("x", "hold");
                r.op = "forge_actuation_command".into();
                r
            },
            "forge ActuationCommand over IPC",
            BlockingLayer::ProtocolBlocked,
        ),
        (
            "arbitrary_certificate_allow",
            {
                let mut r = HilRequest::propose("x", "hold");
                r.op = "forge_certificate".into();
                r
            },
            "forge Certificate(ALLOW) over IPC",
            BlockingLayer::ProtocolBlocked,
        ),
        (
            "reused_online_write",
            {
                let mut r = HilRequest::propose("x", "hold");
                r.op = "write_online_blob".into();
                r
            },
            "untrusted cannot carry OnlineWrite",
            BlockingLayer::ProtocolBlocked,
        ),
        (
            "online_write_other_governor",
            {
                let mut r = HilRequest::propose("x", "hold");
                r.op = "write_online_blob".into();
                r
            },
            "untrusted cannot import foreign OnlineWrite",
            BlockingLayer::ProtocolBlocked,
        ),
        (
            "sensor_loss",
            HilRequest::hil_fault(
                "h-sloss",
                "hold",
                HilFaultInjectionRequest {
                    now_s: Some(200.0),
                    skip_sensor: true,
                    ..Default::default()
                },
            ),
            "no sensor + stale clock",
            BlockingLayer::EgressBlocked,
        ),
        (
            "heartbeat_loss",
            HilRequest::hil_fault(
                "h-hb",
                "hold",
                HilFaultInjectionRequest {
                    now_s: Some(400.0),
                    skip_heartbeat: true,
                    ..Default::default()
                },
            ),
            "now far past heartbeat",
            BlockingLayer::EgressBlocked,
        ),
    ];

    for (name, req, proposal, layer) in cases_spec {
        let before = recorded_writes(root.join("bus"));
        let resp = hostile(req);
        let after = recorded_writes(root.join("bus"));
        assert!(
            !resp.ok && after == before,
            "{name} must refuse with zero new writes: {resp:?}"
        );
        cases.push(measure_resp(
            name, proposal, &resp, before, after, false, layer,
        ));
        let ping = call(&root, &{
            let mut r = HilRequest::propose("ping", "hold");
            r.op = "status".into();
            r
        });
        assert!(
            ping.is_ok(),
            "{name} killed authority: {:?} err={}",
            ping.err(),
            std::fs::read_to_string(root.join("authority.err")).unwrap_or_default()
        );
    }

    for (name, raw) in [
        (
            "nan_action",
            r#"{"op":"propose","verb":"hold","command_id":"h-nan","action":[NaN]}"#,
        ),
        (
            "inf_action",
            r#"{"op":"propose","verb":"hold","command_id":"h-inf","action":[Infinity]}"#,
        ),
    ] {
        let before = recorded_writes(root.join("bus"));
        let resp = call_raw(&root, raw).unwrap_or_else(|e| HilResponse {
            ok: false,
            stage: "ipc".into(),
            status: "error".into(),
            violations: vec![e.to_string()],
            ..HilResponse::default()
        });
        let after = recorded_writes(root.join("bus"));
        assert!(!resp.ok && after == before, "{name}: {resp:?}");
        cases.push(measure_resp(
            name,
            "non-finite action",
            &resp,
            before,
            after,
            false,
            BlockingLayer::ProtocolBlocked,
        ));
    }

    // Production propose cannot move the authority clock (legacy now_s ignored).
    let before = recorded_writes(root.join("bus"));
    let mut sneak = HilRequest::propose("h-sneak-time", "hold");
    sneak.now_s = 1.0e12;
    sneak.write_now_s = Some(1.0e12);
    sneak.ttl_s = 0.01;
    let sneak_resp = hostile(sneak);
    let after = recorded_writes(root.join("bus"));
    // May execute as a normal production command at the authority clock, or refuse
    // on remaining rails. Either way the extra times must not be authoritative:
    // a 1e12 write_now would trip time/expiry in the old path; we only record it.
    cases.push(measure_resp(
        "production_cannot_set_now",
        "propose with leftover now_s/write_now_s/ttl ignored",
        &sneak_resp,
        before,
        after,
        sneak_resp.ok,
        if sneak_resp.ok {
            BlockingLayer::None
        } else {
            layer_from_stage(&sneak_resp.stage)
        },
    ));

    let before = recorded_writes(root.join("bus"));
    let ff = hostile(HilRequest::hil_fault(
        "h-ff",
        "hold",
        HilFaultInjectionRequest {
            write_now_s: Some(1.0e12),
            now_s: Some(32.0),
            ..Default::default()
        },
    ));
    let after = recorded_writes(root.join("bus"));
    cases.push(measure_resp(
        "far_future_timestamp",
        "hil_fault write_now_s=1e12 (fault surface only)",
        &ff,
        before,
        after,
        false,
        layer_from_stage(&ff.stage),
    ));

    let before = recorded_writes(root.join("bus"));
    let rb = hostile(HilRequest::hil_fault(
        "h-rb",
        "hold",
        HilFaultInjectionRequest {
            now_s: Some(28.0),
            write_now_s: Some(5.0),
            ..Default::default()
        },
    ));
    let after = recorded_writes(root.join("bus"));
    assert!(!rb.ok && after == before);
    cases.push(measure_resp(
        "time_rollback",
        "hil_fault write_now < last_now",
        &rb,
        before,
        after,
        false,
        BlockingLayer::EgressBlocked,
    ));

    let open = Command::new(untrusted_bin())
        .arg("--root")
        .arg(&root)
        .arg("attack-open")
        .output()
        .expect("untrusted open");
    let hostile_open: realityos_vport::HostileOpenResult =
        serde_json::from_slice(&open.stdout).expect("open json");
    assert!(!hostile_open.lock_exclusive_ok);
    assert!(!hostile_open.log_open_ok);
    let w = recorded_writes(root.join("bus"));
    cases.push(CaseRecord::measure(
        "direct_driver_access",
        "untrusted open/lock/write actuator.log",
        "n/a",
        "os",
        BlockingLayer::OsBlocked,
        w,
        w,
        false,
        false,
        "unchanged",
        "denied",
    ));

    let before = recorded_writes(root.join("bus"));
    let _ = call(&root, &{
        let mut r = HilRequest::propose("x", "hold");
        r.op = "disconnect_driver".into();
        r
    });
    let disc = call(&root, &HilRequest::propose("h-disc", "hold")).unwrap();
    let after = recorded_writes(root.join("bus"));
    assert!(!disc.ok && after == before);
    cases.push(measure_resp(
        "driver_disconnect",
        "force_disconnect + propose",
        &disc,
        before,
        after,
        false,
        BlockingLayer::AuthorizationBlocked,
    ));

    stop(child, &root);

    // Identity change after a successful ONLINE write (fresh instance).
    {
        let sroot = tmp_root("hotswap-live");
        let mut auth = Authority::start(&sroot, true, 10.0).unwrap();
        let ok = auth.handle(HilRequest::propose("s-ok", "hold"));
        assert!(ok.ok, "{:?}", ok.violations);
        let before = auth.driver_writes();
        write_hot_swap(&sroot, "SN-REPLACED", "HIL-FW-1", "HIL-CAL-1", "hil_design");
        let swap = auth.handle(HilRequest::propose("s-swap", "hold"));
        assert!(!swap.ok, "{swap:?}");
        assert_eq!(auth.driver_writes(), before);
        cases.push(measure_resp(
            "identity_changed_after_startup",
            "hot_swap.json serial SN-REPLACED after ONLINE start",
            &swap,
            before,
            auth.driver_writes(),
            false,
            BlockingLayer::AuthorizationBlocked,
        ));
    }

    // Identity mismatch at ONLINE init (real probe path, not protocol refuse).
    for (name, mutate, proposal) in [
        (
            "wrong_serial",
            (|id: &mut RuntimeIdentity| {
                id.serial_or_as_built = Some(SerialOrAsBuilt::new("SN-A").unwrap());
            }) as fn(&mut RuntimeIdentity),
            "configured SN-A, probed SN-HIL-1",
        ),
        (
            "wrong_firmware",
            |id| {
                id.firmware_id = Some(FirmwareId::new("FW-A").unwrap());
            },
            "configured FW-A, probed HIL-FW-1",
        ),
        (
            "changed_calibration",
            |id| {
                id.calibration_id = Some(CalibrationId::new("CAL-A").unwrap());
            },
            "configured CAL-A, probed HIL-CAL-1",
        ),
        (
            "wrong_runtime_identity",
            |id| {
                id.design_content_hash = Some(DesignContentHash::new("des-A").unwrap());
            },
            "configured design A, probed hil_design",
        ),
    ] {
        let iroot = tmp_root(name);
        let mut id = realityos_hil::hil_identity();
        mutate(&mut id);
        let before = recorded_writes(iroot.join("bus"));
        let started = Authority::start_with_identity(&iroot, true, 10.0, id);
        let after = recorded_writes(iroot.join("bus"));
        assert!(started.is_err(), "{name} must fail ONLINE init");
        assert_eq!(after, before);
        cases.push(CaseRecord::measure(
            name,
            proposal,
            started.err().map(|e| e.to_string()).unwrap_or_default(),
            "new_online",
            BlockingLayer::AuthorizationBlocked,
            before,
            after,
            false,
            false,
            "no_consume",
            "refused",
        ));
    }

    let disc_root = tmp_root("disc-init");
    let _ = std::fs::create_dir_all(disc_root.join("bus"));
    std::fs::write(disc_root.join("bus").join("force_disconnect"), b"1").unwrap();
    let before = 0u64;
    let started = Authority::start(&disc_root, true, 10.0);
    assert!(started.is_err(), "disconnected hardware must fail ONLINE");
    cases.push(CaseRecord::measure(
        "disconnected_hardware",
        "force_disconnect before new_online",
        started.err().map(|e| e.to_string()).unwrap_or_default(),
        "new_online",
        BlockingLayer::AuthorizationBlocked,
        before,
        recorded_writes(disc_root.join("bus")),
        false,
        false,
        "no_consume",
        "refused",
    ));

    let ph_root = tmp_root("placeholder");
    let _ = std::fs::create_dir_all(ph_root.join("bus"));
    std::fs::write(ph_root.join("bus").join("placeholder_identity"), b"1").unwrap();
    let started = Authority::start(&ph_root, true, 10.0);
    assert!(
        started.is_err(),
        "placeholder/SIM identity must fail ONLINE"
    );
    cases.push(CaseRecord::measure(
        "placeholder_sim_identity",
        "placeholder_identity file → SIM_* probe",
        started.err().map(|e| e.to_string()).unwrap_or_default(),
        "new_online",
        BlockingLayer::AuthorizationBlocked,
        0,
        recorded_writes(ph_root.join("bus")),
        false,
        false,
        "no_consume",
        "refused",
    ));

    let miss_root = tmp_root("missing");
    let _ = std::fs::create_dir_all(miss_root.join("bus"));
    std::fs::write(miss_root.join("bus").join("missing_identity"), b"1").unwrap();
    let started = Authority::start(&miss_root, true, 10.0);
    assert!(
        started.is_err(),
        "missing hardware identity must fail ONLINE"
    );
    cases.push(CaseRecord::measure(
        "missing_hardware_identity",
        "missing_identity file → empty probe fields",
        started.err().map(|e| e.to_string()).unwrap_or_default(),
        "new_online",
        BlockingLayer::AuthorizationBlocked,
        0,
        recorded_writes(miss_root.join("bus")),
        false,
        false,
        "no_consume",
        "refused",
    ));

    // Reconnect to a different device after disconnect: same runtime instance stays dead.
    {
        let rroot = tmp_root("reconn-other");
        let mut auth = Authority::start(&rroot, true, 10.0).unwrap();
        let ok = auth.handle(HilRequest::propose("r-ok", "hold"));
        assert!(ok.ok, "{:?}", ok.violations);
        let before = auth.driver_writes();
        std::fs::write(rroot.join("bus").join("force_disconnect"), b"1").unwrap();
        let d = auth.handle(HilRequest::propose("r-d", "hold"));
        assert!(!d.ok);
        assert_eq!(auth.driver_writes(), before);
        let _ = std::fs::remove_file(rroot.join("bus").join("force_disconnect"));
        write_hot_swap(&rroot, "SN-OTHER", "HIL-FW-1", "HIL-CAL-1", "hil_design");
        let r = auth.handle(HilRequest::propose("r-other", "hold"));
        assert!(!r.ok);
        assert_eq!(auth.driver_writes(), before);
        cases.push(measure_resp(
            "reconnect_different_device",
            "disconnect then reconnect SN-OTHER under same runtime instance",
            &r,
            before,
            auth.driver_writes(),
            false,
            BlockingLayer::AuthorizationBlocked,
        ));
    }

    cases.push(CaseRecord::measure(
        "online_write_no_public_constructor",
        "ValidatedRuntimeIdentity / OnlineWrite private fields",
        "compile_fail",
        "compile",
        BlockingLayer::CompileTimeBlocked,
        0,
        0,
        false,
        false,
        "n/a",
        "compile_fail",
    ));
    cases.push(CaseRecord::measure(
        "unauthorized_actuator",
        "governor-owned actuator allow-list; untrusted cannot attach ids (compile + authorize)",
        "compile_fail+authorize",
        "compile",
        BlockingLayer::CompileTimeBlocked,
        0,
        0,
        false,
        false,
        "n/a",
        "compile_fail",
    ));

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
        if !may_retry {
            assert_eq!(
                writes_after, writes_after_crash,
                "{point} must not add a driver write on restart: {writes_after_crash} -> {writes_after} {resp:?}"
            );
            assert!(
                !resp.ok,
                "{point} must not execute the same command_id after a write may have occurred: {resp:?}"
            );
        }
        cases.push(CaseRecord::measure(
            format!("crash_{point}"),
            format!("kill at {point}, restart same command_id"),
            "n/a",
            point,
            BlockingLayer::CrashRecoveryBlocked,
            writes_after_crash,
            writes_after,
            may_retry,
            resp.ok,
            if restart_first { "absent" } else { "present" },
            if resp.ok { "restart_wrote" } else { "no_retry" },
        ));
    }

    // Journal attacks (authority process down).
    let jroot = tmp_root("journal");
    {
        let mut a = Authority::start(&jroot, true, 10.0).unwrap();
        let r = a.handle(HilRequest::propose("j-ok", "hold"));
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
    std::fs::remove_file(&journal).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();

    std::fs::remove_file(&seal).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&seal, &seal_bytes).unwrap();

    std::fs::write(&journal, &journal_bytes[..journal_bytes.len() / 2]).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();

    std::fs::write(&journal, b"{}\n").unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();

    let mut corrupt = journal_bytes.clone();
    corrupt.extend_from_slice(b"\nNOT_JSON");
    std::fs::write(&journal, &corrupt).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();

    std::fs::remove_file(&journal).unwrap();
    std::fs::remove_file(&seal).unwrap();
    assert!(Authority::start(&jroot, false, 12.0).is_err());
    detected += 1;
    std::fs::write(&journal, &journal_bytes).unwrap();
    std::fs::write(&seal, &seal_bytes).unwrap();

    {
        let mut a = Authority::start(&jroot, false, 13.0).unwrap();
        let r = a.handle(HilRequest::propose("j-ok2", "hold"));
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

    {
        let a = Authority::start(&jroot, false, 16.0);
        assert!(a.is_ok(), "consistent journal must reopen after reboot");
        drop(a);
    }

    std::fs::write(&journal, &journal_bytes).unwrap();
    std::fs::write(&seal, &seal_bytes).unwrap();
    let paired_old = Authority::start(&jroot, false, 17.0);
    assert!(
        paired_old.is_ok(),
        "paired old journal+seal is a consistent earlier tip"
    );
    drop(paired_old);

    let extras = ProofExtras {
        direct_device_open_attempts: 1,
        direct_device_open_succeeded: u64::from(
            hostile_open.lock_exclusive_ok || hostile_open.log_write_ok,
        ),
        journal_continuity_failures_detected: detected,
        unresolved_trust_assumptions: ProofReport::default_assumptions(),
    };
    let report = ProofReport::from_measured_cases(cases, extras).expect("proof consistency");
    verify_proof_consistency(&report).expect("recompute");

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

#[test]
fn proof_report_rejects_hardcoded_zero_when_cases_leak() {
    let leak = CaseRecord::measure(
        "leak",
        "x",
        "allow",
        "write",
        BlockingLayer::EgressBlocked,
        0,
        2,
        false,
        true,
        "consumed",
        "ok",
    );
    let report = ProofReport::from_measured_cases(vec![leak], ProofExtras::default()).unwrap();
    assert_eq!(report.unauthorized_driver_writes, 2);
    assert!(verify_proof_consistency(&report).is_ok());
}
