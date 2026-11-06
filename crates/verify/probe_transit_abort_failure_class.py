#!/usr/bin/env python3
"""Classify sealed rotated-probe episodes without weakening Governor gates.

Maps episode/result fields into failure classes used for research prioritization.
Does not authorize contact, change expected-contact policy, or claim Coulomb mu.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

SCHEMA = "realityos.probe_transit_abort_failure_class/v1"


def evidence_dir_has_episode_diagnostic(evidence_dir: Path | None) -> bool:
    """True when a sealed episode-diagnostic artifact exists under evidence_dir."""
    if evidence_dir is None or not evidence_dir.is_dir():
        return False
    direct = evidence_dir / "PROBE_MEASUREMENT_SIDECAR.json.episode-diagnostic.json"
    if direct.is_file():
        return True
    return any(evidence_dir.glob("*.episode-diagnostic.json"))


def classify_episode(
    episode: dict,
    result: dict | None = None,
    inference_bound_diagnosis: dict | None = None,
    *,
    episode_diagnostic_present: bool | None = None,
    evidence_dir: Path | None = None,
) -> dict:
    result = result or {}
    outcome = episode.get("outcome") or episode.get("trial_outcome")
    failure_cause = episode.get("failure_cause") or (
        (result.get("run") or {}).get("failure_cause")
    )
    refusal = episode.get("refusal_code") or (
        (result.get("run") or {}).get("refusal_code")
    )
    wrench_abort = episode.get("wrench_abort") or (
        (result.get("run") or {}).get("wrench_abort_reason")
    )
    unauthorized = (
        episode.get("unauthorized_writes")
        if "unauthorized_writes" in episode
        else (result.get("run") or {}).get("unauthorized_writes")
    )
    sidecar_status = (
        (result.get("verification") or {}).get("offline_sidecar_verifier_status")
        or (result.get("diagnosis") or {}).get("offline_sidecar_verification")
    )
    if episode_diagnostic_present is None:
        episode_diagnostic_present = evidence_dir_has_episode_diagnostic(evidence_dir)
    wrench_inference_reason = episode.get("wrench_inference_reason") or ""
    diagnosis_rows = (
        (inference_bound_diagnosis or {}).get("measurements")
        if isinstance(inference_bound_diagnosis, dict)
        else None
    ) or []
    prior_incompatible_offline = any(
        row.get("diagnosis")
        in {
            "measurement_bounds_do_not_overlap_prior",
            "entirely_above_prior",
            "entirely_below_prior",
        }
        for row in diagnosis_rows
        if isinstance(row, dict)
    )
    abort_kind = None
    if isinstance(wrench_abort, dict):
        abort_kind = wrench_abort.get("kind")
        if abort_kind is None and isinstance(wrench_abort.get("reason"), dict):
            abort_kind = wrench_abort["reason"].get("kind")
    if abort_kind is None and isinstance(result.get("run"), dict):
        nested = (result.get("run") or {}).get("wrench_abort_reason")
        if isinstance(nested, dict):
            abort_kind = nested.get("kind")

    failure_class = "unclassified"
    notes = []
    if unauthorized not in (None, 0):
        failure_class = "unauthorized_write"
    elif abort_kind == "unexpected_contact" and sidecar_status in {
        "NOT_RUN_NO_SIDECAR",
        "UNAVAILABLE_NO_SIDECAR",
        "FAIL_NO_SIDECAR",
        "FAIL_EMPTY_MEASUREMENTS",
        "FAIL_UNREADABLE",
        None,
    }:
        if episode_diagnostic_present:
            # 21004/21005 after diagnostic capture: episode preserved, but the
            # probe-measurement sidecar still failed verification / was not run.
            failure_class = "unexpected_contact_with_episode_diagnostic_only"
            if sidecar_status in {
                "FAIL_NO_SIDECAR",
                "FAIL_EMPTY_MEASUREMENTS",
                "FAIL_UNREADABLE",
            }:
                notes.append(
                    "Governor refusal preserved. Offline sidecar verification "
                    f"completed as {sidecar_status}; episode diagnostic remains. "
                    "Do not invent measurements or weaken contact/force gates. "
                    "Next: seal a non-empty probe-measurement sidecar on a fresh "
                    "authorized capture (or owner-gated 21008 replay)."
                )
            else:
                notes.append(
                    "Governor refusal preserved. Episode diagnostic exists, but the "
                    "probe-measurement sidecar is still missing or unverified. Next: "
                    "complete sidecar verification — do not weaken contact/force gates."
                )
        else:
            # Transit abort with neither sidecar nor episode diagnostic.
            failure_class = "transit_unexpected_contact_without_measurement_trace"
            notes.append(
                "Correct refusal may still apply; diagnostics were unavailable because "
                "no probe measurement sidecar was sealed."
            )
    elif abort_kind == "unexpected_contact":
        failure_class = "unexpected_contact_with_preserved_episode"
        notes.append("Governor refusal preserved; inspect sealed trace before any policy change.")
    elif (
        outcome == "unexpected_refusal"
        and sidecar_status in {"PASS", "pass", "OK"}
        and (
            prior_incompatible_offline
            or "PriorMeasurementIncompatible" in wrench_inference_reason
            or wrench_inference_reason.startswith("no_inference:EmptyIntersection")
        )
    ):
        # 21003 pattern: first SlipOnset update narrows prior; later onset bounds
        # sit entirely outside that prior (offline: measurement_bounds_do_not_overlap_prior).
        if prior_incompatible_offline or "PriorMeasurementIncompatible" in wrench_inference_reason:
            first_onset = episode.get("first_slip_onset_interval")
            prior_before = episode.get("support_friction_prior_before_first_onset")
            # Sealed 21003 episodes may omit first_slip_* episode fields; recover the
            # successive pattern from offline diagnosis (Updated/slip_onset then
            # measurement_bounds_do_not_overlap_prior). Do not relabel that as
            # first-onset-outside-prior.
            diagnosis_had_slip_onset_update = any(
                isinstance(row, dict)
                and (
                    row.get("diagnosis") == "updated"
                    or (
                        isinstance(row.get("inference"), dict)
                        and isinstance(row["inference"].get("Updated"), dict)
                        and row["inference"]["Updated"].get("mode") == "slip_onset"
                    )
                )
                for row in diagnosis_rows
            )
            had_first_onset_narrowing = (
                first_onset is not None
                or prior_before is not None
                or diagnosis_had_slip_onset_update
            )
            entirely_above = any(
                row.get("diagnosis")
                in {"entirely_above_prior", "measurement_bounds_do_not_overlap_prior"}
                for row in diagnosis_rows
                if isinstance(row, dict)
            )
            # PreferHigherLoadFamily only applies after a first SlipOnset narrowing.
            # First onset entirely above the declared prior is a distinct hole.
            if not had_first_onset_narrowing:
                failure_class = (
                    "expected_success_blocked_by_first_onset_outside_prior"
                )
                notes.append(
                    "Sealed probe measurement exists; first SlipOnset bounds sit outside "
                    "the declared prior (often entirely above prior.upper) with no prior "
                    "first-onset posterior to escalate from. PreferHigherLoadFamily does "
                    "not apply. Do not weaken Governor contact/force thresholds; research "
                    "prior envelope honesty or seed/physics match instead."
                )
                if entirely_above:
                    notes.append(
                        "Offline bound diagnosis: measurement_bounds entirely_above_prior."
                    )
            else:
                failure_class = (
                    "expected_success_blocked_by_successive_prior_incompatibility"
                )
                notes.append(
                    "Sealed probe measurements exist; empty prior∩measurement is successive-"
                    "probe incompatibility after a first SlipOnset narrowing. "
                    "Do not weaken Governor contact/force thresholds."
                )
        else:
            failure_class = "expected_success_blocked_by_empty_friction_intersection"
            notes.append(
                "Sealed probe measurements exist; Governor did not weaken. Next research "
                "should attack inference/cadence bounds, not contact-force thresholds."
            )
    elif failure_cause == "force_monitor_abort":
        failure_class = "force_monitor_abort_other"
    elif outcome in {"success", "Success", "completed"}:
        failure_class = "task_success"

    return {
        "schema": SCHEMA,
        "failure_class": failure_class,
        "outcome": outcome,
        "failure_cause": failure_cause,
        "refusal_code": refusal,
        "abort_kind": abort_kind,
        "unauthorized_writes": unauthorized,
        "sidecar_status": sidecar_status,
        "episode_diagnostic_present": bool(episode_diagnostic_present),
        "notes": notes,
        "governor_weakened": False,
        "physical_test": False,
        "admission_status": "NOT_ADMITTED",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--result", type=Path)
    parser.add_argument(
        "--evidence-dir",
        type=Path,
        help="Optional sealed evidence directory; detects *.episode-diagnostic.json",
    )
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    episode = json.loads(args.episode.read_text(encoding="utf-8"))
    result = (
        json.loads(args.result.read_text(encoding="utf-8")) if args.result else None
    )
    report = classify_episode(episode, result, evidence_dir=args.evidence_dir)
    text = json.dumps(report, indent=2) + "\n"
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
