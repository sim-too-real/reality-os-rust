//! HMAC-SHA256 capability envelope. Symmetric shared key — not Ed25519 non-repudiation.

use hmac::{Hmac, Mac};
use serde_json::{json, Value};
use sha2::{Digest, Sha256};

use crate::command::ActuationCommand;

type HmacSha256 = Hmac<Sha256>;

pub const SIGNING_SCHEME: &str = "hmac-sha256";
pub const SCHEMA: &str = "realityos.command_signing/1";

pub fn command_payload_for_sign(cmd: &dyn ActuationCommand) -> Value {
    json!({
        "schema": SCHEMA,
        "command_id": cmd.command_id(),
        "sequence": cmd.sequence(),
        "issued_at_s": round9_strict(cmd.issued_at_s()),
        "expires_at_s": round9_strict(cmd.expires_at_s()),
        "issuer_certificate_status": cmd.issuer_certificate_status(),
        "live_certificate_status": cmd.certificate_status().as_str(),
        "issuer_physical_reason": cmd.issuer_physical_reason(),
        "issuer_allowed_action": cmd.issuer_allowed_action().iter().map(|x| round9_strict(*x)).collect::<Vec<_>>(),
        "allowed_action": cmd.allowed_action().iter().map(|x| round9_strict(*x)).collect::<Vec<_>>(),
        "actuator_ids": cmd.actuator_ids(),
        "waypoints": cmd.follow_waypoints().unwrap_or(&[]),
        "release_hash": cmd.release_hash(),
        "as_built_hash": cmd.as_built_hash(),
        "calibration_ids": cmd.calibration_ids(),
        "sensor_snapshot_id": cmd.sensor_snapshot_id(),
        "sensor_packet_hash": cmd.sensor_packet_hash(),
        "belief_snapshot_id": cmd.belief_snapshot_id(),
        "parent_payload_hash": cmd.parent_payload_hash(),
        "mode": cmd.mode(),
        "frame_id": cmd.frame_id(),
        "units": cmd.units(),
        "policy_hash": cmd.policy_hash(),
        "config_hash": cmd.config_hash(),
        "runtime_instance_hash": cmd.runtime_instance_hash(),
    })
}

pub fn command_payload_hash(cmd: &dyn ActuationCommand) -> String {
    let raw = serde_json::to_string(&command_payload_for_sign(cmd)).unwrap_or_default();
    hex::encode(Sha256::digest(raw.as_bytes()))
}

pub fn signing_key_hash(key: &[u8]) -> String {
    hex::encode(Sha256::digest(key))
}

pub fn sign_payload(key: &[u8], payload_hash: &str) -> String {
    let mut mac = HmacSha256::new_from_slice(key).expect("HMAC key");
    mac.update(payload_hash.as_bytes());
    hex::encode(mac.finalize().into_bytes())
}

/// Component-wise containment. Sign reversal is widening.
pub fn action_within_issuer_envelope(live: &[f64], issuer: &[f64]) -> bool {
    if issuer.is_empty() {
        return true;
    }
    if live.len() != issuer.len() {
        return false;
    }
    live.iter().zip(issuer.iter()).all(|(v, old)| {
        v.is_finite()
            && old.is_finite()
            && v.abs() <= old.abs() + 1e-12
            && (*v == 0.0 || v.signum() == old.signum())
    })
}

pub fn signature_violations(
    cmd: &dyn ActuationCommand,
    key: Option<&[u8]>,
    require_signature: bool,
) -> Vec<String> {
    let mut v = Vec::new();
    if !require_signature && key.is_none() {
        return v;
    }
    if cmd.signing_scheme() != SIGNING_SCHEME && !cmd.signing_scheme().is_empty() {
        v.push("signing_scheme_unsupported".into());
    }
    if cmd.signature().is_empty() {
        v.push("command_unsigned".into());
        return v;
    }
    let Some(key) = key else {
        v.push("signing_key_missing".into());
        return v;
    };
    let expect_hash = command_payload_hash(cmd);
    if !cmd.payload_hash().is_empty() && cmd.payload_hash() != expect_hash {
        v.push("payload_hash_mismatch".into());
    }
    let expect_sig = sign_payload(key, &expect_hash);
    if cmd.signature() != expect_sig {
        v.push("signature_mismatch".into());
    }
    let issuer = cmd.issuer_allowed_action();
    if !issuer.is_empty() && !action_within_issuer_envelope(cmd.allowed_action(), issuer) {
        v.push("live_action_outside_issuer_envelope".into());
    }
    if cmd.certificate_status().as_str() != cmd.issuer_certificate_status()
        && cmd.issuer_certificate_status() != "allow"
        && cmd.certificate_status().allowed()
    {
        v.push("live_certificate_upgraded_past_issuer".into());
    }
    v
}

fn round9_strict(x: f64) -> Value {
    if !x.is_finite() {
        return Value::Null;
    }
    json!((x * 1e9).round() / 1e9)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sign_reversal_is_widening() {
        assert!(action_within_issuer_envelope(&[0.1], &[0.5]));
        assert!(!action_within_issuer_envelope(&[-0.1], &[0.5]));
        assert!(action_within_issuer_envelope(&[0.0], &[0.5]));
    }

    #[test]
    fn non_finite_is_not_mapped_to_zero() {
        assert!(round9_strict(f64::NAN).is_null());
        assert!(round9_strict(f64::INFINITY).is_null());
    }
}
