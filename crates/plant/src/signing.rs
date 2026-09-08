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
        "command_id": cmd.command_id(),
        "sequence": cmd.sequence(),
        "issued_at_s": round9(cmd.issued_at_s()),
        "expires_at_s": round9(cmd.expires_at_s()),
        "issuer_certificate_status": cmd.issuer_certificate_status(),
        "issuer_physical_reason": cmd.issuer_physical_reason(),
        "issuer_allowed_action": cmd.issuer_allowed_action().iter().map(|x| round9(*x)).collect::<Vec<_>>(),
        "sensor_snapshot_id": cmd.sensor_snapshot_id(),
        "belief_snapshot_id": cmd.belief_snapshot_id(),
    })
}

pub fn command_payload_hash(cmd: &dyn ActuationCommand) -> String {
    let raw = serde_json::to_string(&command_payload_for_sign(cmd)).unwrap_or_default();
    hex::encode(Sha256::digest(raw.as_bytes()))
}

pub fn sign_payload(key: &[u8], payload_hash: &str) -> String {
    let mut mac = HmacSha256::new_from_slice(key).expect("HMAC key");
    mac.update(payload_hash.as_bytes());
    hex::encode(mac.finalize().into_bytes())
}

pub fn action_within_issuer_envelope(live: &[f64], issuer: &[f64]) -> bool {
    if issuer.is_empty() {
        return true;
    }
    if live.len() != issuer.len() {
        return false;
    }
    live.iter()
        .zip(issuer.iter())
        .all(|(v, old)| v.is_finite() && v.abs() <= old.abs() + 1e-12)
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
    v
}

fn round9(x: f64) -> f64 {
    if !x.is_finite() {
        return 0.0;
    }
    (x * 1e9).round() / 1e9
}
