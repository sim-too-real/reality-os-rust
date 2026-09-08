//! Driver-owned replay / monotonic journal. Same hash chain as Python CommandLedger.
//! Decision attestation is a *second* chain — do not collapse them.

use std::collections::HashSet;
use std::fs::{self, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use serde_json::{json, Map, Value};
use sha2::{Digest, Sha256};

use crate::command::ActuationCommand;
use crate::error::{PlantError, PlantResult};

const GENESIS: &str = "0000000000000000000000000000000000000000000000000000000000000000";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LedgerEvent {
    pub kind: String,
    pub body: Value,
}

#[derive(Debug, Clone)]
pub struct ContinuityState {
    pub estop: bool,
    pub estop_reason: Option<String>,
    pub last_identity: Option<Value>,
    pub identity_refuse_n: u32,
    pub last_identity_violation: Option<String>,
}

impl Default for ContinuityState {
    fn default() -> Self {
        Self {
            estop: false,
            estop_reason: None,
            last_identity: None,
            identity_refuse_n: 0,
            last_identity_violation: None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct CommandLedger {
    seen_ids: HashSet<String>,
    last_sequence: i64,
    journal_path: Option<PathBuf>,
    fail_closed: bool,
    events: Vec<Value>,
    chain_hash: String,
    unreadable: bool,
}

impl Default for CommandLedger {
    fn default() -> Self {
        Self::new()
    }
}

impl CommandLedger {
    pub fn new() -> Self {
        Self {
            seen_ids: HashSet::new(),
            last_sequence: 0,
            journal_path: None,
            fail_closed: false,
            events: Vec::new(),
            chain_hash: GENESIS.into(),
            unreadable: false,
        }
    }

    pub fn with_journal(path: impl AsRef<Path>, fail_closed: bool) -> PlantResult<Self> {
        let mut ledger = Self::new();
        ledger.journal_path = Some(path.as_ref().to_path_buf());
        ledger.fail_closed = fail_closed;
        ledger.load_journal()?;
        Ok(ledger)
    }

    pub fn is_unreadable(&self) -> bool {
        self.unreadable
    }

    pub fn last_sequence(&self) -> i64 {
        self.last_sequence
    }

    pub fn events(&self) -> &[Value] {
        &self.events
    }

    pub fn chain_hash(&self) -> &str {
        &self.chain_hash
    }

    fn mark_unreadable(&mut self, reason: &str) -> PlantResult<()> {
        self.unreadable = true;
        if self.fail_closed {
            return Err(PlantError::JournalUnreadable(reason.into()));
        }
        Ok(())
    }

    fn load_journal(&mut self) -> PlantResult<()> {
        let Some(path) = self.journal_path.clone() else {
            return Ok(());
        };
        if path.exists() && !path.is_file() {
            return self.mark_unreadable("journal_not_a_file");
        }
        if !path.is_file() {
            return Ok(());
        }
        let file = match fs::File::open(&path) {
            Ok(f) => f,
            Err(e) => return self.mark_unreadable(&format!("journal_unreadable:{e}")),
        };
        let reader = BufReader::new(file);
        let mut last_chain = GENESIS.to_string();
        let mut last_seq = 0_i64;
        let mut seen = HashSet::new();
        let mut loaded = Vec::new();
        for line in reader.lines() {
            let line = match line {
                Ok(l) => l,
                Err(e) => return self.mark_unreadable(&format!("journal_unreadable:{e}")),
            };
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            let rec: Value = match serde_json::from_str(line) {
                Ok(v) => v,
                Err(_) => return self.mark_unreadable("journal_corrupt"),
            };
            if !rec.is_object() {
                return self.mark_unreadable("journal_corrupt");
            }
            loaded.push(rec.clone());
            let kind = rec.get("kind").and_then(Value::as_str).unwrap_or("consume");
            if kind == "consume" {
                if let Some(cid) = rec.get("command_id").and_then(Value::as_str) {
                    if !cid.is_empty() {
                        seen.insert(cid.to_string());
                    }
                }
                if let Some(seq) = rec.get("sequence").and_then(Value::as_i64) {
                    if seq > last_seq {
                        last_seq = seq;
                    }
                }
            }
            if let Some(ch) = rec.get("chain_hash").and_then(Value::as_str) {
                last_chain = ch.to_string();
            }
        }
        self.events = loaded;
        self.seen_ids = seen;
        self.last_sequence = last_seq;
        self.chain_hash = last_chain;
        Ok(())
    }

    fn write_record(&mut self, mut body: Map<String, Value>) -> PlantResult<Value> {
        if self.unreadable && self.fail_closed {
            return Err(PlantError::JournalUnreadable("journal_unreadable".into()));
        }
        body.entry("t_s")
            .or_insert(json!(realityos_kernel::unix_now_s()));
        body.insert("prev_hash".into(), json!(self.chain_hash.clone()));
        let rec_for_hash = Value::Object(body.clone());
        let raw = canonical_json(&rec_for_hash);
        let inner = hex::encode(Sha256::digest(raw.as_bytes()));
        let chain = hex::encode(Sha256::digest(
            format!("{}{}", self.chain_hash, inner).as_bytes(),
        ));
        body.insert("chain_hash".into(), json!(chain.clone()));
        let rec = Value::Object(body);
        if let Some(path) = &self.journal_path {
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent)
                    .map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
            }
            let mut fh = OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
                .map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
            writeln!(fh, "{rec}").map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
        }
        self.chain_hash = chain;
        self.events.push(rec.clone());
        Ok(rec)
    }

    pub fn append_event(&mut self, kind: &str, payload: Map<String, Value>) -> PlantResult<Value> {
        let k = kind.trim();
        if k.is_empty() || k == "consume" {
            return Err(PlantError::refused("append_event_kind_must_not_be_consume"));
        }
        let mut body = payload;
        body.insert("kind".into(), json!(k));
        self.write_record(body)
    }

    pub fn events_of(&self, kind: &str) -> Vec<&Value> {
        self.events
            .iter()
            .filter(|e| e.get("kind").and_then(Value::as_str).unwrap_or("consume") == kind)
            .collect()
    }

    pub fn check(
        &self,
        command: &dyn ActuationCommand,
        now_s: f64,
        expected_release_hash: Option<&str>,
        expected_calibration_ids: &[String],
        expected_sensor_packet_hash: Option<&str>,
        require_sensor_packet_hash: bool,
        require_monotonic_sequence: bool,
    ) -> Vec<String> {
        let mut v = Vec::new();
        let cid = command.command_id();
        if cid.is_empty() {
            v.push("missing command_id".into());
        } else if self.seen_ids.contains(cid) {
            v.push("replayed command_id".into());
        }
        if now_s > command.expires_at_s() {
            v.push("command expired".into());
        }
        if let Some(exp) = expected_release_hash {
            if command.release_hash() != exp {
                v.push("release_hash mismatch".into());
            }
        }
        if !expected_calibration_ids.is_empty() {
            let cals = command.calibration_ids();
            for want in expected_calibration_ids {
                if !want.is_empty() && !cals.iter().any(|c| c == want) {
                    v.push("calibration_ids mismatch".into());
                    break;
                }
            }
        }
        if require_sensor_packet_hash {
            let got = command.sensor_packet_hash();
            if got.is_empty() {
                v.push("command missing sensor_packet_hash".into());
            }
            match expected_sensor_packet_hash {
                None => v.push("missing expected_sensor_packet_hash".into()),
                Some(exp) if got != exp => v.push("sensor_packet_hash mismatch".into()),
                Some(_) => {}
            }
        }
        if require_monotonic_sequence && command.sequence() <= self.last_sequence {
            v.push("sequence not monotonic".into());
        }
        v
    }

    pub fn consume(&mut self, command: &dyn ActuationCommand) -> PlantResult<Value> {
        let cid = command.command_id().to_string();
        if !cid.is_empty() {
            self.seen_ids.insert(cid.clone());
        }
        if command.sequence() > self.last_sequence {
            self.last_sequence = command.sequence();
        }
        let mut body = Map::new();
        body.insert("kind".into(), json!("consume"));
        body.insert("command_id".into(), json!(cid));
        body.insert("sequence".into(), json!(command.sequence()));
        body.insert("release_hash".into(), json!(command.release_hash()));
        body.insert("payload_hash".into(), json!(command.payload_hash()));
        self.write_record(body)
    }

    pub fn continuity_state(&self, serial: &str) -> ContinuityState {
        let mut state = ContinuityState::default();
        let identity_markers = [
            "release_hash",
            "calibration",
            "as_built",
            "sensor_packet",
            "replayed",
            "incomplete_runtime_identity",
            "identity_continuity",
        ];
        for ev in &self.events {
            let ident = ev.get("identity").cloned().filter(Value::is_object);
            let ev_serial = ident
                .as_ref()
                .and_then(|i| i.get("serial_or_as_built"))
                .and_then(Value::as_str)
                .or_else(|| ev.get("serial_or_as_built").and_then(Value::as_str))
                .unwrap_or("");
            if !serial.is_empty() && !ev_serial.is_empty() && ev_serial != serial {
                continue;
            }
            if let Some(id) = ident {
                state.last_identity = Some(id);
            }
            let kind = ev.get("kind").and_then(Value::as_str).unwrap_or("consume");
            let event = ev.get("event").and_then(Value::as_str).unwrap_or("");
            if kind != "governor_event" {
                continue;
            }
            match event {
                "estop" => {
                    state.estop = true;
                    let reason = ev
                        .get("violations")
                        .and_then(Value::as_array)
                        .and_then(|a| a.first())
                        .and_then(Value::as_str)
                        .unwrap_or("estop");
                    state.estop_reason = Some(reason.into());
                }
                "recovery_cleared" => {
                    state.estop = false;
                    state.estop_reason = None;
                }
                "driver_write_refused" => {
                    let viols: Vec<String> = ev
                        .get("violations")
                        .and_then(Value::as_array)
                        .map(|a| {
                            a.iter()
                                .filter_map(Value::as_str)
                                .map(str::to_string)
                                .collect()
                        })
                        .unwrap_or_default();
                    let blob = viols.join(" ");
                    if identity_markers.iter().any(|m| blob.contains(m)) {
                        state.identity_refuse_n += 1;
                        state.last_identity_violation = identity_markers
                            .iter()
                            .find(|m| blob.contains(*m))
                            .map(|s| (*s).to_string());
                    }
                }
                "driver_write" => {
                    if ev.get("ok").and_then(Value::as_bool) == Some(true) {
                        state.identity_refuse_n = 0;
                    }
                }
                _ => {}
            }
        }
        state
    }
}

fn canonical_json(v: &Value) -> String {
    serde_json::to_string(v).unwrap_or_else(|_| "{}".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::ActuationCommand;
    use realityos_kernel::DecisionStatus;

    struct Dummy {
        id: String,
        seq: i64,
        exp: f64,
        rh: String,
        cals: Vec<String>,
        sph: String,
    }

    impl ActuationCommand for Dummy {
        fn command_id(&self) -> &str {
            &self.id
        }
        fn sequence(&self) -> i64 {
            self.seq
        }
        fn issued_at_s(&self) -> f64 {
            0.0
        }
        fn expires_at_s(&self) -> f64 {
            self.exp
        }
        fn allowed_action(&self) -> &[f64] {
            &[]
        }
        fn issuer_allowed_action(&self) -> &[f64] {
            &[]
        }
        fn certificate_status(&self) -> DecisionStatus {
            DecisionStatus::Allow
        }
        fn issuer_certificate_status(&self) -> &str {
            "allow"
        }
        fn issuer_physical_reason(&self) -> &str {
            ""
        }
        fn release_hash(&self) -> &str {
            &self.rh
        }
        fn as_built_hash(&self) -> &str {
            ""
        }
        fn calibration_ids(&self) -> &[String] {
            &self.cals
        }
        fn acknowledged(&self) -> bool {
            true
        }
        fn sensor_snapshot_id(&self) -> &str {
            ""
        }
        fn sensor_packet_hash(&self) -> &str {
            &self.sph
        }
        fn belief_snapshot_id(&self) -> &str {
            ""
        }
        fn payload_hash(&self) -> &str {
            ""
        }
        fn signature(&self) -> &str {
            ""
        }
        fn signer(&self) -> &str {
            ""
        }
        fn signing_scheme(&self) -> &str {
            ""
        }
        fn actuator_ids(&self) -> &[String] {
            &[]
        }
    }

    #[test]
    fn replay_fails_closed() {
        let mut l = CommandLedger::new();
        let cmd = Dummy {
            id: "c1".into(),
            seq: 1,
            exp: 1e12,
            rh: "r".into(),
            cals: vec!["cal".into()],
            sph: String::new(),
        };
        assert!(l
            .check(&cmd, 0.0, Some("r"), &["cal".into()], None, false, false)
            .is_empty());
        l.consume(&cmd).unwrap();
        let v = l.check(&cmd, 0.0, Some("r"), &["cal".into()], None, false, false);
        assert!(v.iter().any(|s| s.contains("replayed")));
    }

    #[test]
    fn governor_event_does_not_consume() {
        let mut l = CommandLedger::new();
        let mut body = Map::new();
        body.insert("event".into(), json!("estop"));
        l.append_event("governor_event", body).unwrap();
        assert!(l.seen_ids.is_empty());
        assert_eq!(l.events_of("governor_event").len(), 1);
    }
}
