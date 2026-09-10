//! Driver-owned replay / monotonic journal. Same hash chain as Python CommandLedger.
//! Decision attestation is a *second* chain — do not collapse them.

use std::cell::Cell;
use std::collections::HashSet;
use std::fs::{self, OpenOptions, Permissions};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};
use std::time::Duration;

#[cfg(unix)]
use std::os::unix::fs::PermissionsExt;

use serde::{Deserialize, Serialize};
use serde_json::{json, Map, Value};
use sha2::{Digest, Sha256};

use crate::command::ActuationCommand;
use crate::consume::ConsumePhase;
use crate::error::{PlantError, PlantResult};

const GENESIS: &str = "0000000000000000000000000000000000000000000000000000000000000000";
const SEAL_SCHEMA: &str = "realityos.ledger_seal/1";

thread_local! {
    static TEST_JOURNAL_WRITE_DELAY_MS: Cell<u64> = const { Cell::new(0) };
}

/// Test-only: delay each durable journal write on this thread.
/// Used to prove successful watchdog pets do not fsync.
pub fn set_test_journal_write_delay_ms(ms: u64) {
    TEST_JOURNAL_WRITE_DELAY_MS.with(|c| c.set(ms.min(2000)));
}

fn restrict_owner_rw(path: &Path) {
    #[cfg(unix)]
    {
        let _ = fs::set_permissions(path, Permissions::from_mode(0o600));
    }
    #[cfg(not(unix))]
    {
        let _ = path;
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct AuthoritySeal {
    schema: String,
    chain_hash: String,
    last_sequence: i64,
    event_count: usize,
    seen_count: usize,
}

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
    seal_path: Option<PathBuf>,
    persist_seal: bool,
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
            seal_path: None,
            persist_seal: false,
        }
    }

    pub fn with_journal(path: impl AsRef<Path>, fail_closed: bool) -> PlantResult<Self> {
        let mut ledger = Self::new();
        ledger.journal_path = Some(path.as_ref().to_path_buf());
        ledger.fail_closed = fail_closed;
        ledger.load_journal()?;
        Ok(ledger)
    }

    /// ONLINE journal. Hash-chain is tamper-*evident*, not authenticated.
    /// Same-filesystem deletion of both journal and seal looks like first boot;
    /// `first_online=false` refuses that case.
    pub fn with_online_journal(path: impl AsRef<Path>, first_online: bool) -> PlantResult<Self> {
        let path = path.as_ref().to_path_buf();
        let seal_path = seal_path_for(&path);
        let journal_exists = path.is_file();
        let seal_exists = seal_path.is_file();
        if first_online {
            if journal_exists || seal_exists {
                return Err(PlantError::refused(
                    "first_online_but_journal_or_seal_exists",
                ));
            }
        } else if !journal_exists && !seal_exists {
            return Err(PlantError::JournalMissing);
        }
        if seal_exists && !journal_exists {
            return Err(PlantError::JournalDeleted);
        }
        if journal_exists && !seal_exists {
            return Err(PlantError::JournalSealMissing);
        }
        let mut ledger = Self::new();
        ledger.journal_path = Some(path);
        ledger.seal_path = Some(seal_path);
        ledger.fail_closed = true;
        ledger.persist_seal = true;
        ledger.load_journal()?;
        if ledger.unreadable {
            return Err(PlantError::JournalUnreadable("journal_unreadable".into()));
        }
        if seal_exists {
            ledger.check_seal()?;
        }
        ledger.persist_seal()?;
        Ok(ledger)
    }

    pub fn command_phase(&self, command_id: &str) -> ConsumePhase {
        let mut phase = ConsumePhase::Unseen;
        for ev in &self.events {
            let cid = ev.get("command_id").and_then(Value::as_str).unwrap_or("");
            if cid != command_id {
                continue;
            }
            match ev.get("kind").and_then(Value::as_str).unwrap_or("") {
                "prepare" => phase = ConsumePhase::Prepared,
                "consume" => phase = ConsumePhase::Consumed,
                "unknown_outcome" => phase = ConsumePhase::Unknown,
                _ => {}
            }
        }
        phase
    }

    fn check_seal(&self) -> PlantResult<()> {
        let Some(path) = &self.seal_path else {
            return Ok(());
        };
        let raw = fs::read_to_string(path)
            .map_err(|e| PlantError::JournalUnreadable(format!("seal_unreadable:{e}")))?;
        let seal: AuthoritySeal = serde_json::from_str(&raw)
            .map_err(|_| PlantError::JournalUnreadable("seal_corrupt".into()))?;
        if seal.schema != SEAL_SCHEMA {
            return Err(PlantError::JournalUnreadable("seal_schema".into()));
        }
        if self.events.len() < seal.event_count {
            return Err(PlantError::JournalRollback);
        }
        if seal.event_count == 0 {
            return Ok(());
        }
        let prefix = &self.events[seal.event_count - 1];
        let prefix_hash = prefix
            .get("chain_hash")
            .and_then(Value::as_str)
            .unwrap_or("");
        if prefix_hash != seal.chain_hash {
            return Err(PlantError::JournalReplaced);
        }
        Ok(())
    }

    fn persist_seal(&self) -> PlantResult<()> {
        if !self.persist_seal {
            return Ok(());
        }
        let Some(path) = &self.seal_path else {
            return Ok(());
        };
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
        }
        let seal = AuthoritySeal {
            schema: SEAL_SCHEMA.into(),
            chain_hash: self.chain_hash.clone(),
            last_sequence: self.last_sequence,
            event_count: self.events.len(),
            seen_count: self.seen_ids.len(),
        };
        let raw = serde_json::to_string(&seal)
            .map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
        let tmp = path.with_extension("seal.tmp");
        fs::write(&tmp, raw).map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
        let fh = OpenOptions::new()
            .write(true)
            .open(&tmp)
            .map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
        fh.sync_all()
            .map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
        fs::rename(&tmp, path).map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
        restrict_owner_rw(path);
        Ok(())
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
            self.chain_hash = GENESIS.into();
            return Ok(());
        }
        let file = match fs::File::open(&path) {
            Ok(f) => f,
            Err(e) => return self.mark_unreadable(&format!("journal_unreadable:{e}")),
        };
        let reader = BufReader::new(file);
        let mut expected_prev = GENESIS.to_string();
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
            let Some(obj) = rec.as_object() else {
                return self.mark_unreadable("journal_corrupt");
            };
            let stored_prev = obj.get("prev_hash").and_then(Value::as_str).unwrap_or("");
            if stored_prev != expected_prev {
                return self.mark_unreadable("journal_prev_hash_mismatch");
            }
            let mut body = obj.clone();
            body.remove("chain_hash");
            let raw = canonical_json(&Value::Object(body.clone()));
            let inner = hex::encode(Sha256::digest(raw.as_bytes()));
            let recomputed =
                hex::encode(Sha256::digest(format!("{expected_prev}{inner}").as_bytes()));
            let stored_chain = obj.get("chain_hash").and_then(Value::as_str).unwrap_or("");
            if stored_chain != recomputed {
                return self.mark_unreadable("journal_chain_hash_mismatch");
            }
            loaded.push(rec.clone());
            let kind = rec.get("kind").and_then(Value::as_str).unwrap_or("consume");
            if matches!(kind, "consume" | "prepare" | "unknown_outcome") {
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
            expected_prev = recomputed;
        }
        self.events = loaded;
        self.seen_ids = seen;
        self.last_sequence = last_seq;
        self.chain_hash = expected_prev;
        Ok(())
    }

    fn write_record(&mut self, mut body: Map<String, Value>) -> PlantResult<Value> {
        if self.unreadable && self.fail_closed {
            return Err(PlantError::JournalUnreadable("journal_unreadable".into()));
        }
        let delay_ms = TEST_JOURNAL_WRITE_DELAY_MS.with(Cell::get);
        if delay_ms > 0 {
            std::thread::sleep(Duration::from_millis(delay_ms));
        }
        // Integer millis — f64 unix seconds do not JSON-round-trip, which
        // breaks the hash chain under fail_closed ONLINE journals.
        body.entry("t_ms").or_insert(json!(unix_now_ms()));
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
            fh.sync_all()
                .map_err(|e| PlantError::JournalUnreadable(e.to_string()))?;
            restrict_owner_rw(path);
        }
        self.chain_hash = chain;
        self.events.push(rec.clone());
        self.persist_seal()?;
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

    pub fn prepare(&mut self, command: &dyn ActuationCommand) -> PlantResult<Value> {
        let cid = command.command_id().to_string();
        if cid.is_empty() {
            return Err(PlantError::refused("missing command_id"));
        }
        if self.seen_ids.contains(&cid) {
            return Err(PlantError::refused("replayed command_id"));
        }
        self.seen_ids.insert(cid.clone());
        if command.sequence() > self.last_sequence {
            self.last_sequence = command.sequence();
        }
        let mut body = Map::new();
        body.insert("kind".into(), json!("prepare"));
        body.insert("command_id".into(), json!(cid));
        body.insert("sequence".into(), json!(command.sequence()));
        body.insert("release_hash".into(), json!(command.release_hash()));
        body.insert("payload_hash".into(), json!(command.payload_hash()));
        self.write_record(body)
    }

    pub fn ack(&mut self, command: &dyn ActuationCommand) -> PlantResult<Value> {
        let cid = command.command_id();
        if !self.seen_ids.contains(cid) {
            return Err(PlantError::refused("ack_without_prepare"));
        }
        let mut body = Map::new();
        body.insert("kind".into(), json!("consume"));
        body.insert("command_id".into(), json!(cid));
        body.insert("sequence".into(), json!(command.sequence()));
        body.insert("release_hash".into(), json!(command.release_hash()));
        body.insert("payload_hash".into(), json!(command.payload_hash()));
        self.write_record(body)
    }

    pub fn mark_unknown(&mut self, command: &dyn ActuationCommand) -> PlantResult<Value> {
        let cid = command.command_id();
        if !cid.is_empty() {
            self.seen_ids.insert(cid.to_string());
        }
        let mut body = Map::new();
        body.insert("kind".into(), json!("unknown_outcome"));
        body.insert("command_id".into(), json!(cid));
        body.insert("sequence".into(), json!(command.sequence()));
        self.write_record(body)
    }

    pub fn consume(&mut self, command: &dyn ActuationCommand) -> PlantResult<Value> {
        self.prepare(command)?;
        self.ack(command)
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

fn unix_now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn seal_path_for(journal: &Path) -> PathBuf {
    let mut name = journal.as_os_str().to_os_string();
    name.push(".authority-seal");
    PathBuf::from(name)
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

    fn dummy(id: &str, seq: i64) -> Dummy {
        Dummy {
            id: id.into(),
            seq,
            exp: 1e12,
            rh: "r".into(),
            cals: vec!["cal".into()],
            sph: String::new(),
        }
    }

    #[test]
    fn online_journal_missing_without_first_boot_refuses() {
        let dir =
            std::env::temp_dir().join(format!("realityos-ledger-missing-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("driver.jsonl");
        let err = CommandLedger::with_online_journal(&path, false).unwrap_err();
        assert_eq!(err, PlantError::JournalMissing);
    }

    #[test]
    fn online_journal_deletion_and_rollback_fail_closed() {
        let dir = std::env::temp_dir().join(format!(
            "realityos-ledger-seal-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("driver.jsonl");
        let mut l = CommandLedger::with_online_journal(&path, true).unwrap();
        l.consume(&dummy("c1", 1)).unwrap();
        drop(l);

        let seal = {
            let mut n = path.clone().into_os_string();
            n.push(".authority-seal");
            std::path::PathBuf::from(n)
        };
        assert!(seal.is_file());
        std::fs::remove_file(&path).unwrap();
        let err = CommandLedger::with_online_journal(&path, false).unwrap_err();
        assert_eq!(err, PlantError::JournalDeleted);

        let dir2 = std::env::temp_dir().join(format!(
            "realityos-ledger-rollback-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::create_dir_all(&dir2);
        let path2 = dir2.join("driver.jsonl");
        let mut l = CommandLedger::with_online_journal(&path2, true).unwrap();
        l.consume(&dummy("a", 1)).unwrap();
        l.consume(&dummy("b", 2)).unwrap();
        let full = std::fs::read_to_string(&path2).unwrap();
        drop(l);
        let first_line = full.lines().next().unwrap();
        std::fs::write(&path2, format!("{first_line}\n")).unwrap();
        let err = CommandLedger::with_online_journal(&path2, false).unwrap_err();
        assert_eq!(err, PlantError::JournalRollback);
    }

    #[test]
    fn online_journal_reloads_after_writes() {
        let dir = std::env::temp_dir().join(format!(
            "realityos-ledger-reload-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::create_dir_all(&dir);
        let path = dir.join("driver.jsonl");
        let mut l = CommandLedger::with_online_journal(&path, true).unwrap();
        l.consume(&dummy("r1", 1)).unwrap();
        drop(l);
        let l2 = CommandLedger::with_online_journal(&path, false).unwrap();
        assert_eq!(l2.command_phase("r1"), ConsumePhase::Consumed);
        assert!(!l2.command_phase("r1").may_attempt_write());
    }

    #[test]
    fn prepare_is_not_retryable() {
        let mut l = CommandLedger::new();
        let cmd = dummy("p1", 3);
        l.prepare(&cmd).unwrap();
        assert_eq!(l.command_phase("p1"), ConsumePhase::Prepared);
        assert!(!l.command_phase("p1").may_attempt_write());
        assert!(l.prepare(&cmd).is_err());
    }
}
