//! Lowest trustworthy egress counters. Not plant.write_count.
//!
//! `record_attempt` is a command-egress attempt, not a physical device write.
//! Certified serial TX is counted only after write_all+flush of a command frame.
//! Setup/sensor traffic must not increment certified-command counters.

use std::io::{self, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::config::{ACKS_FILE, EGRESS_ATTEMPTS_FILE, EGRESS_LOG, SERIAL_TX_FILE, WRITES_FILE};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EgressSnapshot {
    pub command_egress_attempts: u64,
    pub serial_tx_completed: u64,
    pub device_acknowledgements: u64,
    pub last_goal_position: Option<i32>,
    pub last_present_position: Option<i32>,
    pub last_instruction: String,
}

#[derive(Debug)]
pub struct EgressLog {
    root: PathBuf,
}

impl EgressLog {
    pub fn open(root: impl Into<PathBuf>) -> io::Result<Self> {
        let root = root.into();
        std::fs::create_dir_all(&root)?;
        for name in [WRITES_FILE, EGRESS_ATTEMPTS_FILE, SERIAL_TX_FILE, ACKS_FILE] {
            if !root.join(name).exists() {
                std::fs::write(root.join(name), b"0")?;
            }
        }
        Ok(Self { root })
    }

    pub fn snapshot(&self) -> EgressSnapshot {
        EgressSnapshot {
            command_egress_attempts: self.command_egress_attempts(),
            serial_tx_completed: self.serial_tx_completed(),
            device_acknowledgements: read_u64(&self.root.join(ACKS_FILE)),
            last_goal_position: None,
            last_present_position: None,
            last_instruction: String::new(),
        }
    }

    pub fn command_egress_attempts(&self) -> u64 {
        read_u64(&self.root.join(EGRESS_ATTEMPTS_FILE)).max(read_u64(&self.root.join(WRITES_FILE)))
    }

    pub fn serial_tx_completed(&self) -> u64 {
        read_u64(&self.root.join(SERIAL_TX_FILE))
    }

    /// Command-path intent only. Not a physical device write.
    pub fn record_attempt(
        &self,
        instruction: &str,
        addr: u16,
        bytes: usize,
        goal: Option<i32>,
    ) -> io::Result<u64> {
        let n = self.command_egress_attempts().saturating_add(1);
        std::fs::write(self.root.join(WRITES_FILE), n.to_string())?;
        std::fs::write(self.root.join(EGRESS_ATTEMPTS_FILE), n.to_string())?;
        let rec = serde_json::json!({
            "kind": "command_egress_attempt",
            "n": n,
            "instruction": instruction,
            "addr": addr,
            "bytes": bytes,
            "goal": goal,
        });
        append_jsonl(&self.root.join(EGRESS_LOG), &rec)?;
        Ok(n)
    }

    /// After the entire certified command frame passed write_all+flush.
    pub fn record_serial_tx(&self, instruction: &str, addr: u16) -> io::Result<u64> {
        let n = self.serial_tx_completed().saturating_add(1);
        std::fs::write(self.root.join(SERIAL_TX_FILE), n.to_string())?;
        let rec = serde_json::json!({
            "kind": "serial_tx_completed",
            "n": n,
            "instruction": instruction,
            "addr": addr,
        });
        append_jsonl(&self.root.join(EGRESS_LOG), &rec)?;
        Ok(n)
    }

    pub fn record_setup_xfer(&self, instruction: &str, addr: u16) -> io::Result<()> {
        let rec = serde_json::json!({
            "kind": "setup_or_sensor",
            "instruction": instruction,
            "addr": addr,
        });
        append_jsonl(&self.root.join(EGRESS_LOG), &rec)
    }

    pub fn record_ack(&self, ok: bool, error: u8, present: Option<i32>) -> io::Result<u64> {
        let n = if ok {
            let n = read_u64(&self.root.join(ACKS_FILE)).saturating_add(1);
            std::fs::write(self.root.join(ACKS_FILE), n.to_string())?;
            n
        } else {
            read_u64(&self.root.join(ACKS_FILE))
        };
        let rec = serde_json::json!({
            "kind": "ack",
            "ok": ok,
            "error": error,
            "present": present,
        });
        append_jsonl(&self.root.join(EGRESS_LOG), &rec)?;
        Ok(n)
    }
}

pub fn recorded_writes(root: impl AsRef<Path>) -> u64 {
    recorded_egress_attempts(root)
}

pub fn recorded_egress_attempts(root: impl AsRef<Path>) -> u64 {
    let root = root.as_ref();
    read_u64(&root.join(EGRESS_ATTEMPTS_FILE)).max(read_u64(&root.join(WRITES_FILE)))
}

pub fn recorded_serial_tx(root: impl AsRef<Path>) -> u64 {
    read_u64(&root.as_ref().join(SERIAL_TX_FILE))
}

pub fn recorded_acks(root: impl AsRef<Path>) -> u64 {
    read_u64(&root.as_ref().join(ACKS_FILE))
}

fn read_u64(path: &Path) -> u64 {
    std::fs::read_to_string(path)
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(0)
}

fn append_jsonl(path: &Path, v: &serde_json::Value) -> io::Result<()> {
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)?;
    writeln!(f, "{v}")?;
    f.flush()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn attempt_is_not_serial_tx() {
        let dir = std::env::temp_dir().join(format!(
            "realityos-egress-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let log = EgressLog::open(&dir).unwrap();
        log.record_attempt("goal_position", 116, 4, Some(2048))
            .unwrap();
        assert_eq!(log.command_egress_attempts(), 1);
        assert_eq!(log.serial_tx_completed(), 0);
        log.record_serial_tx("goal_position", 116).unwrap();
        assert_eq!(log.serial_tx_completed(), 1);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
