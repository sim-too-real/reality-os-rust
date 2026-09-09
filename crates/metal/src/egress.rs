//! Lowest trustworthy egress counters. Not plant.write_count.

use std::io::{self, Write};
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::config::{ACKS_FILE, EGRESS_LOG, WRITES_FILE};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct EgressSnapshot {
    pub attempted_bus_writes: u64,
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
        if !root.join(WRITES_FILE).exists() {
            std::fs::write(root.join(WRITES_FILE), b"0")?;
        }
        if !root.join(ACKS_FILE).exists() {
            std::fs::write(root.join(ACKS_FILE), b"0")?;
        }
        Ok(Self { root })
    }

    pub fn snapshot(&self) -> EgressSnapshot {
        EgressSnapshot {
            attempted_bus_writes: read_u64(&self.root.join(WRITES_FILE)),
            device_acknowledgements: read_u64(&self.root.join(ACKS_FILE)),
            last_goal_position: None,
            last_present_position: None,
            last_instruction: String::new(),
        }
    }

    pub fn record_attempt(
        &self,
        instruction: &str,
        addr: u16,
        bytes: usize,
        goal: Option<i32>,
    ) -> io::Result<u64> {
        let n = read_u64(&self.root.join(WRITES_FILE)).saturating_add(1);
        std::fs::write(self.root.join(WRITES_FILE), n.to_string())?;
        let rec = serde_json::json!({
            "kind": "attempt",
            "n": n,
            "instruction": instruction,
            "addr": addr,
            "bytes": bytes,
            "goal": goal,
        });
        append_jsonl(&self.root.join(EGRESS_LOG), &rec)?;
        Ok(n)
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
    read_u64(&root.as_ref().join(WRITES_FILE))
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
