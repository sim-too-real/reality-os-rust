use std::fs::{self, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

use thiserror::Error;

use crate::event::{DebugEvent, EventFilter, EventKind};
use realityos_kernel::Layer;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum DataError {
    #[error("event store unreadable: {0}")]
    Unreadable(String),
}

pub type DataResult<T> = Result<T, DataError>;

#[derive(Debug, Clone)]
pub struct EventStore {
    events: Vec<DebugEvent>,
    path: Option<PathBuf>,
    next_seq: u64,
}

impl Default for EventStore {
    fn default() -> Self {
        Self::new()
    }
}

impl EventStore {
    pub fn new() -> Self {
        Self {
            events: Vec::new(),
            path: None,
            next_seq: 1,
        }
    }

    pub fn with_path(path: impl AsRef<Path>) -> DataResult<Self> {
        let mut s = Self::new();
        s.path = Some(path.as_ref().to_path_buf());
        s.load()?;
        Ok(s)
    }

    pub fn len(&self) -> usize {
        self.events.len()
    }

    pub fn is_empty(&self) -> bool {
        self.events.is_empty()
    }

    pub fn events(&self) -> &[DebugEvent] {
        &self.events
    }

    pub fn append(&mut self, mut event: DebugEvent) -> DataResult<&DebugEvent> {
        event.seq = self.next_seq;
        event.metal = false;
        self.next_seq += 1;
        if let Some(path) = &self.path {
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent).map_err(|e| DataError::Unreadable(e.to_string()))?;
            }
            let mut fh = OpenOptions::new()
                .create(true)
                .append(true)
                .open(path)
                .map_err(|e| DataError::Unreadable(e.to_string()))?;
            let line = serde_json::to_string(&event).unwrap_or_else(|_| "{}".into());
            writeln!(fh, "{line}").map_err(|e| DataError::Unreadable(e.to_string()))?;
        }
        self.events.push(event);
        Ok(self.events.last().expect("just pushed"))
    }

    pub fn query(&self, filter: &EventFilter) -> Vec<&DebugEvent> {
        self.events.iter().filter(|e| filter.matches(e)).collect()
    }

    pub fn last_of(&self, kind: EventKind) -> Option<&DebugEvent> {
        self.events.iter().rev().find(|e| e.kind == kind)
    }

    pub fn refuses(&self) -> Vec<&DebugEvent> {
        self.events.iter().filter(|e| !e.ok).collect()
    }

    pub fn layers_seen(&self) -> Vec<Layer> {
        let mut out = Vec::new();
        for e in &self.events {
            if !out.contains(&e.layer) {
                out.push(e.layer);
            }
        }
        out
    }

    fn load(&mut self) -> DataResult<()> {
        let Some(path) = &self.path else {
            return Ok(());
        };
        if !path.is_file() {
            return Ok(());
        }
        let file = fs::File::open(path).map_err(|e| DataError::Unreadable(e.to_string()))?;
        for line in BufReader::new(file).lines() {
            let line = line.map_err(|e| DataError::Unreadable(e.to_string()))?;
            if line.trim().is_empty() {
                continue;
            }
            let ev: DebugEvent = serde_json::from_str(&line)
                .map_err(|e| DataError::Unreadable(format!("corrupt:{e}")))?;
            self.next_seq = self.next_seq.max(ev.seq + 1);
            self.events.push(ev);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::event::DebugEvent;
    use realityos_kernel::Layer;

    #[test]
    fn query_by_kind_and_refuse() {
        let mut s = EventStore::new();
        s.append(DebugEvent::new(
            Layer::Governor,
            EventKind::Write,
            true,
            1.0,
            "a",
        ))
        .unwrap();
        s.append(DebugEvent::new(
            Layer::Governor,
            EventKind::Refuse,
            false,
            2.0,
            "b",
        ))
        .unwrap();
        assert_eq!(s.refuses().len(), 1);
        let f = EventFilter {
            kind: Some(EventKind::Write),
            ..EventFilter::default()
        };
        assert_eq!(s.query(&f).len(), 1);
        assert_eq!(s.len(), 2);
    }
}
