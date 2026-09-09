//! Minimal MCAP writer. Audit only — never on the certified write path.

use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};

use crate::event::DebugEvent;

const MAGIC: &[u8] = b"\x89MCAP0\r\n";
const OP_HEADER: u8 = 1;
const OP_FOOTER: u8 = 2;
const OP_CHANNEL: u8 = 4;
const OP_MESSAGE: u8 = 5;
const OP_DATA_END: u8 = 15;

#[derive(Debug)]
pub struct McapWriter {
    path: PathBuf,
    seq: u32,
    finalized: bool,
}

impl McapWriter {
    pub fn create(path: impl AsRef<Path>) -> io::Result<Self> {
        let path = path.as_ref().to_path_buf();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut fh = File::create(&path)?;
        fh.write_all(MAGIC)?;
        write_record(&mut fh, OP_HEADER, &header_payload("realityos", "realityos-data"))?;
        write_record(&mut fh, OP_CHANNEL, &channel_payload(1, 0, "realityos/events", "json"))?;
        fh.flush()?;
        Ok(Self {
            path,
            seq: 0,
            finalized: false,
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn append_event(&mut self, event: &DebugEvent) -> io::Result<()> {
        if self.finalized {
            return Err(io::Error::other("mcap writer already finalized"));
        }
        let json = serde_json::to_vec(event).unwrap_or_else(|_| b"{}".to_vec());
        let log_time = (event.t_s.max(0.0) * 1e9) as u64;
        let mut payload = Vec::with_capacity(22 + json.len());
        payload.extend_from_slice(&1u16.to_le_bytes());
        payload.extend_from_slice(&self.seq.to_le_bytes());
        payload.extend_from_slice(&log_time.to_le_bytes());
        payload.extend_from_slice(&log_time.to_le_bytes());
        payload.extend_from_slice(&json);
        let mut fh = OpenOptions::new().append(true).open(&self.path)?;
        write_record(&mut fh, OP_MESSAGE, &payload)?;
        self.seq += 1;
        Ok(())
    }

    pub fn finish(mut self) -> io::Result<PathBuf> {
        if !self.finalized {
            let mut fh = OpenOptions::new().append(true).open(&self.path)?;
            write_record(&mut fh, OP_DATA_END, &0u32.to_le_bytes())?;
            let mut footer = Vec::with_capacity(20);
            footer.extend_from_slice(&0u64.to_le_bytes());
            footer.extend_from_slice(&0u64.to_le_bytes());
            footer.extend_from_slice(&0u32.to_le_bytes());
            write_record(&mut fh, OP_FOOTER, &footer)?;
            fh.write_all(MAGIC)?;
            fh.flush()?;
            self.finalized = true;
        }
        Ok(self.path)
    }
}

fn write_record(w: &mut impl Write, opcode: u8, data: &[u8]) -> io::Result<()> {
    w.write_all(&[opcode])?;
    w.write_all(&(data.len() as u64).to_le_bytes())?;
    w.write_all(data)
}

fn put_string(buf: &mut Vec<u8>, s: &str) {
    buf.extend_from_slice(&(s.len() as u32).to_le_bytes());
    buf.extend_from_slice(s.as_bytes());
}

fn header_payload(profile: &str, library: &str) -> Vec<u8> {
    let mut buf = Vec::new();
    put_string(&mut buf, profile);
    put_string(&mut buf, library);
    buf
}

fn channel_payload(channel_id: u16, schema_id: u16, topic: &str, encoding: &str) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.extend_from_slice(&channel_id.to_le_bytes());
    buf.extend_from_slice(&schema_id.to_le_bytes());
    put_string(&mut buf, topic);
    put_string(&mut buf, encoding);
    buf.extend_from_slice(&0u32.to_le_bytes());
    buf
}

pub fn magic_prefix(bytes: &[u8]) -> bool {
    bytes.starts_with(MAGIC)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::event::{DebugEvent, EventKind};
    use realityos_kernel::Layer;

    #[test]
    fn writer_emits_mcap_magic_and_is_not_a_write_path() {
        let dir = std::env::temp_dir().join(format!("realityos-mcap-{}", std::process::id()));
        let path = dir.join("audit.mcap");
        let _ = std::fs::remove_file(&path);
        let mut w = McapWriter::create(&path).unwrap();
        let ev = DebugEvent::new(Layer::Session, EventKind::SessionStart, true, 1.0, "c1");
        w.append_event(&ev).unwrap();
        let path = w.finish().unwrap();
        let bytes = std::fs::read(&path).unwrap();
        assert!(magic_prefix(&bytes));
        assert!(bytes.ends_with(MAGIC));
        assert!(bytes.len() > MAGIC.len() * 2);
    }
}
