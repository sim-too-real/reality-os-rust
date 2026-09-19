//! Device-side serial peer: bytes in, complete instruction, device process,
//! status bytes out, optional transport faults, TX/RX transcript.
//!
//! Emulates the XL330, not a second copy of `Xl330Driver`.

use std::sync::{Arc, Mutex};

use realityos_metal::protocol::{decode_instruction, encode_reboot, encode_status, HEADER};

use crate::device::VirtualXl330;
use crate::faults::{corrupt_crc_bytes, FaultKind, FaultSchedule};

const GARBAGE_SUFFIX: &[u8] = &[0xAA, 0xBB];

#[derive(Clone)]
pub struct SerialTranscript {
    pub tx: Vec<Vec<u8>>,
    pub rx: Vec<Vec<u8>>,
}

pub type SharedXl330 = Arc<Mutex<VirtualXl330>>;

pub struct VirtualSerialPeer {
    device: SharedXl330,
    acc: Vec<u8>,
    pending_out: Vec<u8>,
    transport: FaultSchedule,
    connected: bool,
    silent: bool,
    echo: bool,
    last_delay_ms: u64,
    split_first: Option<usize>,
    packets: u32,
    tx: Vec<Vec<u8>>,
    rx: Vec<Vec<u8>>,
}

impl VirtualSerialPeer {
    pub fn new(device: SharedXl330) -> Self {
        Self {
            device,
            acc: Vec::new(),
            pending_out: Vec::new(),
            transport: FaultSchedule::empty(),
            connected: true,
            silent: false,
            echo: false,
            last_delay_ms: 0,
            split_first: None,
            packets: 0,
            tx: Vec::new(),
            rx: Vec::new(),
        }
    }

    pub fn with_echo(mut self, echo: bool) -> Self {
        self.echo = echo;
        self
    }

    pub fn device(&self) -> SharedXl330 {
        self.device.clone()
    }

    pub fn set_transport_schedule(&mut self, s: FaultSchedule) {
        self.transport = s;
    }

    pub fn last_delay_ms(&self) -> u64 {
        self.last_delay_ms
    }

    pub fn packet_count(&self) -> u32 {
        self.packets
    }

    pub fn is_connected(&self) -> bool {
        self.connected
    }

    pub fn transcript(&self) -> SerialTranscript {
        SerialTranscript {
            tx: self.tx.clone(),
            rx: self.rx.clone(),
        }
    }

    /// Push host bytes. Complete instructions are processed even while
    /// disconnected so `FaultKind::Reconnect` can restore the byte path.
    pub fn push(&mut self, bytes: &[u8]) {
        self.tx.push(bytes.to_vec());
        self.acc.extend_from_slice(bytes);
        while let Some(inst) = take_complete_instruction(&mut self.acc) {
            let out = self.handle_instruction(&inst);
            self.pending_out.extend_from_slice(&out);
        }
        if self.acc.len() > 512 {
            self.acc.clear();
        }
    }

    /// Read up to `max` queued status bytes (honours split-across-reads).
    pub fn read(&mut self, max: usize) -> Vec<u8> {
        if !self.connected {
            return Vec::new();
        }
        if self.pending_out.is_empty() {
            return Vec::new();
        }
        let n = if let Some(first) = self.split_first.take() {
            first.min(max).min(self.pending_out.len())
        } else {
            max.min(self.pending_out.len())
        };
        self.pending_out.drain(..n).collect()
    }

    /// One complete instruction → (optional echo) + status. Used by the
    /// in-process production-codec path (no PTY).
    pub fn exchange(&mut self, instruction: &[u8]) -> Vec<u8> {
        self.push(instruction);
        let mut out = Vec::new();
        loop {
            let chunk = self.read(4096);
            if chunk.is_empty() {
                break;
            }
            out.extend_from_slice(&chunk);
        }
        out
    }

    fn handle_instruction(&mut self, inst: &[u8]) -> Vec<u8> {
        self.packets = self.packets.saturating_add(1);
        self.last_delay_ms = 0;
        self.split_first = None;
        let kinds = self.transport.take_at(self.packets);
        let mut drop_status = false;
        let mut truncate = None;
        let mut corrupt_crc = false;
        let mut wrong_id = false;
        let mut duplicate = false;
        let mut garbage_prefix = false;
        let mut garbage_suffix = false;
        let mut reboot = false;
        for kind in kinds {
            match kind {
                FaultKind::DropStatus | FaultKind::DropStatusAfterApply => drop_status = true,
                FaultKind::DelayStatus { ms } | FaultKind::StatusAfterTimeout { ms } => {
                    self.last_delay_ms = self.last_delay_ms.max(ms);
                }
                FaultKind::TruncateStatus { keep } => truncate = Some(keep),
                FaultKind::CorruptOutgoingCrc => corrupt_crc = true,
                FaultKind::WrongStatusId => wrong_id = true,
                FaultKind::DuplicateStatus => duplicate = true,
                FaultKind::SplitStatusAcrossReads { first } => self.split_first = Some(first),
                FaultKind::GarbagePrefix => garbage_prefix = true,
                FaultKind::GarbageSuffix => garbage_suffix = true,
                FaultKind::DeviceSilent => self.silent = true,
                FaultKind::Disconnect => {
                    self.connected = false;
                    drop_status = true;
                }
                FaultKind::Reconnect => {
                    self.connected = true;
                    self.silent = false;
                }
                FaultKind::RebootDuringRequest => reboot = true,
                other => self
                    .device
                    .lock()
                    .expect("virtual xl330")
                    .apply_fault_kind(other),
            }
        }
        if !self.connected || self.silent {
            return Vec::new();
        }
        if reboot {
            let id = self.device.lock().expect("virtual xl330").id();
            let _ = self
                .device
                .lock()
                .expect("virtual xl330")
                .process(&encode_reboot(id));
        }
        let mut status = self.device.lock().expect("virtual xl330").process(inst);
        if drop_status {
            status.clear();
        }
        if wrong_id {
            if let Ok(st) = realityos_metal::protocol::decode_status(&status) {
                status = encode_status(st.id.wrapping_add(1), st.error, &st.params);
            }
        }
        if corrupt_crc {
            status = corrupt_crc_bytes(status);
        }
        if let Some(keep) = truncate {
            if keep < status.len() {
                status.truncate(keep);
            }
        }
        if duplicate && !status.is_empty() {
            let copy = status.clone();
            status.extend_from_slice(&copy);
        }
        if garbage_prefix {
            let junk = corrupt_crc_bytes(encode_status(254, 0, &[]));
            let mut p = junk;
            p.extend_from_slice(&status);
            status = p;
        }
        if garbage_suffix {
            status.extend_from_slice(GARBAGE_SUFFIX);
        }
        let mut out = Vec::new();
        if self.echo {
            out.extend_from_slice(inst);
        }
        out.extend_from_slice(&status);
        if !status.is_empty() {
            self.rx.push(status);
        }
        out
    }
}

fn take_complete_instruction(acc: &mut Vec<u8>) -> Option<Vec<u8>> {
    if acc.len() < 10 {
        return None;
    }
    let start = acc.windows(4).position(|w| w == HEADER.as_slice())?;
    if start > 0 {
        acc.drain(..start);
    }
    for end in 10..=acc.len() {
        if decode_instruction(&acc[..end]).is_ok() {
            return Some(acc.drain(..end).collect());
        }
    }
    None
}
