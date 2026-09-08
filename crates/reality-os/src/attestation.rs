//! Decision attestation chain. Distinct from the driver CommandLedger.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

const GENESIS: &str = "0000000000000000000000000000000000000000000000000000000000000000";

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AttestedRecord {
    pub seq: u64,
    pub status: String,
    pub physical_reason: String,
    pub content_hash: String,
    pub prev_hash: String,
    pub chain_hash: String,
}

#[derive(Debug, Clone, Default)]
pub struct CertificateLedger {
    records: Vec<AttestedRecord>,
    chain: String,
}

impl CertificateLedger {
    pub fn new() -> Self {
        Self {
            records: Vec::new(),
            chain: GENESIS.into(),
        }
    }

    pub fn append(&mut self, status: &str, physical_reason: &str) -> AttestedRecord {
        let seq = self.records.len() as u64 + 1;
        let content = format!("{seq}|{status}|{physical_reason}");
        let content_hash = hex::encode(Sha256::digest(content.as_bytes()));
        let chain_hash = hex::encode(Sha256::digest(
            format!("{}{}", self.chain, content_hash).as_bytes(),
        ));
        let rec = AttestedRecord {
            seq,
            status: status.into(),
            physical_reason: physical_reason.into(),
            content_hash,
            prev_hash: self.chain.clone(),
            chain_hash: chain_hash.clone(),
        };
        self.chain = chain_hash;
        self.records.push(rec.clone());
        rec
    }

    pub fn verify_chain(&self) -> Result<(), usize> {
        let mut prev = GENESIS.to_string();
        for (i, rec) in self.records.iter().enumerate() {
            if rec.prev_hash != prev {
                return Err(i);
            }
            let content = format!("{}|{}|{}", rec.seq, rec.status, rec.physical_reason);
            let content_hash = hex::encode(Sha256::digest(content.as_bytes()));
            if content_hash != rec.content_hash {
                return Err(i);
            }
            let expect = hex::encode(Sha256::digest(
                format!("{}{}", prev, content_hash).as_bytes(),
            ));
            if expect != rec.chain_hash {
                return Err(i);
            }
            prev = rec.chain_hash.clone();
        }
        Ok(())
    }

    pub fn len(&self) -> usize {
        self.records.len()
    }

    pub fn is_empty(&self) -> bool {
        self.records.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tamper_breaks_chain() {
        let mut l = CertificateLedger::new();
        l.append("allow", "ok");
        l.append("refuse", "no");
        assert!(l.verify_chain().is_ok());
        l.records[0].status = "abort".into();
        assert_eq!(l.verify_chain(), Err(0));
    }
}
