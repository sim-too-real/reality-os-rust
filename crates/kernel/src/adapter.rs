//! Versioned adapter handshake. Deterministic errors. No plant write.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::time::MonoTime;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AdapterOffer {
    pub protocol_version: u16,
    pub capabilities: Vec<String>,
    pub deadline: MonoTime,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdapterCancel {
    pub token: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AdapterSession {
    pub protocol_version: u16,
    pub granted: Vec<String>,
}

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum AdapterError {
    #[error("deadline exceeded")]
    DeadlineExceeded,
    #[error("cancelled")]
    Cancelled,
    #[error("unsupported capability {0}")]
    Unsupported(String),
    #[error("{0}")]
    Deterministic(String),
}

pub const ADAPTER_PROTOCOL: u16 = 1;

pub fn handshake(
    offer: &AdapterOffer,
    now: MonoTime,
    required: &[&str],
    cancel: Option<&AdapterCancel>,
) -> Result<AdapterSession, AdapterError> {
    if offer.protocol_version != ADAPTER_PROTOCOL {
        return Err(AdapterError::Deterministic(format!(
            "protocol_mismatch:{}",
            offer.protocol_version
        )));
    }
    if let Some(c) = cancel {
        if !c.token.is_empty() {
            return Err(AdapterError::Cancelled);
        }
    }
    if now.secs() > offer.deadline.secs() {
        return Err(AdapterError::DeadlineExceeded);
    }
    let mut granted = Vec::new();
    for cap in required {
        if !offer.capabilities.iter().any(|c| c == cap) {
            return Err(AdapterError::Unsupported((*cap).into()));
        }
        granted.push((*cap).to_string());
    }
    Ok(AdapterSession {
        protocol_version: ADAPTER_PROTOCOL,
        granted,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn handshake_errors_are_deterministic() {
        let now = MonoTime::from_secs(1.0).unwrap();
        let deadline = MonoTime::from_secs(2.0).unwrap();
        let offer = AdapterOffer {
            protocol_version: ADAPTER_PROTOCOL,
            capabilities: vec!["codec".into()],
            deadline,
        };
        let a = handshake(&offer, now, &["codec"], None);
        let b = handshake(&offer, now, &["codec"], None);
        assert_eq!(a, b);
        assert!(handshake(&offer, now, &["write"], None).is_err());
        let late = MonoTime::from_secs(3.0).unwrap();
        assert_eq!(
            handshake(&offer, late, &["codec"], None),
            Err(AdapterError::DeadlineExceeded)
        );
        let cancel = AdapterCancel {
            token: "c".into(),
        };
        assert_eq!(
            handshake(&offer, now, &["codec"], Some(&cancel)),
            Err(AdapterError::Cancelled)
        );
    }
}
