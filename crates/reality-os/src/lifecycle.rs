//! Compiler-visible command stages. Signing happens after identity/evidence bind.
//!
//! UntrustedProposal (Intent / PolicyProposal)
//!   → IssuedCommand (`RealityOs::decide` only)
//!   → CertifiedIntent
//!   → SessionBound
//!   → EvidenceBound
//!   → Signed
//!   → Acknowledged          (SIM lifecycle / fixture)
//!   → OnlineWrite           (ONLINE governor capability; not this module)
//!   → Prepared / Consumed / Unknown  (plant ledger)

use crate::command::{CertifiedCommand, IssuedCommand};

#[derive(Debug, Clone)]
pub struct CertifiedIntent(CertifiedCommand);

#[derive(Debug, Clone)]
pub struct SessionBound(CertifiedCommand);

#[derive(Debug, Clone)]
pub struct EvidenceBound(CertifiedCommand);

#[derive(Debug, Clone)]
pub struct SignedCommand(CertifiedCommand);

#[derive(Debug, Clone)]
pub struct AcknowledgedCommand(CertifiedCommand);

impl CertifiedIntent {
    pub fn from_issued(cmd: IssuedCommand) -> Self {
        Self(cmd.into_command())
    }

    pub fn from_command(cmd: CertifiedCommand) -> Self {
        Self(cmd)
    }

    pub fn bind_identity(
        self,
        release_hash: &str,
        as_built: &str,
        calibration_id: &str,
    ) -> Result<SessionBound, Vec<String>> {
        Ok(SessionBound(self.0.bind_identity(
            release_hash,
            as_built,
            calibration_id,
        )?))
    }

    /// SIM-only. ONLINE acknowledgement is an `OnlineWrite` transition.
    pub fn acknowledge_sim(self) -> AcknowledgedCommand {
        AcknowledgedCommand(self.0.acknowledge())
    }
}

impl SessionBound {
    pub fn bind_evidence(self, expected_hash: &str) -> Result<EvidenceBound, Vec<String>> {
        Ok(EvidenceBound(self.0.bind_evidence(expected_hash)?))
    }

    /// SIM-only shortcut. ONLINE acknowledgement is an `OnlineWrite` transition.
    pub fn acknowledge_sim(self) -> AcknowledgedCommand {
        AcknowledgedCommand(self.0.acknowledge())
    }

    pub fn into_command(self) -> CertifiedCommand {
        self.0
    }
}

impl EvidenceBound {
    pub fn sign(self, key: &[u8]) -> SignedCommand {
        SignedCommand(self.0.sign(key))
    }
}

impl SignedCommand {
    pub fn acknowledge(self) -> AcknowledgedCommand {
        AcknowledgedCommand(self.0.acknowledge())
    }
}

impl AcknowledgedCommand {
    pub fn as_command(&self) -> &CertifiedCommand {
        &self.0
    }

    pub fn into_command(self) -> CertifiedCommand {
        self.0
    }
}
