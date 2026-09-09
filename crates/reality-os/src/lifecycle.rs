//! Compiler-visible command stages. Signing happens after identity/evidence bind.
//!
//! UntrustedProposal (Intent / PolicyProposal)
//!   → CertifiedIntent (`CertifiedCommand` from decide, unacked)
//!   → SessionBound
//!   → EvidenceBound
//!   → Signed
//!   → Acknowledged
//!   → Prepared / Consumed / Unknown  (plant ledger)

use crate::command::CertifiedCommand;

#[derive(Debug, Clone)]
pub struct CertifiedIntent(pub CertifiedCommand);

#[derive(Debug, Clone)]
pub struct SessionBound(pub CertifiedCommand);

#[derive(Debug, Clone)]
pub struct EvidenceBound(pub CertifiedCommand);

#[derive(Debug, Clone)]
pub struct SignedCommand(pub CertifiedCommand);

#[derive(Debug, Clone)]
pub struct AcknowledgedCommand(pub CertifiedCommand);

impl CertifiedIntent {
    pub fn from_issued(cmd: CertifiedCommand) -> Self {
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
}

impl SessionBound {
    pub fn bind_evidence(self, expected_hash: &str) -> Result<EvidenceBound, Vec<String>> {
        Ok(EvidenceBound(self.0.bind_evidence(expected_hash)?))
    }

    /// SIM-only shortcut. ONLINE must sign after evidence bind.
    pub fn acknowledge_sim(self) -> AcknowledgedCommand {
        AcknowledgedCommand(self.0.acknowledge())
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
