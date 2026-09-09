//! Distinct time domains. A physical watchdog must never read sim or ROS time.

use crate::error::{KernelError, KernelResult};
use crate::units::require_finite;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct MonoTime(f64);

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct SimTime(f64);

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct SyncTime(f64);

macro_rules! time_new {
    ($name:ident) => {
        impl $name {
            pub fn from_secs(secs: f64) -> KernelResult<Self> {
                Ok(Self(require_finite(secs, stringify!($name))?))
            }

            #[inline]
            pub const fn secs(self) -> f64 {
                self.0
            }
        }
    };
}

time_new!(MonoTime);
time_new!(SimTime);
time_new!(SyncTime);

impl MonoTime {
    pub fn checked_add(self, dt_s: f64) -> KernelResult<Self> {
        let dt = require_finite(dt_s, "dt_s")?;
        let sum = self.0 + dt;
        if !sum.is_finite() {
            return Err(KernelError::validation("deadline", "overflow"));
        }
        Self::from_secs(sum)
    }

    pub fn saturating_age_s(self, later: Self) -> f64 {
        (later.0 - self.0).max(0.0)
    }
}

/// Positive TTL on a monotonic clock. Rejects non-finite, non-positive, and overflow.
pub fn deadline_from_ttl(now: MonoTime, ttl_s: f64) -> KernelResult<MonoTime> {
    if !ttl_s.is_finite() {
        return Err(KernelError::validation("ttl_s", "non-finite"));
    }
    if ttl_s <= 0.0 {
        return Err(KernelError::validation("ttl_s", "must be positive"));
    }
    now.checked_add(ttl_s)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ttl_must_be_positive_and_finite() {
        let now = MonoTime::from_secs(1.0).unwrap();
        assert!(deadline_from_ttl(now, 0.0).is_err());
        assert!(deadline_from_ttl(now, f64::NAN).is_err());
        assert_eq!(deadline_from_ttl(now, 2.0).unwrap().secs(), 3.0);
    }

    #[test]
    fn overflow_is_rejected() {
        let now = MonoTime::from_secs(f64::MAX).unwrap();
        assert!(now.checked_add(f64::MAX).is_err());
    }
}
