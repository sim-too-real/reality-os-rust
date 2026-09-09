//! Authority-owned time.
//!
//! * [`AuthorityClock::monotonic_now`] is the freshness / issue / expiry /
//!   heartbeat / watchdog / write-time anchor.
//! * [`unix_now_s`] is wall / synchronized **audit** time only. It is not
//!   monotonic and must not be used for safety freshness.

use std::sync::{Arc, Mutex};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use crate::time::MonoTime;

/// Seconds since Unix epoch. Tests pass an explicit value; never sleep on the gate.
///
/// Audit / CLI only. Not a freshness anchor.
#[inline]
pub fn unix_now_s() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

#[inline]
pub fn finite_or_err(value: f64, field: &'static str) -> crate::error::KernelResult<f64> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(crate::error::KernelError::validation(
            field,
            "must be finite",
        ))
    }
}

/// Monotonic clock the authority process consults for execution timing.
pub trait AuthorityClock: Send + Sync {
    fn monotonic_now(&self) -> MonoTime;
}

/// Production clock: OS monotonic time since construction (not Unix wall time).
pub struct OsMonotonicClock {
    origin: Instant,
}

impl OsMonotonicClock {
    pub fn new() -> Self {
        Self {
            origin: Instant::now(),
        }
    }
}

impl Default for OsMonotonicClock {
    fn default() -> Self {
        Self::new()
    }
}

impl AuthorityClock for OsMonotonicClock {
    fn monotonic_now(&self) -> MonoTime {
        let secs = self.origin.elapsed().as_secs_f64();
        MonoTime::from_secs(secs)
            .unwrap_or_else(|_| MonoTime::from_secs(0.0).expect("zero is finite"))
    }
}

/// Deterministic clock for tests and HIL fault injection.
pub struct FakeClock {
    now_s: Mutex<f64>,
}

impl FakeClock {
    pub fn at(now_s: f64) -> Self {
        Self {
            now_s: Mutex::new(now_s),
        }
    }

    pub fn arc(now_s: f64) -> Arc<Self> {
        Arc::new(Self::at(now_s))
    }

    pub fn set(&self, now_s: f64) {
        if let Ok(mut g) = self.now_s.lock() {
            *g = now_s;
        }
    }

    pub fn advance(&self, dt_s: f64) {
        if let Ok(mut g) = self.now_s.lock() {
            *g += dt_s;
        }
    }
}

impl AuthorityClock for FakeClock {
    fn monotonic_now(&self) -> MonoTime {
        let secs = self.now_s.lock().map(|g| *g).unwrap_or(0.0);
        MonoTime::from_secs(secs)
            .unwrap_or_else(|_| MonoTime::from_secs(0.0).expect("zero is finite"))
    }
}

/// Erase a concrete clock into the governor/session injection type.
pub fn arc_clock(clock: impl AuthorityClock + 'static) -> Arc<dyn AuthorityClock> {
    Arc::new(clock)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fake_clock_is_deterministic_and_settable() {
        let c = FakeClock::at(10.0);
        assert_eq!(c.monotonic_now().secs(), 10.0);
        c.advance(2.5);
        assert_eq!(c.monotonic_now().secs(), 12.5);
        c.set(3.0);
        assert_eq!(c.monotonic_now().secs(), 3.0);
    }

    #[test]
    fn os_monotonic_is_finite_and_non_decreasing() {
        let c = OsMonotonicClock::new();
        let a = c.monotonic_now().secs();
        let b = c.monotonic_now().secs();
        assert!(a.is_finite() && b.is_finite());
        assert!(b + 1e-12 >= a);
    }

    #[test]
    fn unix_wall_is_not_the_authority_clock_trait() {
        // Presence of unix_now_s is intentional for audit records.
        // Freshness must use AuthorityClock, not this function.
        let _ = unix_now_s();
        let _c: &dyn AuthorityClock = &OsMonotonicClock::new();
    }
}
