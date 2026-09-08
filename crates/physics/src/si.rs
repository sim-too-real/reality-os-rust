//! SI constants used by the gate. Exact or conventional — never MEASURED.

/// Standard acceleration of gravity (m/s²). Conventional gₙ, exact 9.80665.
pub const G0: f64 = 9.80665;

/// Dispose default (Hz). Dual-rate SIM; not a lab-measured loop.
pub const DISPOSE_HZ: f64 = 1000.0;

/// Screen default (Hz). Neural/proposal band.
pub const SCREEN_HZ: f64 = 50.0;
