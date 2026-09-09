//! Finite, bounded scalars. Illegal magnitudes are unrepresentable at the constructor.

use crate::error::{KernelError, KernelResult};

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct FiniteF64(f64);

impl FiniteF64 {
    pub fn new(value: f64, field: &'static str) -> KernelResult<Self> {
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(KernelError::validation(field, "must be finite"))
        }
    }

    #[inline]
    pub const fn get(self) -> f64 {
        self.0
    }
}

pub fn require_finite(value: f64, field: &'static str) -> KernelResult<f64> {
    FiniteF64::new(value, field).map(FiniteF64::get)
}

pub fn require_positive(value: f64, field: &'static str) -> KernelResult<f64> {
    let v = require_finite(value, field)?;
    if v > 0.0 {
        Ok(v)
    } else {
        Err(KernelError::validation(field, "must be positive"))
    }
}

pub fn require_finite_slice(values: &[f64], field: &'static str) -> KernelResult<()> {
    if values.iter().any(|x| !x.is_finite()) {
        return Err(KernelError::validation(field, "non-finite"));
    }
    Ok(())
}

pub fn require_exact_len(values: &[f64], expected: usize, field: &'static str) -> KernelResult<()> {
    if values.len() != expected {
        return Err(KernelError::validation(field, "dimension mismatch"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_nan_and_non_positive() {
        assert!(require_finite(f64::NAN, "x").is_err());
        assert!(require_positive(0.0, "ttl").is_err());
        assert!(require_positive(1.5, "ttl").is_ok());
    }

    #[test]
    fn exact_len_is_strict() {
        assert!(require_exact_len(&[1.0, 2.0], 2, "q").is_ok());
        assert!(require_exact_len(&[1.0], 2, "q").is_err());
    }
}
