use thiserror::Error;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum PhysicsError {
    #[error("non-finite {0}")]
    NonFinite(&'static str),
    #[error("non-positive {0}")]
    NonPositive(&'static str),
    #[error("negative {0}")]
    Negative(&'static str),
}

pub type PhysicsResult<T> = Result<T, PhysicsError>;

pub fn finite(x: f64, name: &'static str) -> PhysicsResult<f64> {
    if x.is_finite() {
        Ok(x)
    } else {
        Err(PhysicsError::NonFinite(name))
    }
}

pub fn nonneg(x: f64, name: &'static str) -> PhysicsResult<f64> {
    let x = finite(x, name)?;
    if x < 0.0 {
        Err(PhysicsError::Negative(name))
    } else {
        Ok(x)
    }
}

pub fn positive(x: f64, name: &'static str) -> PhysicsResult<f64> {
    let x = finite(x, name)?;
    if x <= 0.0 {
        Err(PhysicsError::NonPositive(name))
    } else {
        Ok(x)
    }
}
