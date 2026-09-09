//! ONLINE plants refuse motion outside `execute_certified_command`.
//! The write token is crate-private. Sibling crates cannot enter the guard.

use std::cell::Cell;

use crate::error::{PlantError, PlantResult};
use crate::traits::Plant;

thread_local! {
    static IN_CERTIFIED_WRITE: Cell<u32> = const { Cell::new(0) };
}

#[inline]
pub(crate) fn in_certified_write() -> bool {
    IN_CERTIFIED_WRITE.with(|c| c.get() > 0)
}

pub(crate) struct CertifiedWriteGuard;

impl CertifiedWriteGuard {
    pub(crate) fn enter() -> Self {
        IN_CERTIFIED_WRITE.with(|c| c.set(c.get().saturating_add(1)));
        Self
    }
}

impl Drop for CertifiedWriteGuard {
    fn drop(&mut self) {
        IN_CERTIFIED_WRITE.with(|c| c.set(c.get().saturating_sub(1)));
    }
}

pub(crate) fn with_certified_write<T>(f: impl FnOnce() -> T) -> T {
    let _g = CertifiedWriteGuard::enter();
    f()
}

pub fn plant_requires_certified_write(plant: &dyn Plant) -> bool {
    plant.is_online() || plant.caps().online
}

pub fn refuse_uncertified_online_write(plant: &dyn Plant, method: &'static str) -> PlantResult<()> {
    if plant_requires_certified_write(plant) && !in_certified_write() {
        return Err(PlantError::UncertifiedOnline { method });
    }
    Ok(())
}
