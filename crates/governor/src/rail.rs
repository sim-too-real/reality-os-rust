//! Runtime typestates. ONLINE cannot implement the unlocked mutation trait.

mod sealed {
    pub trait Sealed {}
}

pub trait Rail: sealed::Sealed {
    const ONLINE_LOCKED: bool;
}

pub trait UnlockedRail: Rail {}

#[derive(Debug, Clone, Copy, Default)]
pub struct Simulation;

#[derive(Debug, Clone, Copy, Default)]
pub struct Hil;

#[derive(Debug, Clone, Copy, Default)]
pub struct OnlineLocked;

impl sealed::Sealed for Simulation {}
impl sealed::Sealed for Hil {}
impl sealed::Sealed for OnlineLocked {}

impl Rail for Simulation {
    const ONLINE_LOCKED: bool = false;
}
impl Rail for Hil {
    const ONLINE_LOCKED: bool = false;
}
impl Rail for OnlineLocked {
    const ONLINE_LOCKED: bool = true;
}

impl UnlockedRail for Simulation {}
impl UnlockedRail for Hil {}
