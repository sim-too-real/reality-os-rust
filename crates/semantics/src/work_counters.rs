//! Measured counts of geometry work already performed on this thread.
//! `None` is reserved for callers that never entered the owning function.

use std::cell::Cell;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WorkSnapshot {
    pub ik_attempts: u64,
    pub fk_evaluations: u64,
    pub collision_queries: u64,
    pub jacobian_evaluations: u64,
    pub mechanics_evaluations: u64,
}

struct Counts {
    ik: Cell<u64>,
    fk: Cell<u64>,
    collision: Cell<u64>,
    jacobian: Cell<u64>,
    mechanics: Cell<u64>,
}

impl Counts {
    const fn new() -> Self {
        Self {
            ik: Cell::new(0),
            fk: Cell::new(0),
            collision: Cell::new(0),
            jacobian: Cell::new(0),
            mechanics: Cell::new(0),
        }
    }
}

thread_local! {
    static COUNTS: Counts = const { Counts::new() };
}

fn bump(cell: &Cell<u64>) {
    cell.set(cell.get().saturating_add(1));
}

pub fn note_ik_attempt() {
    COUNTS.with(|counts| bump(&counts.ik));
}

pub fn note_fk_evaluation() {
    COUNTS.with(|counts| bump(&counts.fk));
}

pub fn note_collision_query() {
    COUNTS.with(|counts| bump(&counts.collision));
}

pub fn note_jacobian_evaluation() {
    COUNTS.with(|counts| bump(&counts.jacobian));
}

pub fn note_mechanics_evaluation() {
    COUNTS.with(|counts| bump(&counts.mechanics));
}

pub fn work_snapshot() -> WorkSnapshot {
    COUNTS.with(|counts| WorkSnapshot {
        ik_attempts: counts.ik.get(),
        fk_evaluations: counts.fk.get(),
        collision_queries: counts.collision.get(),
        jacobian_evaluations: counts.jacobian.get(),
        mechanics_evaluations: counts.mechanics.get(),
    })
}

pub fn reset_work_counters() {
    COUNTS.with(|counts| {
        counts.ik.set(0);
        counts.fk.set(0);
        counts.collision.set(0);
        counts.jacobian.set(0);
        counts.mechanics.set(0);
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counters_increase_only_when_the_owning_note_runs() {
        reset_work_counters();
        assert_eq!(work_snapshot(), WorkSnapshot::default());
        note_ik_attempt();
        note_fk_evaluation();
        note_fk_evaluation();
        note_collision_query();
        note_jacobian_evaluation();
        note_mechanics_evaluation();
        assert_eq!(
            work_snapshot(),
            WorkSnapshot {
                ik_attempts: 1,
                fk_evaluations: 2,
                collision_queries: 1,
                jacobian_evaluations: 1,
                mechanics_evaluations: 1,
            }
        );
    }
}
