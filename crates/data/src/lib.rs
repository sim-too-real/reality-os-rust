//! Queryable event/data layer. Does not authorize motion.

pub mod event;
pub mod snapshot;
pub mod store;

pub use event::{DebugEvent, EventFilter, EventKind};
pub use snapshot::SessionSnapshot;
pub use store::{DataError, EventStore};

pub const SCHEMA: &str = "realityos.data/1";
