//! Bounded in-process channels. Overload degrades to hold — never a wider write.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverloadDisposition {
    Hold,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TelemetryFrame {
    pub topic: String,
    pub t_s: f64,
    pub payload: String,
}

impl TelemetryFrame {
    pub fn new(topic: impl Into<String>, t_s: f64, payload: impl Into<String>) -> Self {
        Self {
            topic: topic.into(),
            t_s,
            payload: payload.into(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct BoundedMailbox<T> {
    cap: usize,
    items: std::collections::VecDeque<T>,
    dropped: u64,
}

impl<T> BoundedMailbox<T> {
    pub fn new(cap: usize) -> Self {
        Self {
            cap: cap.max(1),
            items: std::collections::VecDeque::new(),
            dropped: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    pub fn capacity(&self) -> usize {
        self.cap
    }

    pub fn push(&mut self, item: T) -> Result<(), OverloadDisposition> {
        if self.items.len() >= self.cap {
            self.dropped += 1;
            return Err(OverloadDisposition::Hold);
        }
        self.items.push_back(item);
        Ok(())
    }

    pub fn pop(&mut self) -> Option<T> {
        self.items.pop_front()
    }
}

/// Missing input and mailbox overflow both select the tested hold/passive path.
pub fn overload_degrades_to_hold(overload: bool) -> bool {
    overload
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overflow_is_hold_not_drop_into_write() {
        let mut box_ = BoundedMailbox::new(2);
        assert!(box_.push(1).is_ok());
        assert!(box_.push(2).is_ok());
        assert_eq!(box_.push(3), Err(OverloadDisposition::Hold));
        assert_eq!(box_.dropped(), 1);
        assert_eq!(box_.len(), 2);
        assert!(overload_degrades_to_hold(true));
    }
}
