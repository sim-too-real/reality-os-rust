//! Friction belongs to an interacting pair. Record only coefficients the source provides.

use serde::{Deserialize, Serialize};

use crate::physical_quantity::PhysicalFactProvenance;
use crate::provenance::Provenanced;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PairFriction {
    pub body_a: String,
    pub body_b: String,
    /// Coulomb-like sliding coefficient when the source provides one number.
    pub sliding_mu: Provenanced<f64>,
    pub torsional_mu: Option<Provenanced<f64>>,
    pub rolling_mu: Option<Provenanced<f64>>,
    pub static_mu: Option<Provenanced<f64>>,
    pub dynamic_mu: Option<Provenanced<f64>>,
}

impl PairFriction {
    pub fn unknown(a: impl Into<String>, b: impl Into<String>, source: impl Into<String>) -> Self {
        Self {
            body_a: a.into(),
            body_b: b.into(),
            sliding_mu: Provenanced::unknown(source, 0.0),
            torsional_mu: None,
            rolling_mu: None,
            static_mu: None,
            dynamic_mu: None,
        }
    }

    /// Preserve exactly one Coulomb coefficient. Do not invent static/dynamic split.
    pub fn coulomb(a: impl Into<String>, b: impl Into<String>, mu: Provenanced<f64>) -> Self {
        Self {
            body_a: a.into(),
            body_b: b.into(),
            sliding_mu: mu,
            torsional_mu: None,
            rolling_mu: None,
            static_mu: None,
            dynamic_mu: None,
        }
    }

    pub fn sliding_mu_known(&self) -> Option<f64> {
        self.sliding_mu.known_value().copied()
    }

    pub fn sliding_provenance(&self) -> PhysicalFactProvenance {
        PhysicalFactProvenance::from(self.sliding_mu.provenance)
    }

    pub fn pair_matches(&self, a: &str, b: &str) -> bool {
        (self.body_a == a && self.body_b == b) || (self.body_a == b && self.body_b == a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::provenance::Provenance;

    #[test]
    fn single_coefficient_does_not_invent_static_dynamic() {
        let mu = Provenanced::declared(0.4, "scenario.pair", 0.0);
        let p = PairFriction::coulomb("tool", "object", mu);
        assert_eq!(p.sliding_mu_known(), Some(0.4));
        assert!(p.static_mu.is_none());
        assert!(p.dynamic_mu.is_none());
        assert!(p.torsional_mu.is_none());
        assert_eq!(p.sliding_mu.provenance, Provenance::ModelDeclared);
    }

    #[test]
    fn unknown_pair_stays_unknown() {
        let p = PairFriction::unknown("tool", "object", "missing");
        assert!(p.sliding_mu_known().is_none());
        assert_eq!(p.sliding_provenance(), PhysicalFactProvenance::Unknown);
    }
}
