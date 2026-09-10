use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Provenance {
    ModelDeclared,
    CalibrationMeasured,
    HardwareMeasured,
    SimulatorDerived,
    UserDeclared,
    LearnedEstimate,
    Assumed,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Provenanced<T> {
    pub value: Option<T>,
    pub provenance: Provenance,
    pub source: String,
    pub as_of_s: f64,
    pub uncertainty: Option<T>,
}

impl<T> Provenanced<T> {
    pub fn unknown(source: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            value: None,
            provenance: Provenance::Unknown,
            source: source.into(),
            as_of_s,
            uncertainty: None,
        }
    }

    pub fn declared(value: T, source: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::ModelDeclared,
            source: source.into(),
            as_of_s,
            uncertainty: None,
        }
    }

    pub fn simulator_derived(value: T, source: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::SimulatorDerived,
            source: source.into(),
            as_of_s,
            uncertainty: None,
        }
    }

    pub fn assumed(value: T, source: impl Into<String>, as_of_s: f64) -> Self {
        Self {
            value: Some(value),
            provenance: Provenance::Assumed,
            source: source.into(),
            as_of_s,
            uncertainty: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_mass_is_unknown_not_a_default() {
        let m: Provenanced<f64> = Provenanced::unknown("body.mass_kg", 0.0);
        assert!(m.value.is_none());
        assert_eq!(m.provenance, Provenance::Unknown);
        assert!(m.uncertainty.is_none());
    }

    #[test]
    fn assumed_requires_an_explicit_note_source() {
        let mu = Provenanced::assumed(0.6, "user:earth_indoor_screen", 0.0);
        assert_eq!(mu.provenance, Provenance::Assumed);
        assert_eq!(mu.value, Some(0.6));
        assert!(mu.source.contains("user:"));
    }
}
