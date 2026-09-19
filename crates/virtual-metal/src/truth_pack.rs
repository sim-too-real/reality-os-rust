//! XL330-M288 truth pack. Every field carries provenance, units, and schema.

use serde::{Deserialize, Serialize};

pub const TRUTH_PACK_SCHEMA: &str = "realityos.virtual_metal.truth_pack/1";
pub const TRUTH_PACK_VERSION: &str = "1";

const EMANUAL_XL330: &str =
    "https://docs.robotis.com/docs/dxl/model_reference/x_series/xl_series/xl330-m288";
const EMANUAL_PROTOCOL2: &str = "https://docs.robotis.com/docs/dxl/protocol/protocol2/";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum Provenance {
    ManufacturerSpecified,
    ManufacturerDerived,
    OfficialSdkBehavior,
    ThirdPartyMeasured,
    AcademicMeasured,
    PhysicsDerived,
    CalibratedFromRealHardware,
    Estimated,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Provenanced<T> {
    pub value: Option<T>,
    pub range: Option<(T, T)>,
    pub units: String,
    pub provenance: Provenance,
    pub source: String,
}

impl<T> Provenanced<T> {
    pub fn specified(value: T, units: &str, source: &str) -> Self {
        Self {
            value: Some(value),
            range: None,
            units: units.into(),
            provenance: Provenance::ManufacturerSpecified,
            source: source.into(),
        }
    }

    pub fn derived(value: T, units: &str, source: &str) -> Self {
        Self {
            value: Some(value),
            range: None,
            units: units.into(),
            provenance: Provenance::ManufacturerDerived,
            source: source.into(),
        }
    }

    pub fn estimated_range(lo: T, hi: T, units: &str, source: &str) -> Self {
        Self {
            value: None,
            range: Some((lo, hi)),
            units: units.into(),
            provenance: Provenance::Estimated,
            source: source.into(),
        }
    }

    pub fn unknown(units: &str, source: &str) -> Self {
        Self {
            value: None,
            range: None,
            units: units.into(),
            provenance: Provenance::Unknown,
            source: source.into(),
        }
    }

    pub fn is_unknown(&self) -> bool {
        matches!(self.provenance, Provenance::Unknown) && self.value.is_none()
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Xl330TruthPack {
    pub schema: String,
    pub version: String,
    pub model_number: Provenanced<u16>,
    pub hardware_model: Provenanced<String>,
    pub gear_ratio: Provenanced<f64>,
    pub encoder_pulses_per_rev: Provenanced<u16>,
    pub position_mode_min: Provenanced<i32>,
    pub position_mode_max: Provenanced<i32>,
    pub input_voltage_min_v: Provenanced<f64>,
    pub input_voltage_max_v: Provenanced<f64>,
    pub recommended_voltage_v: Provenanced<f64>,
    pub stall_torque_3v7_nm: Provenanced<f64>,
    pub stall_torque_5v_nm: Provenanced<f64>,
    pub stall_torque_6v_nm: Provenanced<f64>,
    pub stall_current_5v_a: Provenanced<f64>,
    pub no_load_speed_3v7_rpm: Provenanced<f64>,
    pub no_load_speed_5v_rpm: Provenanced<f64>,
    pub no_load_speed_6v_rpm: Provenanced<f64>,
    pub mass_kg: Provenanced<f64>,
    pub protocol: Provenanced<u8>,
    pub factory_id: Provenanced<u8>,
    pub factory_baud_index: Provenanced<u8>,
    pub pwm_limit_raw: Provenanced<u16>,
    pub current_limit_ma: Provenanced<u16>,
    pub velocity_limit_raw: Provenanced<u32>,
    /// Coexists with manufacturer stall. Not filled from public data.
    pub stall_torque_measured_nm: Provenanced<f64>,
    pub gearbox_efficiency: Provenanced<f64>,
    pub backlash_rad: Provenanced<f64>,
    pub motor_resistance_ohm: Provenanced<f64>,
    pub motor_kt_nm_per_a: Provenanced<f64>,
    pub thermal_time_s: Provenanced<f64>,
    pub boot_delay_s: Provenanced<f64>,
    pub encoder_noise_ticks: Provenanced<f64>,
    pub usb_adapter_latency_s: Provenanced<f64>,
}

impl Xl330TruthPack {
    pub fn xl330_m288() -> Self {
        Self {
            schema: TRUTH_PACK_SCHEMA.into(),
            version: TRUTH_PACK_VERSION.into(),
            model_number: Provenanced::specified(1200, "1", EMANUAL_XL330),
            hardware_model: Provenanced::specified("XL330-M288-T".into(), "1", EMANUAL_XL330),
            gear_ratio: Provenanced::specified(288.4, "1", EMANUAL_XL330),
            encoder_pulses_per_rev: Provenanced::specified(4096, "pulse/rev", EMANUAL_XL330),
            position_mode_min: Provenanced::specified(0, "pulse", EMANUAL_XL330),
            position_mode_max: Provenanced::specified(4095, "pulse", EMANUAL_XL330),
            input_voltage_min_v: Provenanced::specified(3.7, "V", EMANUAL_XL330),
            input_voltage_max_v: Provenanced::specified(6.0, "V", EMANUAL_XL330),
            recommended_voltage_v: Provenanced::specified(5.0, "V", EMANUAL_XL330),
            stall_torque_3v7_nm: Provenanced::specified(0.42, "N.m", EMANUAL_XL330),
            stall_torque_5v_nm: Provenanced::specified(0.52, "N.m", EMANUAL_XL330),
            stall_torque_6v_nm: Provenanced::specified(0.60, "N.m", EMANUAL_XL330),
            stall_current_5v_a: Provenanced::specified(1.47, "A", EMANUAL_XL330),
            no_load_speed_3v7_rpm: Provenanced::specified(76.0, "rev/min", EMANUAL_XL330),
            no_load_speed_5v_rpm: Provenanced::specified(103.0, "rev/min", EMANUAL_XL330),
            no_load_speed_6v_rpm: Provenanced::specified(123.0, "rev/min", EMANUAL_XL330),
            mass_kg: Provenanced::derived(0.018, "kg", EMANUAL_XL330),
            protocol: Provenanced::specified(2, "1", EMANUAL_PROTOCOL2),
            factory_id: Provenanced::specified(1, "1", EMANUAL_XL330),
            factory_baud_index: Provenanced::specified(1, "1", EMANUAL_XL330),
            pwm_limit_raw: Provenanced::specified(885, "0.113%", EMANUAL_XL330),
            current_limit_ma: Provenanced::specified(1750, "mA", EMANUAL_XL330),
            velocity_limit_raw: Provenanced::specified(445, "0.229 rev/min", EMANUAL_XL330),
            stall_torque_measured_nm: Provenanced::unknown("N.m", "no CALIBRATED_FROM_REAL_HARDWARE in V1"),
            gearbox_efficiency: Provenanced::estimated_range(
                0.4,
                0.85,
                "1",
                "engineering bound; not manufacturer-specified for XL330-M288",
            ),
            backlash_rad: Provenanced::unknown("rad", "not in XL330-M288 e-Manual"),
            motor_resistance_ohm: Provenanced::unknown("ohm", "phase R not published for XL330-M288"),
            motor_kt_nm_per_a: Provenanced::unknown(
                "N.m/A",
                "manufacturer publishes output stall Nm/A, not motor-side Kt",
            ),
            thermal_time_s: Provenanced::unknown("s", "thermal time constant not published"),
            boot_delay_s: Provenanced::estimated_range(
                0.3,
                1.5,
                "s",
                "in-repo metal driver REBOOT_IDENTIFY_DEADLINE_MS=1500; not a datasheet distribution",
            ),
            encoder_noise_ticks: Provenanced::unknown("pulse", "AS5601 noise not characterized here"),
            usb_adapter_latency_s: Provenanced::unknown("s", "adapter-specific; not an XL330 property"),
        }
    }
}
