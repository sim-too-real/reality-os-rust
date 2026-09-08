//! First-principles SI used by the last-gate.
//!
//! These are identities and conventional constants. They are **not** MEASURED
//! plant parameters. Unknown μ / Kt / I stay caller-supplied screens.

pub mod energy;
pub mod error;
pub mod kinematics;
pub mod limits;
pub mod motor;
pub mod newton;
pub mod sampling;
pub mod si;

pub use energy::{contact_energy_j, mechanical_power_w, rotational_ke_j, translational_ke_j};
pub use error::{PhysicsError, PhysicsResult};
pub use kinematics::{coulomb_decel_m_s2, stop_distance_m, stop_time_s};
pub use limits::{in_limits, joint_limit_margin};
pub use motor::{joule_w, motor_torque_nm, thermal_derate};
pub use newton::{accel_from_force, force_n, torque_nm};
pub use sampling::{dispose_period_s, is_stale, nyquist_hz, period_s, screen_period_s};
pub use si::{DISPOSE_HZ, G0, SCREEN_HZ};

pub const SCHEMA: &str = "realityos.physics/1";
