//! Product wiring: sensor in → session → Governor → backed plant → port.
//! Does not invent a second write path.

use realityos_core::IssuedCommand;
use realityos_data::{DebugEvent, EventKind, EventStore, SessionSnapshot};
use realityos_kernel::{CorrelationId, Layer};
use realityos_plant::{
    ActionParams, FieldbusLink, HardwareBackedPlant, Plant, SimulatedHardwarePort,
};
use realityos_ros2::{architecture_map, ConnectionLayer, JointState, LinkState};

use crate::mode::SessionStartError;
use crate::session::{DispatchResult, RuntimeSession, StartArgs};

pub struct HardwareControlBridge<P: Plant> {
    pub session: RuntimeSession<P>,
    pub events: EventStore,
    pub fieldbus: FieldbusLink,
}

impl HardwareControlBridge<HardwareBackedPlant<SimulatedHardwarePort>> {
    /// SIM/HIL rails harness. Identity is HARNESS-* (not SIM_*), metal remains false.
    pub fn sim_harness(
        release_hash: impl Into<String>,
        now_s: f64,
    ) -> Result<Self, SessionStartError> {
        let port = SimulatedHardwarePort::new("HARNESS-SERIAL-001");
        let plant = HardwareBackedPlant::new(port, "hardware_backed", 1, 10.0);
        let mut args = StartArgs::simulation(release_hash);
        args.serial_or_as_built = "HARNESS-SERIAL-001".into();
        args.firmware_id = "HARNESS-FW-1.0.0".into();
        args.calibration_id = "HARNESS-CAL-001".into();
        args.design_content_hash = "harness_design_content".into();
        let mut session = RuntimeSession::start(args, plant, now_s)?;
        let mut ident = session.governor.plant_mut().probe_identity();
        ident.metal = false;
        let mut events = EventStore::new();
        let _ = events.append(
            DebugEvent::new(
                Layer::Session,
                EventKind::SessionStart,
                true,
                now_s,
                "session",
            )
            .with_payload(serde_json::json!({
                "release": session.governor.identity().release_hash.as_str(),
                "metal": false,
            })),
        );
        let _ = ident;
        Ok(Self {
            session,
            events,
            fieldbus: FieldbusLink::named_hole(),
        })
    }
}

impl<P: Plant> HardwareControlBridge<P> {
    pub fn ingest_joint_state(&mut self, js: &JointState, now_s: f64) -> Result<String, String> {
        if !js.finite() {
            return Err("non_finite_joint_state".into());
        }
        let samples = js.to_samples();
        let hash = self
            .session
            .ingest_sensor(&samples, Some(js.timestamp_s), now_s)?;
        let corr = CorrelationId::new(format!("sensor-{hash}"));
        let _ = self.events.append(
            DebugEvent::new(
                Layer::Bridge,
                EventKind::SensorIngest,
                true,
                now_s,
                corr.as_str(),
            )
            .with_payload(serde_json::json!({"hash": hash, "n": samples.len()})),
        );
        Ok(hash)
    }

    pub fn dispatch(&mut self, command: IssuedCommand, now_s: f64) -> DispatchResult {
        let corr = CorrelationId::from_command(command.command_id(), command.sequence_value());
        let out =
            self.session
                .bind_and_dispatch(command.into_command(), &ActionParams::empty(), now_s);
        let kind = if out.ok {
            EventKind::Write
        } else {
            EventKind::Refuse
        };
        let _ = self.events.append(
            DebugEvent::new(Layer::Bridge, kind, out.ok, now_s, corr.as_str()).with_payload(
                serde_json::json!({
                    "violations": out.violations,
                    "mode": out.mode.as_str(),
                }),
            ),
        );
        out
    }

    pub fn snapshot(&self) -> SessionSnapshot {
        let mut snap = SessionSnapshot::empty();
        snap.mode = self.session.mode.as_str().into();
        snap.release_hash = self
            .session
            .governor
            .identity()
            .release_hash
            .as_str()
            .into();
        snap.serial = self.session.governor.identity().serial_str().into();
        snap.estop = self.session.governor.estop();
        snap.last_heartbeat_s = self.session.governor.last_heartbeat_s();
        snap.last_sensor_s = self.session.governor.last_sensor_s();
        snap.last_sensor_hash = self.session.last_sensor_hash().map(str::to_string);
        snap.writes = self
            .events
            .query(&realityos_data::EventFilter {
                kind: Some(EventKind::Write),
                ok: Some(true),
                ..realityos_data::EventFilter::default()
            })
            .len() as u64;
        snap.refuses = self.events.refuses().len() as u64;
        snap.metal = false;
        snap
    }

    pub fn connection_report(&self) -> Vec<ConnectionLayer> {
        let mut layers = architecture_map();
        for l in &mut layers {
            if l.name == "fieldbus" {
                l.state = match self.fieldbus.state() {
                    realityos_plant::LinkState::Present => LinkState::Present,
                    realityos_plant::LinkState::NamedHole => LinkState::NamedHole,
                    realityos_plant::LinkState::Refused => LinkState::Refused,
                };
                l.note = self.fieldbus.note.clone();
            }
        }
        layers
    }
}
