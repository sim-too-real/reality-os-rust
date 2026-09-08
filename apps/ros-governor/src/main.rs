//! CLI composition root. SIM. Never metal. Never invent.

use anyhow::{bail, Result};
use clap::{Parser, Subcommand};
use realityos_core::{Intent, RealityOs, WorldView};
use realityos_plant::SimPlant;
use realityos_ros2::VetoMsg;
use realityos_session::{RuntimeMode, RuntimeSession, SafetyEdge, StartArgs};

#[derive(Parser)]
#[command(
    name = "ros-governor",
    about = "Reality OS + Governor last-gate (SIM). Learned systems never write motors.",
    version
)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Print product identity and honesty stamps.
    Status,
    /// Plan/certify/decide. Does not write a plant.
    Decide {
        #[arg(long, default_value = "hold")]
        verb: String,
        #[arg(long, default_value = "1.0")]
        tau_max: f64,
        #[arg(long)]
        action: Option<f64>,
        #[arg(long)]
        source: Option<String>,
    },
    /// Start a SIM session, decide, bind, dispatch.
    Dispatch {
        #[arg(long, default_value = "hold")]
        verb: String,
        #[arg(long, default_value = "rel-sim-cli")]
        release: String,
    },
    /// Engage software e-stop analog (not ISO 13850).
    Estop {
        #[arg(long, default_value = "manual")]
        reason: String,
    },
    /// Sealed safety-edge decide+gate (null plant).
    Edge {
        #[arg(long, default_value = "hold")]
        verb: String,
    },
    /// Print ROS 2 adapter contracts.
    Topics,
    /// Print every connection layer from first principles to metal (holes named).
    Chain,
    /// Dump data-layer events after a SIM harness decide+dispatch.
    Debug,
    /// Run Governor + Reality OS scenario matrices (100s of cases).
    Gauntlet,
    /// Grok/offline skill propose (never motors).
    Propose {
        #[arg(long, default_value = "hold the payload")]
        prompt: String,
    },
    /// Print first-principles rate bands.
    Rates,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt().with_env_filter("info").init();
    match Cli::parse().command {
        Commands::Status => status(),
        Commands::Decide {
            verb,
            tau_max,
            action,
            source,
        } => decide(&verb, tau_max, action, source.as_deref()),
        Commands::Dispatch { verb, release } => dispatch(&verb, &release),
        Commands::Estop { reason } => estop(&reason),
        Commands::Edge { verb } => edge(&verb),
        Commands::Topics => topics(),
        Commands::Chain => chain(),
        Commands::Debug => debug_run(),
        Commands::Gauntlet => gauntlet(),
        Commands::Propose { prompt } => propose(&prompt),
        Commands::Rates => rates(),
    }
}

fn status() -> Result<()> {
    println!("product: theworld-runtime (Rust)");
    println!("schema_family: {}", realityos_kernel::SCHEMA_FAMILY);
    println!("metal: false");
    println!("measured: false");
    println!("invent_authority: false");
    println!("learned_actuator_authority: false");
    println!("online: false");
    println!("modes: SIMULATION | HIL | ONLINE (ONLINE start is fail-closed; this CLI is SIM)");
    println!("last_gate: RuntimeGovernor.write_driver");
    println!("kernel: RealityOs.decide");
    Ok(())
}

fn decide(verb: &str, tau_max: f64, action: Option<f64>, source: Option<&str>) -> Result<()> {
    let mut ros = RealityOs::new();
    let mut req = realityos_core::DecideRequest::new(
        Intent::language(verb, verb),
        WorldView {
            tau_max: vec![tau_max],
            ..WorldView::default()
        },
        1.0,
    );
    if let Some(a) = action {
        req.proposal = Some(realityos_core::PolicyProposal {
            action: vec![a],
            source: source.unwrap_or("operator").into(),
            policy_id: "cli".into(),
        });
    }
    let d = ros.decide(req);
    println!("status: {}", d.status);
    println!("allowed: {}", d.allowed);
    println!("reason: {}", d.physical_reason);
    println!("command: {}", d.command.is_some());
    println!("metal: false");
    Ok(())
}

fn dispatch(verb: &str, release: &str) -> Result<()> {
    let plant = SimPlant::new("cli", 1, 10.0);
    let mut sess = RuntimeSession::start(StartArgs::simulation(release), plant, 10.0)
        .map_err(|e| anyhow::anyhow!(e))?;
    sess.governor.mark_sensor(10.0, None);
    let mut ros = RealityOs::new();
    let d = ros.decide(realityos_core::DecideRequest::new(
        Intent::language(verb, verb),
        WorldView {
            tau_max: vec![5.0],
            ..WorldView::default()
        },
        10.0,
    ));
    if !d.allowed {
        bail!("kernel {} — {}", d.status, d.physical_reason);
    }
    let cmd = d.command.expect("allow issues command");
    let out = sess.bind_and_dispatch(cmd, &realityos_plant::ActionParams::empty(), 10.0);
    println!("{}", serde_json::to_string_pretty(&out.to_json())?);
    Ok(())
}

fn estop(reason: &str) -> Result<()> {
    let plant = SimPlant::new("cli", 1, 10.0);
    let mut sess = RuntimeSession::start(StartArgs::simulation("rel-estop"), plant, 1.0)
        .map_err(|e| anyhow::anyhow!(e))?;
    let t = sess.governor.engage_estop(reason, 1.0);
    let v = VetoMsg::from_trace(&t);
    println!("veto: {}", v.veto);
    println!("reason: {}", v.reason);
    println!("mode: {}", RuntimeMode::Simulation.as_str());
    println!("iso_certified: false");
    Ok(())
}

fn edge(verb: &str) -> Result<()> {
    let mut edge = SafetyEdge::default();
    let r = edge.decide_and_gate(
        Intent::language(verb, verb),
        WorldView {
            tau_max: vec![5.0],
            ..WorldView::default()
        },
        1.0,
        "rel-edge",
    );
    println!("{}", serde_json::to_string_pretty(&r)?);
    Ok(())
}

fn topics() -> Result<()> {
    for (topic, ty) in realityos_ros2::TOPICS {
        println!("{topic}  {ty}");
    }
    println!(
        "hardware_writes_enabled: {}",
        realityos_ros2::HARDWARE_WRITES_ENABLED
    );
    Ok(())
}

fn chain() -> Result<()> {
    for layer in realityos_ros2::architecture_map() {
        println!(
            "{:02}  {:<24}  {:<12}  {}",
            layer.index,
            layer.name,
            format!("{:?}", layer.state).to_ascii_lowercase(),
            layer.note
        );
    }
    println!("metal: false");
    println!("fieldbus_tx: named_hole");
    Ok(())
}

fn debug_run() -> Result<()> {
    use realityos_session::HardwareControlBridge;
    let mut br =
        HardwareControlBridge::sim_harness("rel-debug", 1.0).map_err(|e| anyhow::anyhow!(e))?;
    let js = realityos_ros2::JointState {
        name: vec!["j1".into()],
        position: vec![0.0],
        velocity: vec![0.0],
        effort: vec![0.0],
        timestamp_s: 1.0,
    };
    br.ingest_joint_state(&js, 1.0)
        .map_err(|e| anyhow::anyhow!(e))?;
    let mut ros = RealityOs::new();
    let d = ros.decide(realityos_core::DecideRequest::new(
        Intent::language("hold", "hold"),
        WorldView {
            tau_max: vec![5.0],
            ..WorldView::default()
        },
        1.0,
    ));
    if let Some(mut cmd) = d.command {
        cmd.release_hash.clear();
        let _ = br.dispatch(cmd, 1.0);
    }
    println!("{}", serde_json::to_string_pretty(&br.snapshot())?);
    println!("events: {}", br.events.len());
    for e in br.events.events() {
        println!(
            "  seq={} {} {} ok={} corr={}",
            e.seq,
            e.layer,
            e.kind.as_str(),
            e.ok,
            e.correlation_id
        );
    }
    Ok(())
}

fn gauntlet() -> Result<()> {
    let g = realityos_gauntlet::governor_matrix();
    let r = realityos_gauntlet::reality_os_matrix();
    let (gn, gb) = realityos_gauntlet::summarize(&g);
    let (rn, rb) = realityos_gauntlet::summarize(&r);
    println!("governor_scenarios: {gn} fail: {gb}");
    println!("reality_os_scenarios: {rn} fail: {rb}");
    println!("robots: uniaxial, arm6, wheeled, unitree_h1");
    println!("envs: earth, moon, ice, high_g");
    println!("metal: false");
    if gb + rb > 0 {
        bail!("gauntlet failures governor={gb} reality_os={rb}");
    }
    Ok(())
}

fn propose(prompt: &str) -> Result<()> {
    let p = realityos_agent::offline_propose(prompt);
    println!("{}", serde_json::to_string_pretty(&p)?);
    println!("learned_actuator_authority: false");
    println!("live_grok: enable feature live-grok + XAI_API_KEY (not on 1kHz path)");
    Ok(())
}

fn rates() -> Result<()> {
    for b in realityos_rate::bands() {
        println!("{}  {} Hz  period={:.6}s", b.name, b.hz, b.period_s());
    }
    println!(
        "dispose_budget_fastpath_us: {}",
        realityos_rate::BUDGET_FASTPATH_US
    );
    println!("not_preempt_rt: true");
    Ok(())
}
