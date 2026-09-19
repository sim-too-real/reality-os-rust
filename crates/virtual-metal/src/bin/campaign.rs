use clap::Parser;
use realityos_virtual_metal::{run_campaign, run_tier2_smoke};
use std::path::PathBuf;

#[derive(Parser, Debug)]
struct Args {
    #[arg(long, default_value_t = 1)]
    seed: u64,
    #[arg(long, default_value_t = 1000)]
    n: usize,
    #[arg(long, default_value_t = 1)]
    tier: u8,
    #[arg(long)]
    out: Option<PathBuf>,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let rec = if args.tier >= 2 {
        run_tier2_smoke(args.seed, args.n).map_err(|e| anyhow::anyhow!(e))?
    } else {
        run_campaign(args.seed, args.n)
    };
    let json = serde_json::to_string_pretty(&rec)?;
    if let Some(path) = args.out {
        std::fs::write(&path, &json)?;
    } else {
        println!("{json}");
    }
    if rec.hardware_present {
        anyhow::bail!("virtual_metal_refuses_hardware_present=true");
    }
    if rec.schema != realityos_virtual_metal::CAMPAIGN_SCHEMA {
        anyhow::bail!("schema");
    }
    if rec.n_fail > 0 {
        anyhow::bail!("virtual_metal_campaign_fail n_fail={}", rec.n_fail);
    }
    Ok(())
}
