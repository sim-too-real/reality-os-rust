use clap::Parser;
use realityos_virtual_metal::run_campaign;
use std::path::PathBuf;

#[derive(Parser, Debug)]
struct Args {
    #[arg(long, default_value_t = 1)]
    seed: u64,
    #[arg(long, default_value_t = 1000)]
    n: usize,
    #[arg(long)]
    out: Option<PathBuf>,
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    let rec = run_campaign(args.seed, args.n);
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
    Ok(())
}
