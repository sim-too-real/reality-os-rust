//! Init must not persist probe baud/id hints into metal.json.
#![cfg(unix)]

use realityos_metal::config::{MetalConfig, CONFIG_FILE};

#[test]
fn smoke_init_does_not_persist_baud_or_id_hints() {
    let root = std::env::temp_dir().join(format!(
        "realityos-metal-init-hint-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    let bin = env!("CARGO_BIN_EXE_realityos-metal-smoke");
    let status = std::process::Command::new(bin)
        .args([
            "--root",
            root.to_str().unwrap(),
            "--device",
            "/dev/ttyUSB0",
            "init",
        ])
        .env("REALITYOS_METAL_BAUD", "4000000")
        .env("REALITYOS_METAL_SERVO_ID", "7")
        .status()
        .expect("run realityos-metal-smoke init");
    assert!(status.success(), "init failed: {status}");
    let cfg = MetalConfig::load(root.join(CONFIG_FILE)).expect("metal.json");
    assert_eq!(
        cfg.baud, 57_600,
        "leftover REALITYOS_METAL_BAUD must not land in metal.json before discover"
    );
    assert_eq!(
        cfg.servo_id, 1,
        "leftover REALITYOS_METAL_SERVO_ID must not land in metal.json before discover"
    );
    let _ = std::fs::remove_dir_all(&root);
}
