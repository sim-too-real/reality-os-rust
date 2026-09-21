//! Process-isolated MuJoCo worker client. Isolated mjModel/mjData per instance.

use crate::bundle::RobotBundle;
use crate::format::ModelFormat;
use serde_json::{json, Value};
use std::io::{BufRead, BufReader, Read, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;
use thiserror::Error;

static WORKER_SEQ: AtomicU64 = AtomicU64::new(1);
static WORKER_POOL: Mutex<Vec<MujocoInstance>> = Mutex::new(Vec::new());

/// Recycle an isolated worker process. Each checkout still has its own mjModel/mjData after load.
pub fn checkout_worker() -> Result<MujocoInstance, ExecError> {
    if let Ok(mut pool) = WORKER_POOL.lock() {
        if let Some(w) = pool.pop() {
            if w.alive() {
                return Ok(w);
            }
        }
    }
    MujocoInstance::spawn()
}

pub fn checkin_worker(inst: MujocoInstance) {
    if !inst.alive() {
        return;
    }
    // Bound native MjModel lifetime on large Menagerie compiles.
    if inst.compile_count >= 8 {
        drop(inst);
        return;
    }
    if let Ok(mut pool) = WORKER_POOL.lock() {
        if pool.len() < 8 {
            pool.push(inst);
            return;
        }
    }
    drop(inst);
}

#[derive(Debug, Error)]
pub enum ExecError {
    #[error("{0}")]
    Msg(String),
    #[error("worker timeout rpc={rpc} elapsed_ms={elapsed_ms}")]
    Timeout { rpc: String, elapsed_ms: u128 },
    #[error("worker dead rpc={rpc} exit={exit:?}")]
    Dead { rpc: String, exit: Option<i32> },
}

impl ExecError {
    pub fn infra_detail(&self) -> String {
        self.to_string()
    }
}

impl From<String> for ExecError {
    fn from(s: String) -> Self {
        Self::Msg(s)
    }
}

pub fn require_mujoco_env() -> bool {
    matches!(
        std::env::var("REALITYOS_REQUIRE_MUJOCO")
            .unwrap_or_default()
            .as_str(),
        "1" | "true" | "TRUE" | "yes"
    )
}

pub fn mujoco_available() -> bool {
    Command::new("python")
        .args(["-c", "import mujoco"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

pub fn ensure_mujoco_or_skip() -> bool {
    if mujoco_available() {
        return true;
    }
    if require_mujoco_env() {
        panic!("REALITYOS_REQUIRE_MUJOCO=1 but mujoco_available()==false");
    }
    false
}

pub fn worker_script_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("mujoco_worker.py")
}

fn rpc_timeout() -> Duration {
    let ms = std::env::var("REALITYOS_RPC_TIMEOUT_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(90_000u64);
    Duration::from_millis(ms.max(50))
}

pub fn episode_timeout() -> Duration {
    let ms = std::env::var("REALITYOS_EPISODE_TIMEOUT_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(120_000u64);
    Duration::from_millis(ms.max(100))
}

pub struct MujocoInstance {
    child: Child,
    stdin: Option<ChildStdin>,
    stdout: Option<BufReader<std::process::ChildStdout>>,
    pub inspect: Value,
    pub mujoco_version: String,
    pub last_rpc: String,
    pub last_exit: Option<i32>,
    pub compile_count: u32,
}

impl Drop for MujocoInstance {
    fn drop(&mut self) {
        if self.alive() {
            let _ = self.rpc(&json!({"cmd":"close"}));
        }
        self.kill_and_reap();
    }
}

impl MujocoInstance {
    pub fn spawn() -> Result<Self, ExecError> {
        if !mujoco_available() {
            return Err(ExecError::Msg("MUJOCO_UNAVAILABLE".into()));
        }
        let script = worker_script_path();
        if !script.exists() {
            return Err(ExecError::Msg(format!(
                "worker missing: {}",
                script.display()
            )));
        }
        let mut child = Command::new("python")
            .arg("-u")
            .arg(&script)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .env("PYTHONUNBUFFERED", "1")
            .env("MUJOCO_GL", "disable")
            .spawn()
            .map_err(|e| ExecError::Msg(e.to_string()))?;
        if let Some(stderr) = child.stderr.take() {
            thread::spawn(move || {
                let mut r = BufReader::new(stderr);
                let mut buf = String::new();
                while r.read_line(&mut buf).ok().unwrap_or(0) > 0 {
                    if buf.len() > 16_384 {
                        buf.clear();
                    }
                }
            });
        }
        let stdin = child
            .stdin
            .take()
            .ok_or_else(|| ExecError::Msg("stdin".into()))?;
        let stdout = child
            .stdout
            .take()
            .ok_or_else(|| ExecError::Msg("stdout".into()))?;
        let mut inst = Self {
            child,
            stdin: Some(stdin),
            stdout: Some(BufReader::new(stdout)),
            inspect: Value::Null,
            mujoco_version: String::new(),
            last_rpc: "spawn".into(),
            last_exit: None,
            compile_count: 0,
        };
        let hello = inst.rpc(&json!({"cmd":"hello"}))?;
        if hello["ok"] != true {
            return Err(ExecError::Msg(
                hello["error"].as_str().unwrap_or("hello").into(),
            ));
        }
        inst.mujoco_version = hello["mujoco_version"].as_str().unwrap_or("").into();
        let _ = WORKER_SEQ.fetch_add(1, Ordering::Relaxed);
        Ok(inst)
    }

    pub fn alive(&self) -> bool {
        self.stdin.is_some() && self.stdout.is_some()
    }

    pub fn kill_and_reap(&mut self) {
        let _ = self.child.kill();
        if let Ok(status) = self.child.wait() {
            self.last_exit = status.code();
        }
        self.stdin = None;
        self.stdout = None;
    }

    pub fn rpc(&mut self, msg: &Value) -> Result<Value, ExecError> {
        self.rpc_timeout(msg, rpc_timeout())
    }

    pub fn rpc_timeout(&mut self, msg: &Value, timeout: Duration) -> Result<Value, ExecError> {
        let cmd = msg
            .get("cmd")
            .and_then(|v| v.as_str())
            .unwrap_or("unknown")
            .to_string();
        self.last_rpc = cmd.clone();
        let Some(stdin) = self.stdin.as_mut() else {
            return Err(ExecError::Dead {
                rpc: cmd,
                exit: self.last_exit,
            });
        };
        let line = serde_json::to_string(msg).map_err(|e| ExecError::Msg(e.to_string()))?;
        if stdin
            .write_all(line.as_bytes())
            .and_then(|_| stdin.write_all(b"\n"))
            .and_then(|_| stdin.flush())
            .is_err()
        {
            self.kill_and_reap();
            return Err(ExecError::Dead {
                rpc: cmd,
                exit: self.last_exit,
            });
        }
        let Some(mut stdout) = self.stdout.take() else {
            return Err(ExecError::Dead {
                rpc: cmd.clone(),
                exit: self.last_exit,
            });
        };
        let (tx, rx) = mpsc::channel();
        thread::spawn(move || {
            let mut resp = String::new();
            let read = stdout.read_line(&mut resp);
            let _ = tx.send((stdout, read.map(|_| resp)));
        });
        match rx.recv_timeout(timeout) {
            Ok((stdout, Ok(resp))) => {
                self.stdout = Some(stdout);
                if resp.is_empty() {
                    self.kill_and_reap();
                    return Err(ExecError::Dead {
                        rpc: cmd,
                        exit: self.last_exit,
                    });
                }
                serde_json::from_str(resp.trim()).map_err(|e| ExecError::Msg(e.to_string()))
            }
            Ok((_, Err(e))) => {
                self.kill_and_reap();
                Err(ExecError::Msg(e.to_string()))
            }
            Err(_) => {
                self.kill_and_reap();
                Err(ExecError::Timeout {
                    rpc: cmd,
                    elapsed_ms: timeout.as_millis(),
                })
            }
        }
    }

    pub fn load_bundle(
        &mut self,
        bundle: &RobotBundle,
        objects: &[Value],
        seed: u64,
    ) -> Result<Value, ExecError> {
        match bundle.format.format {
            ModelFormat::Sdf => {
                return Err(ExecError::Msg("UNSUPPORTED_REQUIRES_CONVERSION".into()));
            }
            ModelFormat::Step => {
                return Err(ExecError::Msg(crate::format::step_diagnostic()));
            }
            ModelFormat::Usd => {
                return Err(ExecError::Msg(
                    "EXPERIMENTAL_UNSUPPORTED_IN_VERIFY_V1".into(),
                ));
            }
            ModelFormat::Unknown => {
                return Err(ExecError::Msg(bundle.format.detail.clone()));
            }
            _ => {}
        }
        let fmt = match bundle.format.format {
            ModelFormat::Urdf => "urdf",
            _ => "mjcf",
        };
        let asset_roots: Vec<String> = bundle
            .asset_roots_abs()
            .iter()
            .map(|p| p.to_string_lossy().into_owned())
            .collect();
        let resp = self.rpc(&json!({
            "cmd": "load",
            "format": fmt,
            "path": bundle.model_path.to_string_lossy(),
            "asset_roots": asset_roots,
            "objects": objects,
            "seed": seed,
        }))?;
        if resp["ok"] != true {
            return Err(ExecError::Msg(
                resp["error"].as_str().unwrap_or("load_failed").into(),
            ));
        }
        self.inspect = resp["inspect"].clone();
        self.compile_count = self.compile_count.saturating_add(1);
        Ok(resp)
    }

    pub fn local_linear(&mut self) -> Result<Value, ExecError> {
        let r = self.rpc(&json!({"cmd":"local_linear"}))?;
        if r["ok"] != true {
            return Err(ExecError::Msg(
                r["error"].as_str().unwrap_or("NOT_EVALUATED").into(),
            ));
        }
        Ok(r)
    }

    /// Privileged gravity generalized force after the prediction is frozen.
    /// Must not enter the pre-execution predictor.
    pub fn gravity_oracle(&mut self, qpos: Option<&[f64]>) -> Result<Value, ExecError> {
        let mut msg = json!({"cmd": "gravity_oracle"});
        if let Some(q) = qpos {
            msg["qpos"] = json!(q);
        }
        let r = self.rpc(&msg)?;
        if r["ok"] != true {
            return Err(ExecError::Msg(
                r["error"].as_str().unwrap_or("gravity_oracle").into(),
            ));
        }
        Ok(r)
    }

    pub fn peek_ctrl(&mut self) -> Result<(Vec<f64>, u64), ExecError> {
        let r = self.rpc(&json!({"cmd":"peek_ctrl"}))?;
        let ctrl = json_f64_vec(&r["ctrl"]);
        let n = r["ctrl_write_count"].as_u64().unwrap_or(0);
        Ok((ctrl, n))
    }

    pub fn set_ctrl(&mut self, ctrl: &[f64]) -> Result<Value, ExecError> {
        self.rpc(&json!({"cmd":"set_ctrl","ctrl": ctrl}))
    }

    pub fn set_safe_ctrl(&mut self, ctrl: &[f64]) -> Result<Value, ExecError> {
        self.rpc(&json!({"cmd":"set_safe_ctrl","ctrl": ctrl}))
    }

    pub fn step(&mut self, n: u32) -> Result<Value, ExecError> {
        self.rpc(&json!({"cmd":"step","n": n}))
    }

    pub fn configure_body(
        &mut self,
        body: &str,
        pos: Option<[f64; 3]>,
        mass: Option<f64>,
        friction: Option<f64>,
        size: Option<Vec<f64>>,
        hide: bool,
    ) -> Result<Value, ExecError> {
        let mut msg = json!({"cmd": "configure_body", "body": body, "hide": hide});
        if let Some(p) = pos {
            msg["pos"] = json!(p);
        }
        if let Some(m) = mass {
            msg["mass"] = json!(m);
        }
        if let Some(f) = friction {
            msg["friction"] = json!(f);
        }
        if let Some(s) = size {
            msg["size"] = json!(s);
        }
        let r = self.rpc(&msg)?;
        if r["ok"] != true {
            return Err(ExecError::Msg(
                r["error"].as_str().unwrap_or("configure_body").into(),
            ));
        }
        Ok(r)
    }

    pub fn set_body_pos(&mut self, body: &str, pos: [f64; 3]) -> Result<Value, ExecError> {
        let r = self.rpc(&json!({"cmd":"set_body_pos","body": body, "pos": pos}))?;
        if r["ok"] != true {
            return Err(ExecError::Msg(
                r["error"].as_str().unwrap_or("set_body_pos").into(),
            ));
        }
        Ok(r)
    }

    pub fn reset(
        &mut self,
        qpos: Option<&[f64]>,
        qvel: Option<&[f64]>,
    ) -> Result<Value, ExecError> {
        self.reset_ex(qpos, qvel, None)
    }

    pub fn reset_keyframe(&mut self, keyframe: i32) -> Result<Value, ExecError> {
        self.reset_ex(None, None, Some(keyframe))
    }

    fn reset_ex(
        &mut self,
        qpos: Option<&[f64]>,
        qvel: Option<&[f64]>,
        keyframe: Option<i32>,
    ) -> Result<Value, ExecError> {
        let mut msg = json!({"cmd":"reset"});
        if let Some(k) = keyframe {
            msg["keyframe"] = json!(k);
        }
        if let Some(q) = qpos {
            msg["qpos"] = json!(q);
        }
        if let Some(v) = qvel {
            msg["qvel"] = json!(v);
        }
        self.rpc(&msg)
    }

    pub fn state(&mut self) -> Result<Value, ExecError> {
        self.step(0)
    }

    pub fn apply_force(&mut self, body: &str, force: [f64; 3]) -> Result<(), ExecError> {
        let r = self.rpc(&json!({"cmd":"apply_xfrc","body": body, "force": force}))?;
        if r["ok"] != true {
            return Err(ExecError::Msg(r["error"].as_str().unwrap_or("xfrc").into()));
        }
        Ok(())
    }

    pub fn clear_forces(&mut self) -> Result<(), ExecError> {
        let _ = self.rpc(&json!({"cmd":"clear_xfrc"}))?;
        Ok(())
    }

    pub fn passive_rollout(&mut self, n: u32) -> Result<Value, ExecError> {
        self.rpc(&json!({"cmd":"passive_rollout","n": n}))
    }

    pub fn solve_ik(&mut self, site: &str, target: [f64; 3]) -> Result<(Vec<f64>, f64), ExecError> {
        let r = self.rpc(&json!({
            "cmd": "solve_ik",
            "site": site,
            "target": target,
            "iters": 40
        }))?;
        if r["ok"] != true {
            return Err(ExecError::Msg(
                r["error"].as_str().unwrap_or("solve_ik").into(),
            ));
        }
        Ok((json_f64_vec(&r["qpos"]), r["error"].as_f64().unwrap_or(1.0)))
    }

    pub fn inject_nan(&mut self) -> Result<Value, ExecError> {
        self.rpc(&json!({"cmd":"inject_nan"}))
    }
}

pub fn json_f64_vec(v: &Value) -> Vec<f64> {
    v.as_array()
        .map(|a| a.iter().filter_map(|x| x.as_f64()).collect())
        .unwrap_or_default()
}

pub fn wait_brief() {
    std::thread::sleep(Duration::from_millis(1));
}

#[allow(dead_code)]
fn _read_discard(mut r: impl Read) {
    let _ = r.read(&mut [0u8; 1]);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_reports_simulation_only() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let mut w = MujocoInstance::spawn().unwrap();
        let h = w.rpc(&json!({"cmd":"hello"})).unwrap();
        assert_eq!(h["evidence_status"], crate::honesty::SIMULATION_ONLY);
        assert_eq!(h["metal"], false);
        assert!(h["mujoco_version"].as_str().unwrap().starts_with('3'));
    }

    #[test]
    fn hang_rpc_times_out_and_is_reaped() {
        if !ensure_mujoco_or_skip() {
            return;
        }
        let mut w = MujocoInstance::spawn().unwrap();
        let err = w
            .rpc_timeout(
                &json!({"cmd":"hang","seconds": 30}),
                Duration::from_millis(400),
            )
            .unwrap_err();
        match err {
            ExecError::Timeout { rpc, .. } => assert_eq!(rpc, "hang"),
            other => panic!("{other}"),
        }
        assert!(!w.alive());
    }
}
