//! Process-isolated MuJoCo worker client. Isolated mjModel/mjData per instance.

use crate::bundle::RobotBundle;
use crate::format::{FormatDisposition, ModelFormat};
use serde_json::{json, Value};
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::Duration;
use thiserror::Error;

static WORKER_SEQ: AtomicU64 = AtomicU64::new(1);
static WORKER_POOL: Mutex<Vec<MujocoInstance>> = Mutex::new(Vec::new());

/// Recycle an isolated worker process. Each checkout still has its own mjModel/mjData after load.
pub fn checkout_worker() -> Result<MujocoInstance, ExecError> {
    if let Ok(mut pool) = WORKER_POOL.lock() {
        if let Some(w) = pool.pop() {
            return Ok(w);
        }
    }
    MujocoInstance::spawn()
}

pub fn checkin_worker(inst: MujocoInstance) {
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
}

impl From<String> for ExecError {
    fn from(s: String) -> Self {
        Self::Msg(s)
    }
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

pub fn worker_script_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("mujoco_worker.py")
}

pub struct MujocoInstance {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<std::process::ChildStdout>,
    pub inspect: Value,
    pub mujoco_version: String,
}

impl Drop for MujocoInstance {
    fn drop(&mut self) {
        let _ = self.rpc(&json!({"cmd":"close"}));
        let _ = self.child.kill();
        let _ = self.child.wait();
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
            stdin,
            stdout: BufReader::new(stdout),
            inspect: Value::Null,
            mujoco_version: String::new(),
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

    pub fn rpc(&mut self, msg: &Value) -> Result<Value, ExecError> {
        let line = serde_json::to_string(msg).map_err(|e| ExecError::Msg(e.to_string()))?;
        self.stdin
            .write_all(line.as_bytes())
            .and_then(|_| self.stdin.write_all(b"\n"))
            .and_then(|_| self.stdin.flush())
            .map_err(|e| ExecError::Msg(e.to_string()))?;
        let mut resp = String::new();
        self.stdout
            .read_line(&mut resp)
            .map_err(|e| ExecError::Msg(e.to_string()))?;
        if resp.is_empty() {
            return Err(ExecError::Msg("worker_eof".into()));
        }
        serde_json::from_str(resp.trim()).map_err(|e| ExecError::Msg(e.to_string()))
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
                if bundle.format.disposition == FormatDisposition::FeatureGatedExperimental {
                    let probe = self.rpc(&json!({"cmd":"hello"}))?;
                    if probe["usd_supported"] != true {
                        return Err(ExecError::Msg("UNSUPPORTED_EXPERIMENTAL_USD".into()));
                    }
                }
            }
            ModelFormat::Unknown => {
                return Err(ExecError::Msg(bundle.format.detail.clone()));
            }
            _ => {}
        }
        let fmt = match bundle.format.format {
            ModelFormat::Urdf => "urdf",
            ModelFormat::Usd => "usd",
            _ => "mjcf",
        };
        let resp = self.rpc(&json!({
            "cmd": "load",
            "format": fmt,
            "xml": bundle.model_text,
            "path": bundle.model_path.to_string_lossy(),
            "objects": objects,
            "seed": seed,
        }))?;
        if resp["ok"] != true {
            return Err(ExecError::Msg(
                resp["error"].as_str().unwrap_or("load_failed").into(),
            ));
        }
        self.inspect = resp["inspect"].clone();
        Ok(resp)
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

    pub fn step(&mut self, n: u32) -> Result<Value, ExecError> {
        self.rpc(&json!({"cmd":"step","n": n}))
    }

    pub fn reset(
        &mut self,
        qpos: Option<&[f64]>,
        qvel: Option<&[f64]>,
    ) -> Result<Value, ExecError> {
        let mut msg = json!({"cmd":"reset"});
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_reports_simulation_only() {
        if !mujoco_available() {
            return;
        }
        let mut w = MujocoInstance::spawn().unwrap();
        let h = w.rpc(&json!({"cmd":"hello"})).unwrap();
        assert_eq!(h["evidence_status"], crate::honesty::SIMULATION_ONLY);
        assert_eq!(h["metal"], false);
        assert!(h["mujoco_version"].as_str().unwrap().starts_with('3'));
    }
}
