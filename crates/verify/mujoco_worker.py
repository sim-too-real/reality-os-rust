#!/usr/bin/env python3
"""Isolated MuJoCo execution worker.

SIMULATION_ONLY. metal=false. This process never emits realityos.metal_proof.
It does not certify real-world safety. SIM != METAL.
"""

from __future__ import annotations

import json
import math
import os
import sys
import traceback
from typing import Any

os.environ.setdefault("MUJOCO_GL", "disable")

EVIDENCE_STATUS = "SIMULATION_ONLY"
SCHEMA = "realityos.simulation_plant/1"

try:
    import mujoco
except Exception as exc:  # pragma: no cover
    print(json.dumps({"ok": False, "error": f"MUJOCO_UNAVAILABLE:{exc}"}), flush=True)
    sys.exit(2)


def _finite(xs) -> bool:
    return all(math.isfinite(float(x)) for x in xs)


def _jf(x: Any) -> float | None:
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _jlist(xs) -> list[float | None]:
    return [_jf(x) for x in xs]


def _sanitize(obj: Any) -> Any:
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_sanitize(v) for v in obj]
    return obj


class Instance:
    def __init__(self) -> None:
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self.ctrl_write_count = 0
        self.last_ctrl: list[float] = []
        self.warnings: list[str] = []
        self.source_format = ""
        self.lost_features: list[str] = []
        self.usd_experimental = False

    def inspect(self) -> dict[str, Any]:
        m, d = self.model, self.data
        assert m is not None and d is not None
        joints = []
        for i in range(m.njnt):
            j = m.joint(i)
            parent = int(m.jnt_bodyid[i])
            child = parent
            joints.append(
                {
                    "name": j.name or f"joint_{i}",
                    "type": int(m.jnt_type[i]),
                    "qposadr": int(m.jnt_qposadr[i]),
                    "dofadr": int(m.jnt_dofadr[i]),
                    "range": [float(m.jnt_range[i][0]), float(m.jnt_range[i][1])],
                    "limited": bool(m.jnt_limited[i]),
                    "parent_body": m.body(parent).name or f"body_{parent}",
                    "child_body": m.body(child).name or f"body_{child}",
                }
            )
        actuators = []
        for i in range(m.nu):
            a = m.actuator(i)
            trn = int(m.actuator_trntype[i])
            target_id = int(m.actuator_trnid[i][0])
            target = ""
            if trn == 0 and 0 <= target_id < m.njnt:  # joint
                target = m.joint(target_id).name
            elif 0 <= target_id < m.nbody:
                target = m.body(target_id).name
            actuators.append(
                {
                    "name": a.name or f"actuator_{i}",
                    "trntype": trn,
                    "target": target,
                    "target_id": target_id,
                    "ctrlrange": [float(m.actuator_ctrlrange[i][0]), float(m.actuator_ctrlrange[i][1])],
                    "ctrllimited": bool(m.actuator_ctrllimited[i]),
                    "forcerange": [float(m.actuator_forcerange[i][0]), float(m.actuator_forcerange[i][1])],
                    "forcelimited": bool(m.actuator_forcelimited[i]),
                    "gaintype": int(m.actuator_gaintype[i]),
                    "biastype": int(m.actuator_biastype[i]),
                }
            )
        sensors = []
        for i in range(m.nsensor):
            s = m.sensor(i)
            sensors.append(
                {
                    "name": s.name or f"sensor_{i}",
                    "type": int(m.sensor_type[i]),
                    "dim": int(m.sensor_dim[i]),
                    "adr": int(m.sensor_adr[i]),
                }
            )
        cameras = []
        for i in range(m.ncam):
            c = m.camera(i)
            cameras.append(
                {
                    "name": c.name or f"camera_{i}",
                    "parent_body": m.body(int(m.cam_bodyid[i])).name,
                }
            )
        bodies = []
        for i in range(m.nbody):
            b = m.body(i)
            parent = int(m.body_parentid[i])
            bodies.append(
                {
                    "name": b.name or f"body_{i}",
                    "mass": float(m.body_mass[i]),
                    "inertia": [float(x) for x in m.body_inertia[i]],
                    "parent": m.body(parent).name if parent >= 0 else "",
                    "parent_id": parent,
                    "ipos": [float(x) for x in m.body_ipos[i]],
                }
            )
        sites = []
        for i in range(m.nsite):
            sites.append({"name": m.site(i).name or f"site_{i}", "body": m.body(int(m.site_bodyid[i])).name})
        return {
            "nq": int(m.nq),
            "nv": int(m.nv),
            "nu": int(m.nu),
            "nbody": int(m.nbody),
            "njnt": int(m.njnt),
            "nactuator": int(m.nu),
            "nsensor": int(m.nsensor),
            "ncam": int(m.ncam),
            "nsite": int(m.nsite),
            "timestep": float(m.opt.timestep),
            "gravity": [float(x) for x in m.opt.gravity],
            "joints": joints,
            "actuators": actuators,
            "sensors": sensors,
            "cameras": cameras,
            "bodies": bodies,
            "sites": sites,
            "mujoco_version": mujoco.__version__,
            "source_format": self.source_format,
            "lost_features": self.lost_features,
            "usd_experimental": self.usd_experimental,
            "warnings": list(self.warnings),
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
        }

    def state(self, extra_names: list[str] | None = None) -> dict[str, Any]:
        m, d = self.model, self.data
        assert m is not None and d is not None
        xpos = {}
        for i in range(m.nbody):
            xpos[m.body(i).name or f"body_{i}"] = _jlist(d.xpos[i])
        sites = {}
        for i in range(m.nsite):
            sites[m.site(i).name or f"site_{i}"] = _jlist(d.site_xpos[i])
        contacts = []
        for i in range(d.ncon):
            c = d.contact[i]
            contacts.append(
                {
                    "geom1": int(c.geom1),
                    "geom2": int(c.geom2),
                    "dist": float(c.dist),
                    "pos": [float(x) for x in c.pos],
                    "body1": m.body(int(m.geom_bodyid[c.geom1])).name if c.geom1 >= 0 else "",
                    "body2": m.body(int(m.geom_bodyid[c.geom2])).name if c.geom2 >= 0 else "",
                }
            )
        cfrc = []
        if d.ncon:
            import numpy as np

            force = np.zeros(6, dtype=np.float64)
            for i in range(d.ncon):
                try:
                    mujoco.mj_contactForce(m, d, i, force)
                    cfrc.append(float(math.sqrt(force[0] ** 2 + force[1] ** 2 + force[2] ** 2)))
                except Exception:
                    cfrc.append(0.0)
        qacc = _jlist(d.qacc)
        nan = (not _finite(d.qpos)) or (not _finite(d.qvel)) or any(x is None for x in qacc)
        ke = _jf(d.energy[1]) if len(d.energy) > 1 else 0.0
        out = {
            "time": _jf(d.time) or 0.0,
            "qpos": _jlist(d.qpos),
            "qvel": _jlist(d.qvel),
            "qacc": qacc,
            "ctrl": _jlist(d.ctrl),
            "actuator_force": _jlist(d.actuator_force),
            "sensordata": _jlist(d.sensordata),
            "xpos": xpos,
            "sites": sites,
            "contacts": contacts,
            "contact_forces": cfrc,
            "ncon": int(d.ncon),
            "energy": [float(x) for x in d.energy],
            "kinetic_energy": ke,
            "subtree_com": [float(x) for x in d.subtree_com[1]] if m.nbody > 1 else [0.0, 0.0, 0.0],
            "nan": nan,
            "ctrl_write_count": self.ctrl_write_count,
            "last_ctrl": list(self.last_ctrl),
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
        }
        if extra_names:
            named = {}
            for name in extra_names:
                if name in xpos:
                    named[name] = xpos[name]
                elif name in sites:
                    named[name] = sites[name]
            out["named_pos"] = named
        return out


INST = Instance()


def _scan_urdf_losses(text: str) -> list[str]:
    lost = []
    low = text.lower()
    for token, label in (
        ("<mimic", "urdf_mimic_not_compiled_as_constraint"),
        ("<gazebo", "gazebo_extensions_ignored"),
        ("<transmission", "urdf_transmission_reduced_to_mujoco_actuator"),
        ("type=\"planar\"", "planar_joint_unsupported_or_reduced"),
        ("type=\"floating\"", "floating_joint_may_become_freejoint"),
        ("<calibration", "urdf_calibration_ignored"),
    ):
        if token in low:
            lost.append(label)
    return lost


def _compose_xml(base_xml: str, extras: list[dict[str, Any]]) -> str:
    chunks = []
    for obj in extras:
        kind = obj.get("type", "box")
        name = obj["name"]
        pos = obj.get("pos", [0, 0, 0.05])
        size = obj.get("size", [0.03, 0.03, 0.03])
        rgba = obj.get("rgba", [0.8, 0.2, 0.2, 1])
        mass = float(obj.get("mass", 0.05))
        movable = bool(obj.get("movable", True))
        free = "<freejoint/>" if movable else ""
        if kind == "sphere":
            geom = f'<geom type="sphere" size="{size[0]}" rgba="{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}" mass="{mass}"/>'
        elif kind == "capsule":
            geom = f'<geom type="capsule" size="{size[0]} {size[1] if len(size) > 1 else 0.1}" rgba="{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}" mass="{mass}"/>'
        elif kind == "plane":
            geom = f'<geom type="plane" size="{size[0]} {size[1] if len(size) > 1 else 1} 0.01" rgba="{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"/>'
            free = ""
        else:
            geom = f'<geom type="box" size="{size[0]} {size[1]} {size[2]}" rgba="{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}" mass="{mass}"/>'
        friction = obj.get("friction")
        if friction is not None:
            geom = geom[:-2] + f' friction="{friction} {friction} 0.01"/>'
        chunks.append(
            f'<body name="{name}" pos="{pos[0]} {pos[1]} {pos[2]}">{free}{geom}</body>'
        )
    block = "\n".join(chunks)
    if "</worldbody>" not in base_xml:
        raise ValueError("robot xml missing </worldbody>")
    return base_xml.replace("</worldbody>", block + "\n</worldbody>", 1)


def _load(msg: dict[str, Any]) -> dict[str, Any]:
    INST.model = None
    INST.data = None
    INST.ctrl_write_count = 0
    INST.last_ctrl = []
    INST.warnings = []
    INST.lost_features = []
    INST.usd_experimental = False
    fmt = str(msg.get("format", "mjcf")).lower()
    INST.source_format = fmt
    xml = msg.get("xml")
    path = msg.get("path")
    extras = msg.get("objects") or []
    if fmt in {"usd", "usda", "usdc", "usdz"}:
        INST.usd_experimental = True
        if not hasattr(mujoco, "usd"):
            return {
                "ok": False,
                "error": "UNSUPPORTED_EXPERIMENTAL_USD",
                "detail": "installed MuJoCo build does not expose mujoco.usd",
                "evidence_status": EVIDENCE_STATUS,
                "metal": False,
            }
        try:
            load_usd = getattr(mujoco.usd, "load", None) or getattr(mujoco, "MjModel")
            if path:
                INST.model = mujoco.MjModel.from_xml_path(path)
            else:
                return {"ok": False, "error": "UNSUPPORTED_EXPERIMENTAL_USD", "detail": "USD bytes path required"}
        except Exception as exc:
            return {
                "ok": False,
                "error": "UNSUPPORTED_EXPERIMENTAL_USD",
                "detail": str(exc),
                "evidence_status": EVIDENCE_STATUS,
                "metal": False,
            }
    elif fmt == "urdf" or (fmt == "xml" and xml and "<robot" in xml):
        text = xml
        if path and not text:
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()
        INST.lost_features = _scan_urdf_losses(text or "")
        if extras:
            text = _compose_xml(text, extras)
        INST.model = mujoco.MjModel.from_xml_string(text)
        INST.source_format = "urdf"
    elif fmt in {"mjcf", "xml"}:
        text = xml
        if path and not text:
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()
        if extras:
            text = _compose_xml(text, extras)
        INST.model = mujoco.MjModel.from_xml_string(text)
        INST.source_format = "mjcf"
    else:
        return {"ok": False, "error": f"UNSUPPORTED_FORMAT:{fmt}", "metal": False, "evidence_status": EVIDENCE_STATUS}

    INST.data = mujoco.MjData(INST.model)
    seed = int(msg.get("seed", 0))
    INST.data.qpos[:] = INST.model.qpos0
    INST.data.qvel[:] = 0
    mujoco.mj_forward(INST.model, INST.data)
    INST.last_ctrl = [float(x) for x in INST.data.ctrl]
    warning_bits = []
    for i in range(len(INST.data.warning)):
        if INST.data.warning[i].number:
            warning_bits.append(f"mj_warning_{i}:{int(INST.data.warning[i].number)}")
    INST.warnings = warning_bits
    inspect = INST.inspect()
    inspect["load_seed"] = seed
    return {"ok": True, "inspect": inspect, "state": INST.state(), "metal": False, "evidence_status": EVIDENCE_STATUS}


def _set_ctrl(ctrl: list[float], write: bool) -> None:
    assert INST.model is not None and INST.data is not None
    if len(ctrl) != INST.model.nu:
        raise ValueError(f"ctrl_dim_mismatch:{len(ctrl)}!={INST.model.nu}")
    if not _finite(ctrl):
        raise ValueError("non_finite_ctrl")
    INST.data.ctrl[:] = ctrl
    if write:
        INST.ctrl_write_count += 1
        INST.last_ctrl = [float(x) for x in ctrl]


def handle(msg: dict[str, Any]) -> dict[str, Any]:
    cmd = msg.get("cmd")
    if cmd == "hello":
        usd = hasattr(mujoco, "usd")
        return {
            "ok": True,
            "mujoco_version": mujoco.__version__,
            "usd_supported": usd,
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
            "schema": SCHEMA,
        }
    if cmd == "load":
        return _load(msg)
    if INST.model is None or INST.data is None:
        return {"ok": False, "error": "not_loaded"}
    if cmd == "inspect":
        return {"ok": True, "inspect": INST.inspect()}
    if cmd == "reset":
        mujoco.mj_resetData(INST.model, INST.data)
        if "qpos" in msg:
            q = msg["qpos"]
            if len(q) != INST.model.nq:
                return {"ok": False, "error": f"qpos_dim:{len(q)}!={INST.model.nq}"}
            INST.data.qpos[:] = q
        if "qvel" in msg:
            v = msg["qvel"]
            if len(v) != INST.model.nv:
                return {"ok": False, "error": f"qvel_dim:{len(v)}!={INST.model.nv}"}
            INST.data.qvel[:] = v
        mujoco.mj_forward(INST.model, INST.data)
        return {"ok": True, "state": INST.state()}
    if cmd == "peek_ctrl":
        return {
            "ok": True,
            "ctrl": [float(x) for x in INST.data.ctrl],
            "ctrl_write_count": INST.ctrl_write_count,
            "last_ctrl": list(INST.last_ctrl),
        }
    if cmd == "set_ctrl":
        _set_ctrl(list(msg["ctrl"]), write=True)
        return {"ok": True, "ctrl_write_count": INST.ctrl_write_count, "ctrl": [float(x) for x in INST.data.ctrl]}
    if cmd == "step":
        n = int(msg.get("n", 1))
        max_force = 0.0
        max_speed = 0.0
        nan = False
        for _ in range(max(n, 0)):
            mujoco.mj_step(INST.model, INST.data)
            if not _finite(INST.data.qpos) or not _finite(INST.data.qvel):
                nan = True
                break
            max_speed = max(max_speed, max((abs(float(v)) for v in INST.data.qvel), default=0.0))
            if len(INST.data.actuator_force):
                max_force = max(max_force, max((abs(float(f)) for f in INST.data.actuator_force), default=0.0))
        st = INST.state()
        st["interval_max_speed"] = max_speed
        st["interval_max_force"] = max_force
        st["nan"] = st["nan"] or nan
        return {"ok": not st["nan"], "state": st, "divergent": nan}
    if cmd == "apply_xfrc":
        body = msg["body"]
        force = msg.get("force", [0, 0, 0])
        found = None
        for i in range(INST.model.nbody):
            if INST.model.body(i).name == body:
                found = i
                break
        if found is None:
            return {"ok": False, "error": f"unknown_body:{body}"}
        INST.data.xfrc_applied[found][:3] = force
        return {"ok": True}
    if cmd == "clear_xfrc":
        INST.data.xfrc_applied[:] = 0
        return {"ok": True}
    if cmd == "passive_rollout":
        n = int(msg.get("n", 50))
        INST.data.ctrl[:] = 0
        peak = 0.0
        nan = False
        for _ in range(n):
            mujoco.mj_step(INST.model, INST.data)
            peak = max(peak, max((abs(float(v)) for v in INST.data.qvel), default=0.0))
            if not _finite(INST.data.qpos) or not _finite(INST.data.qvel):
                nan = True
                break
        return {"ok": not nan, "peak_speed": peak, "nan": nan, "state": INST.state()}
    if cmd == "render":
        try:
            w = int(msg.get("width", 64))
            h = int(msg.get("height", 64))
            renderer = mujoco.Renderer(INST.model, height=h, width=w)
            cam = msg.get("camera") or ""
            if cam:
                renderer.update_scene(INST.data, camera=cam)
            else:
                renderer.update_scene(INST.data)
            pixels = renderer.render()
            renderer.close()
            return {
                "ok": True,
                "width": w,
                "height": h,
                "channels": int(pixels.shape[-1]),
                "pixels": pixels.flatten().astype(int).tolist(),
                "metal": False,
            }
        except Exception as exc:
            return {"ok": False, "error": f"RENDER_UNAVAILABLE:{exc}"}
    if cmd == "solve_ik":
        import numpy as np

        site = str(msg.get("site") or "ee")
        target = np.array(msg.get("target") or [0.0, 0.0, 0.0], dtype=float)
        sid = None
        for i in range(INST.model.nsite):
            if (INST.model.site(i).name or f"site_{i}") == site:
                sid = i
                break
        if sid is None and INST.model.nsite > 0:
            sid = 0
        if sid is None:
            return {"ok": False, "error": "no_site"}
        jacp = np.zeros((3, INST.model.nv))
        jacr = np.zeros((3, INST.model.nv))
        damping = 1e-3
        q0 = np.array(INST.data.qpos, copy=True)
        err_norm = 1e9
        for _ in range(int(msg.get("iters", 40))):
            mujoco.mj_forward(INST.model, INST.data)
            mujoco.mj_jacSite(INST.model, INST.data, jacp, jacr, sid)
            err = target - np.array(INST.data.site_xpos[sid])
            err_norm = float(np.linalg.norm(err))
            if err_norm < 1e-3:
                break
            jjt = jacp @ jacp.T + damping * np.eye(3)
            dq = jacp.T @ np.linalg.solve(jjt, err)
            for j in range(INST.model.njnt):
                adr = int(INST.model.jnt_qposadr[j])
                dof = int(INST.model.jnt_dofadr[j])
                jtype = int(INST.model.jnt_type[j])
                # Cartesian IK uses hinge DoFs. Gripper slides stay put.
                if jtype == 3 and 0 <= dof < INST.model.nv:
                    INST.data.qpos[adr] = float(INST.data.qpos[adr] + dq[dof])
                    if INST.model.jnt_limited[j]:
                        lo, hi = INST.model.jnt_range[j]
                        INST.data.qpos[adr] = min(max(float(INST.data.qpos[adr]), float(lo)), float(hi))
        qpos = [float(x) for x in INST.data.qpos]
        INST.data.qpos[:] = q0
        mujoco.mj_forward(INST.model, INST.data)
        return {
            "ok": True,
            "qpos": qpos,
            "error": err_norm,
            "site": site,
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
        }
    if cmd == "inject_nan":
        if INST.model.nq > 0:
            INST.data.qpos[0] = float("nan")
        # Do not call mj_forward/state() here: MuJoCo may write non-JSON to stdout.
        return {
            "ok": True,
            "nan": True,
            "state": {
                "time": 0.0,
                "qpos": [None] + [0.0] * max(INST.model.nq - 1, 0),
                "qvel": [0.0] * INST.model.nv,
                "qacc": [None] * INST.model.nv,
                "ctrl": [0.0] * INST.model.nu,
                "actuator_force": [],
                "nan": True,
                "ncon": 0,
                "contacts": [],
                "xpos": {},
                "sites": {},
                "kinetic_energy": 0.0,
                "subtree_com": [0.0, 0.0, 0.0],
            },
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
        }
    if cmd == "close":
        INST.model = None
        INST.data = None
        return {"ok": True}
    return {"ok": False, "error": f"unknown_cmd:{cmd}"}


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stdin.reconfigure(encoding="utf-8")
    for raw in sys.stdin:
        line = raw.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
            out = handle(msg)
        except Exception as exc:
            out = {
                "ok": False,
                "error": str(exc),
                "trace": traceback.format_exc(limit=4),
                "metal": False,
                "evidence_status": EVIDENCE_STATUS,
            }
        sys.stdout.write(json.dumps(_sanitize(out), ensure_ascii=True, allow_nan=False) + "\n")
        sys.stdout.flush()


if __name__ == "__main__":
    main()
