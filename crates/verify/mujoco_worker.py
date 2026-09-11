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
import time
import traceback
from typing import Any

os.environ.setdefault("MUJOCO_GL", "disable")

EVIDENCE_STATUS = "SIMULATION_ONLY"
SCHEMA = "realityos.simulation_plant/1"

# Drain/redirect native logs so they cannot fill a stderr pipe.
os.environ.setdefault("MUJOCO_LOG_FILE", os.devnull)

try:
    import mujoco
    import numpy as np
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


def _joint_dims(jtype: int) -> tuple[int, int]:
    # mjJNT_FREE, BALL, SLIDE, HINGE
    if jtype == 0:
        return 7, 6
    if jtype == 1:
        return 4, 3
    return 1, 1


class Instance:
    def __init__(self) -> None:
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self.ctrl_write_count = 0
        self.safe_write_count = 0
        self.last_ctrl: list[float] = []
        self.warnings: list[str] = []
        self.source_format = ""
        self.lost_features: list[str] = []
        self.usd_experimental = False
        self.compiled_bytes_hash = ""
        self.asset_names: list[str] = []

    def inspect(self) -> dict[str, Any]:
        m, d = self.model, self.data
        assert m is not None and d is not None
        joints = []
        for i in range(m.njnt):
            j = m.joint(i)
            child_id = int(m.jnt_bodyid[i])
            parent_id = int(m.body_parentid[child_id])
            jtype = int(m.jnt_type[i])
            qdim, ddim = _joint_dims(jtype)
            unsupported = None
            if jtype == 0:
                unsupported = "free_joint_multi_dof"
            elif jtype == 1:
                unsupported = "ball_joint_multi_dof"
            joints.append(
                {
                    "name": j.name or f"joint_{i}",
                    "type": jtype,
                    "qposadr": int(m.jnt_qposadr[i]),
                    "dofadr": int(m.jnt_dofadr[i]),
                    "qpos_dim": qdim,
                    "dof_dim": ddim,
                    "range": [float(m.jnt_range[i][0]), float(m.jnt_range[i][1])],
                    "limited": bool(m.jnt_limited[i]),
                    "axis": [float(x) for x in m.jnt_axis[i]],
                    "pos": [float(x) for x in m.jnt_pos[i]],
                    "parent_body": m.body(parent_id).name or f"body_{parent_id}",
                    "child_body": m.body(child_id).name or f"body_{child_id}",
                    "parent_id": parent_id,
                    "child_id": child_id,
                    "unsupported_reason": unsupported,
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
                    "pos": [float(x) for x in m.cam_pos[i]],
                    "quat": [float(x) for x in m.cam_quat[i]],
                }
            )
        bodies = []
        body_parent = {}
        for i in range(m.nbody):
            b = m.body(i)
            parent = int(m.body_parentid[i])
            body_parent[b.name or f"body_{i}"] = m.body(parent).name if parent >= 0 else ""
            bodies.append(
                {
                    "name": b.name or f"body_{i}",
                    "mass": float(m.body_mass[i]),
                    "inertia": [float(x) for x in m.body_inertia[i]],
                    "parent": m.body(parent).name if parent >= 0 else "",
                    "parent_id": parent,
                    "pos": [float(x) for x in m.body_pos[i]],
                    "quat": [float(x) for x in m.body_quat[i]],
                    "ipos": [float(x) for x in m.body_ipos[i]],
                }
            )
        sites = []
        for i in range(m.nsite):
            sites.append(
                {
                    "name": m.site(i).name or f"site_{i}",
                    "body": m.body(int(m.site_bodyid[i])).name,
                    "pos": [float(x) for x in m.site_pos[i]],
                    "quat": [float(x) for x in m.site_quat[i]],
                }
            )
        geoms = []
        for i in range(m.ngeom):
            geoms.append(
                {
                    "name": m.geom(i).name or f"geom_{i}",
                    "body": m.body(int(m.geom_bodyid[i])).name,
                    "group": int(m.geom_group[i]),
                    "contype": int(m.geom_contype[i]),
                    "conaffinity": int(m.geom_conaffinity[i]),
                }
            )
        ee_chains = _end_effector_chains(m, joints, bodies, sites)
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
            "geoms": geoms,
            "end_effector_chains": ee_chains,
            "self_collision_rule": "ignore_direct_kinematic_neighbors",
            "mujoco_version": mujoco.__version__,
            "source_format": self.source_format,
            "lost_features": self.lost_features,
            "usd_experimental": False,
            "asset_names": list(self.asset_names),
            "warnings": list(self.warnings),
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
        }

    def state(self, extra_names: list[str] | None = None) -> dict[str, Any]:
        m, d = self.model, self.data
        assert m is not None and d is not None
        xpos = {}
        xquat = {}
        for i in range(m.nbody):
            name = m.body(i).name or f"body_{i}"
            xpos[name] = _jlist(d.xpos[i])
            xquat[name] = _jlist(d.xquat[i])
        sites = {}
        site_xquat = {}
        for i in range(m.nsite):
            name = m.site(i).name or f"site_{i}"
            sites[name] = _jlist(d.site_xpos[i])
            q = np.zeros(4, dtype=float)
            mujoco.mju_mat2Quat(q, d.site_xmat[i])
            site_xquat[name] = [float(x) for x in q]
        contacts = []
        for i in range(d.ncon):
            c = d.contact[i]
            g1 = int(c.geom1)
            g2 = int(c.geom2)
            contacts.append(
                {
                    "geom1": g1,
                    "geom2": g2,
                    "dist": float(c.dist),
                    "pos": [float(x) for x in c.pos],
                    "body1": m.body(int(m.geom_bodyid[g1])).name if g1 >= 0 else "",
                    "body2": m.body(int(m.geom_bodyid[g2])).name if g2 >= 0 else "",
                    "group1": int(m.geom_group[g1]) if g1 >= 0 else -1,
                    "group2": int(m.geom_group[g2]) if g2 >= 0 else -1,
                }
            )
        cfrc = []
        if d.ncon:
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
            "xquat": xquat,
            "sites": sites,
            "site_xquat": site_xquat,
            "contacts": contacts,
            "contact_forces": cfrc,
            "ncon": int(d.ncon),
            "energy": [float(x) for x in d.energy],
            "kinetic_energy": ke,
            "subtree_com": [float(x) for x in d.subtree_com[1]] if m.nbody > 1 else [0.0, 0.0, 0.0],
            "nan": nan,
            "ctrl_write_count": self.ctrl_write_count,
            "safe_write_count": self.safe_write_count,
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


def _end_effector_chains(m, joints, bodies, sites) -> list[dict[str, Any]]:
    name_to_id = {m.body(i).name or f"body_{i}": i for i in range(m.nbody)}
    chains = []
    candidates = []
    for s in sites:
        if s["name"] in {"ee", "tip", "site_ee"}:
            candidates.append(s["body"])
    if not candidates:
        for b in reversed(bodies):
            if b["name"] not in {"world", "floor", "ground"}:
                candidates.append(b["name"])
                break
    seen = set()
    for body_name in candidates:
        if body_name in seen or body_name not in name_to_id:
            continue
        seen.add(body_name)
        bid = name_to_id[body_name]
        body_chain = []
        joint_chain = []
        i = bid
        while i > 0:
            body_chain.append(m.body(i).name or f"body_{i}")
            for j in joints:
                if j.get("child_id") == i:
                    joint_chain.append(j["name"])
            i = int(m.body_parentid[i])
        body_chain.reverse()
        joint_chain.reverse()
        chains.append({"bodies": body_chain, "joints": joint_chain, "end_effector": body_name})
    return chains


def _scan_urdf_losses(text: str) -> list[str]:
    lost = []
    low = text.lower()
    for token, label in (
        ("<mimic", "urdf_mimic_not_compiled_as_constraint"),
        ("<gazebo", "gazebo_extensions_ignored"),
        ("<transmission", "urdf_transmission_reduced_to_mujoco_actuator"),
        ('type="planar"', "planar_joint_unsupported_or_reduced"),
        ('type="floating"', "floating_joint_may_become_freejoint"),
        ("<calibration", "urdf_calibration_ignored"),
    ):
        if token in low:
            lost.append(label)
    return lost


def _collect_assets(roots: list[str]) -> dict[str, bytes]:
    assets: dict[str, bytes] = {}
    for root in roots:
        if not root:
            continue
        root_abs = os.path.abspath(root)
        if not os.path.isdir(root_abs):
            continue
        for dirpath, _, files in os.walk(root_abs):
            for fn in files:
                if fn in {"robot.yaml", "LICENSE"}:
                    continue
                full = os.path.join(dirpath, fn)
                rel = os.path.relpath(full, root_abs).replace("\\", "/")
                try:
                    with open(full, "rb") as fh:
                        data = fh.read()
                except OSError:
                    continue
                assets[rel] = data
                assets[fn] = data
    return assets


def _add_scenario_objects(spec: Any, extras: list[dict[str, Any]]) -> None:
    for obj in extras:
        kind = str(obj.get("type", "box"))
        name = str(obj["name"])
        pos = obj.get("pos", [0, 0, 0.05])
        size = obj.get("size", [0.03, 0.03, 0.03])
        rgba = obj.get("rgba", [0.8, 0.2, 0.2, 1])
        mass = float(obj.get("mass", 0.05))
        movable = bool(obj.get("movable", True))
        body = spec.worldbody.add_body(name=name, pos=pos)
        if movable and kind != "plane":
            body.add_freejoint()
        kwargs: dict[str, Any] = {"rgba": rgba, "mass": mass}
        if obj.get("friction") is not None:
            fr = float(obj["friction"])
            kwargs["friction"] = [fr, fr, 0.01]
        if kind == "sphere":
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[size[0], 0, 0], **kwargs)
        elif kind == "capsule":
            body.add_geom(
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[size[0], size[1] if len(size) > 1 else 0.1, 0],
                **kwargs,
            )
        elif kind == "plane":
            body.add_geom(
                type=mujoco.mjtGeom.mjGEOM_PLANE,
                size=[size[0], size[1] if len(size) > 1 else 1, 0.01],
                rgba=rgba,
            )
        else:
            sx, sy, sz = (list(size) + [0.03, 0.03, 0.03])[:3]
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[sx, sy, sz], **kwargs)


def _load_spec(path: str, assets: dict[str, bytes]) -> Any:
    if not hasattr(mujoco, "MjSpec"):
        raise RuntimeError("MjSpec API unavailable")
    try:
        spec = mujoco.MjSpec.from_file(path)
    except Exception:
        with open(path, "rb") as fh:
            raw = fh.read()
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise RuntimeError(f"binary_model_unsupported:{exc}") from exc
        try:
            spec = mujoco.MjSpec.from_string(text, assets=assets)
        except TypeError:
            spec = mujoco.MjSpec.from_string(text)
    if hasattr(spec, "assets") and spec.assets is not None:
        for key, data in assets.items():
            spec.assets[key] = data
    return spec


def _load(msg: dict[str, Any]) -> dict[str, Any]:
    INST.model = None
    INST.data = None
    INST.ctrl_write_count = 0
    INST.safe_write_count = 0
    INST.last_ctrl = []
    INST.warnings = []
    INST.lost_features = []
    INST.usd_experimental = False
    INST.asset_names = []
    fmt = str(msg.get("format", "mjcf")).lower()
    INST.source_format = fmt
    path = msg.get("path")
    extras = msg.get("objects") or []
    if fmt in {"usd", "usda", "usdc", "usdz"}:
        return {
            "ok": False,
            "error": "EXPERIMENTAL_UNSUPPORTED_IN_VERIFY_V1",
            "detail": "USD is not implemented in verify v1; MjModel.from_xml_path is not a USD importer",
            "evidence_status": EVIDENCE_STATUS,
            "metal": False,
        }
    if not path:
        return {"ok": False, "error": "model_path_required", "metal": False, "evidence_status": EVIDENCE_STATUS}
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        return {"ok": False, "error": f"missing_model:{path}", "metal": False, "evidence_status": EVIDENCE_STATUS}
    if fmt == "urdf" or path.lower().endswith(".urdf"):
        try:
            with open(path, "r", encoding="utf-8") as fh:
                INST.lost_features = _scan_urdf_losses(fh.read())
        except OSError:
            INST.lost_features = []
        INST.source_format = "urdf"
    else:
        INST.source_format = "mjcf"
    roots = [os.path.dirname(path)]
    roots.extend(str(r) for r in (msg.get("asset_roots") or []))
    assets = _collect_assets(roots)
    INST.asset_names = sorted({k for k in assets if "/" in k or "." in k})
    try:
        spec = _load_spec(path, assets)
        _add_scenario_objects(spec, extras)
        INST.model = spec.compile()
    except Exception as exc:
        err = str(exc)
        if "mesh" in err.lower() or "file" in err.lower() or "asset" in err.lower():
            return {
                "ok": False,
                "error": f"INVALID:missing_or_unreadable_asset:{err}",
                "metal": False,
                "evidence_status": EVIDENCE_STATUS,
            }
        return {"ok": False, "error": f"compile_failed:{err}", "metal": False, "evidence_status": EVIDENCE_STATUS}

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


def _set_ctrl(ctrl: list[float], write: bool, safe: bool = False) -> None:
    assert INST.model is not None and INST.data is not None
    if len(ctrl) != INST.model.nu:
        raise ValueError(f"ctrl_dim_mismatch:{len(ctrl)}!={INST.model.nu}")
    if not _finite(ctrl):
        raise ValueError("non_finite_ctrl")
    INST.data.ctrl[:] = ctrl
    if write:
        INST.ctrl_write_count += 1
        INST.last_ctrl = [float(x) for x in ctrl]
    elif safe:
        INST.safe_write_count += 1
        INST.last_ctrl = [float(x) for x in ctrl]


def _local_linear() -> dict[str, Any]:
    import numpy as np

    m, d = INST.model, INST.data
    assert m is not None and d is not None
    n = int(2 * m.nv)
    nu = int(m.nu)
    if n <= 0 or nu < 0:
        return {"ok": False, "error": "NOT_EVALUATED", "reason": "empty_state_or_input"}
    if n > 64:
        return {"ok": False, "error": "NOT_EVALUATED", "reason": "state_dim_too_large_for_verify_v1"}
    mujoco.mj_resetData(m, d)
    d.qpos[:] = m.qpos0
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    A = np.zeros((n, n), dtype=np.float64)
    B = np.zeros((n, max(nu, 0)), dtype=np.float64)
    try:
        mujoco.mjd_transitionFD(m, d, 1e-6, 1, A, B, None, None)
    except Exception as exc:
        return {"ok": False, "error": "NOT_EVALUATED", "reason": str(exc)}
    if not np.isfinite(A).all() or not np.isfinite(B).all():
        return {"ok": False, "error": "NOT_EVALUATED", "reason": "non_finite_jacobians"}
    tol = 1e-6
    if nu == 0:
        rank = 0
        ctrb_shape = [n, 0]
    else:
        blocks = []
        akb = B.copy()
        for _ in range(n):
            blocks.append(akb)
            akb = A @ akb
        ctrb = np.hstack(blocks)
        rank = int(np.linalg.matrix_rank(ctrb, tol=tol))
        ctrb_shape = [int(ctrb.shape[0]), int(ctrb.shape[1])]
    return {
        "ok": True,
        "label": "LOCAL_LINEAR_CONTROLLABILITY",
        "state": "compiled_qpos0_qvel0",
        "timestep": float(m.opt.timestep),
        "A_shape": [n, n],
        "B_shape": [n, nu],
        "controllability_matrix_shape": ctrb_shape,
        "rank": rank,
        "state_dim": n,
        "input_dim": nu,
        "tolerance": tol,
        "mujoco_version": mujoco.__version__,
        "method": "mjd_transitionFD",
        "note": "Local discrete linear controllability about the compiled reset. Not global nonlinear controllability.",
        "metal": False,
        "evidence_status": EVIDENCE_STATUS,
    }


def handle(msg: dict[str, Any]) -> dict[str, Any]:
    cmd = msg.get("cmd")
    if cmd == "hello":
        return {
            "ok": True,
            "mujoco_version": mujoco.__version__,
            "usd_supported": False,
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
            "schema": SCHEMA,
        }
    if cmd == "hang":
        time.sleep(float(msg.get("seconds", 3600)))
        return {"ok": True, "hung": True}
    if cmd == "load":
        return _load(msg)
    if INST.model is None or INST.data is None:
        return {"ok": False, "error": "not_loaded"}
    if cmd == "inspect":
        return {"ok": True, "inspect": INST.inspect()}
    if cmd == "local_linear":
        return _local_linear()
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
            "safe_write_count": INST.safe_write_count,
            "last_ctrl": list(INST.last_ctrl),
        }
    if cmd == "set_ctrl":
        _set_ctrl(list(msg["ctrl"]), write=True)
        return {"ok": True, "ctrl_write_count": INST.ctrl_write_count, "ctrl": [float(x) for x in INST.data.ctrl]}
    if cmd == "set_safe_ctrl":
        _set_ctrl(list(msg["ctrl"]), write=False, safe=True)
        return {
            "ok": True,
            "safe_write_count": INST.safe_write_count,
            "ctrl_write_count": INST.ctrl_write_count,
            "ctrl": [float(x) for x in INST.data.ctrl],
        }
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
    if cmd == "set_body_pos":
        body = str(msg.get("body") or "")
        pos = msg.get("pos") or [0, 0, 0]
        found = None
        for i in range(INST.model.nbody):
            if INST.model.body(i).name == body:
                found = i
                break
        if found is None:
            return {"ok": False, "error": f"unknown_body:{body}"}
        jnt = int(INST.model.body_jntadr[found])
        if jnt >= 0 and int(INST.model.jnt_type[jnt]) == 0:
            adr = int(INST.model.jnt_qposadr[jnt])
            INST.data.qpos[adr : adr + 3] = pos
            mujoco.mj_forward(INST.model, INST.data)
            return {"ok": True, "state": INST.state()}
        return {"ok": False, "error": "body_not_freejoint"}
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
        return {"ok": False, "error": "NOT_IMPLEMENTED_IN_VERIFY_V1"}
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

        def _run_ik(q_start: np.ndarray, iters: int) -> tuple[list[float], float]:
            jacp = np.zeros((3, INST.model.nv))
            jacr = np.zeros((3, INST.model.nv))
            damping = 1e-3
            q0 = np.array(INST.data.qpos, copy=True)
            INST.data.qpos[:] = q_start
            err_norm = 1e9
            for _ in range(iters):
                mujoco.mj_forward(INST.model, INST.data)
                mujoco.mj_jacSite(INST.model, INST.data, jacp, jacr, sid)
                err = target - np.array(INST.data.site_xpos[sid])
                err_norm = float(np.linalg.norm(err))
                if err_norm < 1e-4:
                    break
                jjt = jacp @ jacp.T + damping * np.eye(3)
                dq = jacp.T @ np.linalg.solve(jjt, err)
                for j in range(INST.model.njnt):
                    adr = int(INST.model.jnt_qposadr[j])
                    dof = int(INST.model.jnt_dofadr[j])
                    jtype = int(INST.model.jnt_type[j])
                    if jtype == 3 and 0 <= dof < INST.model.nv:
                        INST.data.qpos[adr] = float(INST.data.qpos[adr] + dq[dof])
                        if INST.model.jnt_limited[j]:
                            lo, hi = INST.model.jnt_range[j]
                            INST.data.qpos[adr] = min(
                                max(float(INST.data.qpos[adr]), float(lo)), float(hi)
                            )
            qpos = [float(x) for x in INST.data.qpos]
            INST.data.qpos[:] = q0
            mujoco.mj_forward(INST.model, INST.data)
            return qpos, err_norm

        q_home = np.array(INST.data.qpos, copy=True)
        iters = int(msg.get("iters", 80))
        seeds = [q_home]
        for j in range(INST.model.njnt):
            adr = int(INST.model.jnt_qposadr[j])
            jtype = int(INST.model.jnt_type[j])
            if jtype != 3:
                continue
            for delta in (0.35, -0.35, 0.75, -0.75, 1.1, -1.1):
                q = np.array(q_home, copy=True)
                q[adr] = float(q[adr] + delta)
                if INST.model.jnt_limited[j]:
                    lo, hi = INST.model.jnt_range[j]
                    q[adr] = min(max(float(q[adr]), float(lo)), float(hi))
                seeds.append(q)

        best_q = [float(x) for x in q_home]
        best_err = 1e9
        for seed in seeds:
            qpos, err_norm = _run_ik(seed, iters)
            if err_norm < best_err:
                best_q, best_err = qpos, err_norm
            if err_norm < 1e-3:
                break

        INST.data.qpos[:] = np.array(best_q, dtype=float)
        mujoco.mj_forward(INST.model, INST.data)
        best_err = float(np.linalg.norm(target - np.array(INST.data.site_xpos[sid])))
        INST.data.qpos[:] = q_home
        mujoco.mj_forward(INST.model, INST.data)

        return {
            "ok": True,
            "qpos": best_q,
            "error": best_err,
            "site": site,
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
        }
    if cmd == "inject_nan":
        if INST.model.nq > 0:
            INST.data.qpos[0] = float("nan")
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
    # Do not let MuJoCo / numpy spam fill the RPC stdout.
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
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
