#!/usr/bin/env python3
"""Isolated MuJoCo execution worker.

SIMULATION_ONLY. metal=false. This process never emits realityos.metal_proof.
It does not certify real-world safety. SIM != METAL.
"""

from __future__ import annotations

import gc
import hashlib
import json
import math
import os
import struct
import sys
import time
import traceback
from typing import Any

_ASSET_CACHE: dict[tuple[str, ...], dict[str, bytes]] = {}
_BASE_MODEL: dict[str, Any] = {}
_BASE_INSPECT: dict[str, Any] = {}
_BASE_MESH_ASSETS: dict[str, dict[int, str]] = {}

os.environ.setdefault("MUJOCO_GL", "disable")

EVIDENCE_STATUS = "SIMULATION_ONLY"
SCHEMA = "realityos.simulation_plant/1"

# Drain/redirect native logs so they cannot fill a stderr pipe.
os.environ.setdefault("MUJOCO_LOG_FILE", os.devnull)

try:
    import mujoco
    import numpy as np
    from mujoco_mesh_graph import (
        MAX_COMPILED_MESH_QUERY_VERTICES,
        MeshHullTooLarge,
        compiled_mesh_hull_vertex_ids,
        require_supported_mesh_graph_version,
    )
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
        self.mesh_asset_by_id: dict[int, str] = {}
        self.load_seed = 0
        self.wrench_sequence_by_profile: dict[str, int] = {}
        self.wrench_step_by_profile: dict[str, int] = {}
        self.wrench_latest_by_profile: dict[str, dict[str, Any]] = {}
        self.wrench_pending_by_profile: dict[str, list[dict[str, Any]]] = {}
        self.wrench_last_received_capture_by_profile: dict[str, float] = {}
        self.wrench_monitor_started_by_profile: dict[str, float] = {}
        self.wrench_gravity_reference_by_profile: dict[str, dict[str, Any]] = {}
        self.wrench_free_space_by_profile: dict[str, tuple[Any, Any]] = {}
        self.wrench_abort_latched = False

    def inspect(self) -> dict[str, Any]:
        m, d = self.model, self.data
        assert m is not None and d is not None

        def compiled_mesh_vertices(
            geom_id: int, mesh_id: int
        ) -> tuple[list[list[float]], str | None]:
            if mesh_id < 0 or mesh_id >= m.nmesh:
                raise RuntimeError(f"mesh geom references invalid compiled mesh id {mesh_id}")
            start = int(m.mesh_vertadr[mesh_id])
            count = int(m.mesh_vertnum[mesh_id])
            if count < 0 or start < 0 or start + count > len(m.mesh_vert):
                raise RuntimeError(
                    f"compiled mesh {mesh_id} has invalid vertex range {start}:{start + count}"
                )
            if count < 4:
                return [], "compiled_mesh_has_fewer_than_four_vertices"

            # MuJoCo stores the collision hull as a graph when the compiler
            # produced one. Validate the local packed record and its face
            # indices before materializing only the hull vertices.
            graph_start = int(m.mesh_graphadr[mesh_id])
            if graph_start >= 0:
                try:
                    require_supported_mesh_graph_version(mujoco.__version__)
                except ValueError as exc:
                    raise RuntimeError(str(exc)) from exc
                graph_starts = [int(value) for value in m.mesh_graphadr if int(value) >= 0]
                graph_end = min(
                    (value for value in graph_starts if value > graph_start),
                    default=len(m.mesh_graph),
                )
                try:
                    vertex_ids = compiled_mesh_hull_vertex_ids(
                        m.mesh_graph, graph_start, graph_end, count
                    )
                except MeshHullTooLarge:
                    return [], "compiled_hull_vertex_limit_exceeded"
                except ValueError as exc:
                    raise RuntimeError(
                        f"compiled mesh {mesh_id} has an invalid hull graph: {exc}"
                    ) from exc
                vertices = np.asarray(
                    m.mesh_vert[start + np.asarray(vertex_ids, dtype=np.int64)],
                    dtype=np.float64,
                )
            else:
                if count > MAX_COMPILED_MESH_QUERY_VERTICES:
                    return [], "compiled_mesh_vertex_limit_exceeded_without_hull_graph"
                vertices = np.asarray(m.mesh_vert[start : start + count], dtype=np.float64)
            if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
                raise RuntimeError(f"compiled mesh {mesh_id} has invalid vertices")

            # mesh_vert is already in the compiled mesh frame. The compiler
            # compensates mesh alignment in geom_pos/geom_quat, which the
            # inspector also retains; reapplying mesh_pos/mesh_quat here would
            # apply the same offset twice.
            radius = float(np.linalg.norm(vertices, axis=1).max())
            geom_radius = float(m.geom_rbound[geom_id])
            if not math.isfinite(geom_radius) or radius > geom_radius + max(1e-9, geom_radius * 1e-6):
                raise RuntimeError(
                    f"compiled mesh {mesh_id} vertices exceed geom {geom_id} bounding radius"
                )
            return vertices.tolist(), None

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
            kind = {
                0: "joint",
                1: "joint",
                2: "slidercrank",
                3: "tendon",
                4: "site",
                5: "body",
            }.get(trn, "other")
            target = ""
            if trn in (0, 1) and 0 <= target_id < m.njnt:
                target = m.joint(target_id).name
            elif trn == 3 and 0 <= target_id < getattr(m, "ntendon", 0):
                target = m.tendon(target_id).name
            elif 0 <= target_id < m.nbody:
                target = m.body(target_id).name
            actuators.append(
                {
                    "name": a.name or f"actuator_{i}",
                    "trntype": trn,
                    "transmission_kind": kind,
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
        _geom_names = {
            0: "plane",
            1: "hfield",
            2: "sphere",
            3: "capsule",
            4: "ellipsoid",
            5: "cylinder",
            6: "box",
            7: "mesh",
            8: "sdf",
        }
        for i in range(m.ngeom):
            gtype = int(m.geom_type[i])
            mesh_id = int(m.geom_dataid[i]) if gtype == 7 else -1
            mesh_vertices: list[list[float]] = []
            mesh_vertices_omitted_reason = None
            if gtype == 7:
                mesh_vertices, mesh_vertices_omitted_reason = compiled_mesh_vertices(
                    i, mesh_id
                )
            geoms.append(
                {
                    "id": i,
                    "name": m.geom(i).name or f"geom_{i}",
                    "body": m.body(int(m.geom_bodyid[i])).name,
                    "group": int(m.geom_group[i]),
                    "contype": int(m.geom_contype[i]),
                    "conaffinity": int(m.geom_conaffinity[i]),
                    "pos": [float(x) for x in m.geom_pos[i]],
                    "quat": [float(x) for x in m.geom_quat[i]],
                    "size": [float(x) for x in m.geom_size[i]],
                    "friction": [float(x) for x in m.geom_friction[i]],
                    "solref": [float(x) for x in m.geom_solref[i]],
                    "type": gtype,
                    "type_name": _geom_names.get(gtype, f"type_{gtype}"),
                    "mesh_asset": self.mesh_asset_by_id.get(mesh_id),
                    "mesh_vertices": mesh_vertices,
                    "mesh_vertices_omitted_reason": mesh_vertices_omitted_reason,
                }
            )
        contact_pairs = []
        for i in range(int(m.npair)):
            contact_pairs.append(
                {
                    "pair_id": i,
                    "geom1_id": int(m.pair_geom1[i]),
                    "geom2_id": int(m.pair_geom2[i]),
                }
            )
        excluded_body_pairs = []
        for i in range(int(m.nexclude)):
            signature = int(m.exclude_signature[i])
            body1_id = (signature >> 16) & 0xFFFF
            body2_id = signature & 0xFFFF
            if 0 <= body1_id < m.nbody and 0 <= body2_id < m.nbody:
                excluded_body_pairs.append(
                    {
                        "body1": m.body(body1_id).name or f"body_{body1_id}",
                        "body2": m.body(body2_id).name or f"body_{body2_id}",
                    }
                )
        tendons = []
        wrap_joint = int(getattr(mujoco.mjtWrap, "mjWRAP_JOINT", 1))
        for i in range(int(getattr(m, "ntendon", 0))):
            wrap_joints = []
            adr = int(m.tendon_adr[i]) if hasattr(m, "tendon_adr") else 0
            num = int(m.tendon_num[i]) if hasattr(m, "tendon_num") else 0
            for k in range(num):
                wrap = int(m.wrap_type[adr + k])
                obj = int(m.wrap_objid[adr + k])
                prm = float(m.wrap_prm[adr + k])
                if wrap == wrap_joint and 0 <= obj < m.njnt:
                    wrap_joints.append({"name": m.joint(obj).name or f"joint_{obj}", "coef": prm})
            tendons.append({"name": m.tendon(i).name or f"tendon_{i}", "joints": wrap_joints})
        equalities = []
        for i in range(int(getattr(m, "neq", 0))):
            eq_type = int(m.eq_type[i])
            obj1 = int(m.eq_obj1id[i])
            obj2 = int(m.eq_obj2id[i])
            type_name = {0: "connect", 1: "weld", 2: "joint", 3: "tendon"}.get(
                eq_type, f"type_{eq_type}"
            )
            name1 = ""
            name2 = ""
            if eq_type == 2:
                if 0 <= obj1 < m.njnt:
                    name1 = m.joint(obj1).name
                if 0 <= obj2 < m.njnt:
                    name2 = m.joint(obj2).name
            equalities.append(
                {
                    "name": f"eq_{i}",
                    "type": type_name,
                    "obj1": name1,
                    "obj2": name2,
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
            "qpos0": _jlist(m.qpos0),
            "gravity": [float(x) for x in m.opt.gravity],
            "joints": joints,
            "actuators": actuators,
            "sensors": sensors,
            "cameras": cameras,
            "bodies": bodies,
            "sites": sites,
            "geoms": geoms,
            "contact_pairs": contact_pairs,
            "excluded_body_pairs": excluded_body_pairs,
            "end_effector_chains": ee_chains,
            "tendons": tendons,
            "equalities": equalities,
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
                    "geom1_name": m.geom(g1).name or f"geom_{g1}",
                    "geom2_name": m.geom(g2).name or f"geom_{g2}",
                    "dist": float(c.dist),
                    "pos": [float(x) for x in c.pos],
                    "body1": m.body(int(m.geom_bodyid[g1])).name if g1 >= 0 else "",
                    "body2": m.body(int(m.geom_bodyid[g2])).name if g2 >= 0 else "",
                    "dim": int(c.dim),
                    "friction": [float(value) for value in c.friction],
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
                    nrm = float(force[0])
                    tan = float(math.sqrt(force[1] ** 2 + force[2] ** 2))
                    mag = float(math.sqrt(force[0] ** 2 + force[1] ** 2 + force[2] ** 2))
                    cfrc.append(mag)
                    contacts[i]["normal_force"] = nrm
                    contacts[i]["tangential_force"] = tan
                    contacts[i]["force_local"] = [float(force[0]), float(force[1]), float(force[2])]
                    contacts[i]["frame"] = [float(value) for value in d.contact[i].frame]
                    contacts[i]["force_available"] = True
                except Exception:
                    cfrc.append(0.0)
                    contacts[i]["force_available"] = False
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
    key = tuple(sorted(os.path.abspath(r) for r in roots if r))
    if key in _ASSET_CACHE:
        return _ASSET_CACHE[key]
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
    _ASSET_CACHE[key] = assets
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
        quat = obj.get("quat")
        body_kwargs = {"name": name, "pos": pos}
        if quat is not None:
            body_kwargs["quat"] = quat
        body = spec.worldbody.add_body(**body_kwargs)
        if movable and kind != "plane":
            body.add_freejoint()
        kwargs: dict[str, Any] = {"rgba": rgba, "mass": mass}
        if obj.get("friction") is not None:
            fr = float(obj["friction"])
            kwargs["friction"] = [fr, fr, 0.01]
        if obj.get("solref") is not None:
            solref = [float(value) for value in obj["solref"]]
            if len(solref) != 2 or not all(math.isfinite(value) and value > 0.0 for value in solref):
                raise ValueError("scenario_contact_solref_invalid")
            kwargs["solref"] = solref
        if kind == "sphere":
            body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[size[0], 0, 0], **kwargs)
        elif kind == "capsule":
            body.add_geom(
                type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=[size[0], size[1] if len(size) > 1 else 0.1, 0],
                **kwargs,
            )
        elif kind == "cylinder":
            body.add_geom(
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                size=[size[0], size[1] if len(size) > 1 else 0.04, 0],
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


def _drop_runtime(keep_if_cached: bool = True) -> None:
    prev = INST.model
    INST.model = None
    INST.data = None
    INST.wrench_free_space_by_profile = {}
    cached = keep_if_cached and any(prev is m for m in _BASE_MODEL.values())
    if prev is not None and not cached:
        del prev
        gc.collect()


def _finite_six(values: Any, *, positive: bool = False) -> bool:
    return (
        isinstance(values, list)
        and len(values) == 6
        and _finite(values)
        and (not positive or all(float(value) > 0.0 for value in values))
    )


def _digest_string(hasher, value: str) -> None:
    encoded = value.encode("utf-8")
    hasher.update(struct.pack("<Q", len(encoded)))
    hasher.update(encoded)


def _policy_digest(policy: dict[str, Any]) -> str:
    scope = policy["scope"]
    contract = policy["abort_contract"]
    tracking_correction = float(policy.get("max_position_tracking_correction_rad", 0.0))
    tracking_gain = float(policy.get("position_tracking_correction_gain", 0.0))
    if (
        not math.isfinite(tracking_correction)
        or tracking_correction < 0.0
        or tracking_correction > 0.05
        or not math.isfinite(tracking_gain)
        or tracking_gain < 0.0
        or tracking_gain > 3.0
        or (tracking_correction == 0.0 and tracking_gain != 0.0)
        or (tracking_correction > 0.0 and tracking_gain == 0.0)
    ):
        raise ValueError("position_tracking_correction_bound")
    hasher = hashlib.sha256()
    hasher.update(
        b"realityos.wrench_limit_policy/2\0"
        if tracking_correction > 0.0
        else b"realityos.wrench_limit_policy/1\0"
    )
    _digest_string(hasher, str(scope["sensor_id"]))
    _digest_string(hasher, str(scope["sensor_frame"]))
    for value in scope["caps"]:
        hasher.update(struct.pack("<d", float(value)))
    _digest_string(hasher, str(scope["abort_contract_id"]))
    _digest_string(hasher, str(policy["sensor_calibration_hash"]))
    hasher.update(struct.pack("<d", float(policy["sensor_sample_rate_hz"])))
    if tracking_correction > 0.0:
        hasher.update(struct.pack("<d", tracking_correction))
        hasher.update(struct.pack("<d", tracking_gain))
    for value in policy["max_rise_rate"]:
        hasher.update(struct.pack("<d", float(value)))
    for field in (
        "max_sensing_latency_s",
        "max_abort_latency_s",
        "max_sample_age_s",
        "max_inter_sample_gap_s",
        "max_cross_sensor_skew_s",
    ):
        hasher.update(struct.pack("<d", float(policy[field])))
    _digest_string(hasher, str(policy["monitor_id"]))
    _digest_string(hasher, str(policy["compensation_digest"]))
    _digest_string(hasher, str(contract["contract_id"]))
    if contract["hold_mode"] != "hold_current_joint_position":
        raise ValueError("wrench_abort_hold_mode")
    hasher.update(bytes([0]))
    actuators = contract["allowed_stop_actuator_ids"]
    if not isinstance(actuators, list):
        raise ValueError("wrench_stop_actuators")
    hasher.update(struct.pack("<Q", len(actuators)))
    for actuator in actuators:
        _digest_string(hasher, str(actuator))
    hasher.update(struct.pack("<d", float(contract["max_time_to_stop_s"])))
    hasher.update(struct.pack("<I", int(contract["max_stop_writes"])))
    return hasher.hexdigest()


def _simulation_profile_digest(profile: dict[str, Any]) -> str:
    hasher = hashlib.sha256()
    hasher.update(b"realityos.simulation_wrench_sensor_profile/2\0")
    for field in ("sensor_id", "model_id", "calibration_hash", "sensor_frame", "units"):
        _digest_string(hasher, str(profile[field]))
    hasher.update(struct.pack("<d", float(profile["sample_rate_hz"])))
    hasher.update(struct.pack("<d", float(profile["latency_s"])))
    for field in (
        "resolution",
        "bias",
        "error_bounds",
        "free_space_model_error_bounds",
        "seeded_noise_bound",
    ):
        values = profile.get(field, [0.0] * 6) if field == "free_space_model_error_bounds" else profile[field]
        for value in values:
            hasher.update(struct.pack("<d", float(value)))
    _digest_string(hasher, _policy_digest(profile["limit_policy"]))
    return hasher.hexdigest()


def _wrench_profile_steps(profile: dict[str, Any], profile_digest: str, timestep: float) -> int:
    if not isinstance(profile, dict):
        raise ValueError("wrench_profile_missing")
    policy = profile.get("limit_policy")
    scope = policy.get("scope") if isinstance(policy, dict) else None
    if not isinstance(scope, dict):
        raise ValueError("wrench_policy_missing")
    sensor_id = str(profile.get("sensor_id") or "")
    sensor_frame = str(profile.get("sensor_frame") or "")
    if (
        not sensor_id
        or profile.get("model_id") not in {
            "mujoco-force-torque-sensor-gravity-baseline-v1",
            "mujoco-force-torque-sensor-gravity-baseline-v2",
            "mujoco-force-torque-sensor-free-space-model-v1",
        }
        or not sensor_frame
        or profile.get("units") != "N,N*m"
        or profile.get("calibration_hash") != policy.get("sensor_calibration_hash")
        or profile.get("sample_rate_hz") != policy.get("sensor_sample_rate_hz")
        or profile.get("latency_s") != policy.get("max_sensing_latency_s")
        or scope.get("sensor_id") != sensor_id
        or scope.get("sensor_frame") != sensor_frame
        or len(str(scope.get("policy_digest") or "")) != 64
        or len(str(profile_digest or "")) != 64
    ):
        raise ValueError("wrench_profile_policy_mismatch")
    computed_policy_digest = _policy_digest(policy)
    if str(scope["policy_digest"]).lower() != computed_policy_digest:
        raise ValueError("wrench_policy_digest_mismatch")
    if profile_digest.lower() != _simulation_profile_digest(profile):
        raise ValueError("wrench_profile_digest_mismatch")
    rate = float(profile["sample_rate_hz"])
    latency = float(profile["latency_s"])
    gap = float(policy["max_inter_sample_gap_s"])
    timestep = float(timestep)
    if not math.isfinite(rate) or rate <= 0.0 or not math.isfinite(timestep) or timestep <= 0.0:
        raise ValueError("wrench_rate_or_timestep")
    if not _finite_six(scope.get("caps"), positive=True):
        raise ValueError("wrench_caps")
    if not _finite_six(profile.get("resolution"), positive=True):
        raise ValueError("wrench_resolution")
    if not _finite_six(profile.get("bias")):
        raise ValueError("wrench_bias")
    if not _finite_six(profile.get("error_bounds")) or any(float(x) < 0.0 for x in profile["error_bounds"]):
        raise ValueError("wrench_error_bounds")
    free_space_error = profile.get("free_space_model_error_bounds", [0.0] * 6)
    if not _finite_six(free_space_error) or any(float(x) < 0.0 for x in free_space_error):
        raise ValueError("wrench_free_space_model_error_bounds")
    if str(profile.get("model_id")) != "mujoco-force-torque-sensor-free-space-model-v1" and any(
        float(value) != 0.0 for value in free_space_error
    ):
        raise ValueError("wrench_free_space_model_identity")
    if not _finite_six(profile.get("seeded_noise_bound")) or any(
        float(noise) < 0.0
        or float(noise) + float(resolution) * 0.5 > float(error) + 1e-12
        for noise, resolution, error in zip(
            profile["seeded_noise_bound"], profile["resolution"], profile["error_bounds"]
        )
    ):
        raise ValueError("wrench_noise_bound")
    sample_steps_float = 1.0 / rate / timestep
    gap_steps_float = gap / timestep
    latency_steps_float = latency / timestep
    for label, steps in (
        ("sample_rate", sample_steps_float),
        ("max_gap", gap_steps_float),
    ):
        if steps < 1.0 or abs(steps - round(steps)) > 1e-9 * max(1.0, abs(steps)):
            raise ValueError(f"wrench_{label}_not_representable")
    if latency > 0.0 and (
        latency_steps_float < 1.0
        or abs(latency_steps_float - round(latency_steps_float))
        > 1e-9 * max(1.0, abs(latency_steps_float))
    ):
        raise ValueError("wrench_latency_not_representable")
    return int(round(sample_steps_float))


def _add_wrench_sensors(spec: Any, frame_name: str) -> None:
    name = str(frame_name)
    sites = list(getattr(spec, "sites", []) or [])
    site = next((candidate for candidate in sites if candidate.name == name), None)
    if site is None:
        body = next((candidate for candidate in (getattr(spec, "bodies", []) or []) if candidate.name == name), None)
        if body is None or body.name == "world":
            raise ValueError(f"wrench_sensor_frame_unknown:{name}")
        site_name = "realityos_wrench_sensor_frame"
        if any(candidate.name == site_name for candidate in sites):
            raise ValueError("wrench_sensor_site_name_collision")
        body.add_site(name=site_name, pos=[0.0, 0.0, 0.0], quat=[1.0, 0.0, 0.0, 0.0], size=[0.001])
    else:
        site_name = name

    sensor_names = {"realityos_wrench_force_sensor", "realityos_wrench_torque_sensor"}
    if any(sensor.name in sensor_names for sensor in (getattr(spec, "sensors", []) or [])):
        raise ValueError("wrench_sensor_name_collision")
    spec.add_sensor(
        name="realityos_wrench_force_sensor",
        type=mujoco.mjtSensor.mjSENS_FORCE,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname=site_name,
    )
    spec.add_sensor(
        name="realityos_wrench_torque_sensor",
        type=mujoco.mjtSensor.mjSENS_TORQUE,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname=site_name,
    )


def _wrench_sensor_ids(profile: dict[str, Any]) -> tuple[int, int]:
    m = INST.model
    assert m is not None
    frame_name = str(profile["sensor_frame"])
    site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, frame_name)
    if site_id < 0:
        site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "realityos_wrench_sensor_frame")
    if site_id < 0:
        raise ValueError(f"wrench_sensor_frame_unknown:{frame_name}")
    force_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "realityos_wrench_force_sensor")
    torque_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "realityos_wrench_torque_sensor")
    if force_id < 0 or torque_id < 0:
        raise ValueError("wrench_sensor_model_missing")
    for sensor_id, sensor_type in (
        (force_id, mujoco.mjtSensor.mjSENS_FORCE),
        (torque_id, mujoco.mjtSensor.mjSENS_TORQUE),
    ):
        if (
            int(m.sensor_type[sensor_id]) != int(sensor_type)
            or int(m.sensor_objtype[sensor_id]) != int(mujoco.mjtObj.mjOBJ_SITE)
            or int(m.sensor_objid[sensor_id]) != site_id
            or int(m.sensor_dim[sensor_id]) != 3
        ):
            raise ValueError("wrench_sensor_model_mismatch")
    return force_id, torque_id


def _tool_sensor_wrench(sensor_ids: tuple[int, int], data=None):
    import numpy as np

    m, live_data = INST.model, INST.data
    d = data if data is not None else live_data
    assert m is not None and d is not None
    force_id, torque_id = sensor_ids
    force_start = int(m.sensor_adr[force_id])
    torque_start = int(m.sensor_adr[torque_id])
    # MuJoCo reports the interaction wrench from the instrumented child body
    # toward its parent. The typed sample convention is the wrench acting on
    # the tool, so convert the complete six-axis wrench by action/reaction.
    return np.concatenate(
        (
            -np.asarray(d.sensordata[force_start : force_start + 3], dtype=float),
            -np.asarray(d.sensordata[torque_start : torque_start + 3], dtype=float),
        )
    )


def _model_free_space_wrench(
    profile_digest: str, sensor_ids: tuple[int, int]
) -> list[float]:
    """Predict the instrumented robot subtree's load without environment contacts.

    This model term is paired with, and subtracted from, the live MuJoCo force
    sensor sample. It includes the current articulated gravity and inertial load
    at the same simulator state. Geoms in the instrumented robot's root subtree
    retain self-contact; all other geoms are collision-disabled only while the
    separate shadow data is forwarded. The live simulation state is untouched.
    """
    m, d = INST.model, INST.data
    assert m is not None and d is not None
    cache = INST.wrench_free_space_by_profile.get(profile_digest)
    if cache is None:
        site_id = int(m.sensor_objid[sensor_ids[0]])
        body_id = int(m.site_bodyid[site_id])
        root_body = body_id
        while root_body > 0 and int(m.body_parentid[root_body]) > 0:
            root_body = int(m.body_parentid[root_body])
        if root_body <= 0:
            raise ValueError("wrench_sensor_robot_root_missing")
        robot_bodies = set()
        for candidate in range(1, int(m.nbody)):
            ancestor = candidate
            while ancestor > 0 and ancestor != root_body:
                ancestor = int(m.body_parentid[ancestor])
            if ancestor == root_body:
                robot_bodies.add(candidate)
        external_geoms = np.asarray(
            [int(m.geom_bodyid[geom_id]) not in robot_bodies for geom_id in range(int(m.ngeom))],
            dtype=bool,
        )
        cache = (mujoco.MjData(m), external_geoms)
        INST.wrench_free_space_by_profile[profile_digest] = cache
    shadow, external_geoms = cache
    mujoco.mj_copyData(shadow, m, d)
    captured_qacc = np.asarray(d.qacc, dtype=float).copy()
    original_contype = np.asarray(m.geom_contype[external_geoms]).copy()
    original_conaffinity = np.asarray(m.geom_conaffinity[external_geoms]).copy()
    try:
        m.geom_contype[external_geoms] = 0
        m.geom_conaffinity[external_geoms] = 0
        # A full shadow forward pass refreshes collision and no-contact
        # constraint forces. Its free-space qacc is only an intermediate:
        # restore the captured live acceleration before evaluating force
        # sensors so the prediction is paired with the measured motion state.
        mujoco.mj_forward(m, shadow)
        np.copyto(shadow.qacc, captured_qacc)
        # Rebuild the acceleration-dependent internal-force cache from the
        # restored live qacc before evaluating sensors. mj_sensorAcc alone can
        # reuse cacc/cfrc_int from the preceding no-contact forward pass.
        mujoco.mj_rnePostConstraint(m, shadow)
        mujoco.mj_sensorAcc(m, shadow)
        return [float(value) for value in _tool_sensor_wrench(sensor_ids, shadow)]
    finally:
        m.geom_contype[external_geoms] = original_contype
        m.geom_conaffinity[external_geoms] = original_conaffinity


def _seeded_wrench_noise(seed: int, profile_digest: str, sequence: int, axis: int, bound: float) -> float:
    payload = f"{seed}:{profile_digest}:{sequence}:{axis}".encode("utf-8")
    integer = int.from_bytes(hashlib.sha256(payload).digest()[:8], "little")
    unit = integer / ((1 << 64) - 1)
    return (2.0 * unit - 1.0) * bound


def _sample_tool_wrench(
    profile: dict[str, Any], profile_digest: str, sequence: int, sensor_ids: tuple[int, int]
) -> dict[str, Any]:
    m, d = INST.model, INST.data
    assert m is not None and d is not None
    # mj_step integrates to the next state after computing its sensors. Re-run
    # forward at the captured state so the measured and model-only wrenches are
    # timestamp-aligned rather than mixing adjacent physics steps.
    mujoco.mj_forward(m, d)
    # Keep the raw complete force-sensor reading. The same-state shadow model
    # supplies a separate free-space load for typed probe compensation; safety
    # monitoring still sees this unfiltered raw wrench.
    true_wrench = _tool_sensor_wrench(sensor_ids)
    model_free_space_wrench = _model_free_space_wrench(profile_digest, sensor_ids)
    measured = []
    for axis in range(6):
        bound = float(profile["seeded_noise_bound"][axis])
        noise = _seeded_wrench_noise(INST.load_seed, profile_digest, sequence, axis, bound)
        if abs(noise) > bound + 1e-15:
            raise ValueError("wrench_seeded_noise_out_of_bound")
        value = float(true_wrench[axis]) + float(profile["bias"][axis]) + noise
        resolution = float(profile["resolution"][axis])
        quantized = round(value / resolution) * resolution
        if abs(quantized - (float(true_wrench[axis]) + float(profile["bias"][axis]))) > float(
            profile["error_bounds"][axis]
        ) + 1e-12:
            raise ValueError("wrench_sample_error_bound_exceeded")
        measured.append(float(quantized))
    capture_s = float(INST.data.time)
    site_id = int(m.sensor_objid[sensor_ids[0]])
    sensor_position = [float(value) for value in d.site_xpos[site_id]]
    sensor_quaternion = np.empty(4, dtype=float)
    mujoco.mju_mat2Quat(sensor_quaternion, d.site_xmat[site_id])
    return {
        "sequence": sequence,
        "capture_s": capture_s,
        "receive_s": capture_s + float(profile["latency_s"]),
        "wrench": measured,
        "model_free_space_wrench": model_free_space_wrench,
        "model_free_space_wrench_error_bounds": [
            float(value) for value in profile.get("free_space_model_error_bounds", [0.0] * 6)
        ],
        "error_bounds": [float(value) for value in profile["error_bounds"]],
        "sensor_position": sensor_position,
        "sensor_quaternion": [float(value) for value in sensor_quaternion],
    }


def _sensor_subtree_has_contact(sensor_ids: tuple[int, int]) -> bool:
    """Return whether any external contact touches the instrumented body subtree."""
    m, d = INST.model, INST.data
    assert m is not None and d is not None
    site_id = int(m.sensor_objid[sensor_ids[0]])
    root_body = int(m.site_bodyid[site_id])

    def in_sensor_subtree(body_id: int) -> bool:
        while body_id > 0:
            if body_id == root_body:
                return True
            body_id = int(m.body_parentid[body_id])
        return root_body == 0

    for contact_index in range(int(d.ncon)):
        contact = d.contact[contact_index]
        body1 = int(m.geom_bodyid[int(contact.geom1)])
        body2 = int(m.geom_bodyid[int(contact.geom2)])
        if in_sensor_subtree(body1) or in_sensor_subtree(body2):
            return True
    return False


def _sensor_subtree_body_ids(sensor_ids: tuple[int, int]) -> set[int]:
    """Return the instrumented body and all descendants whose loads cross its sensor."""
    m = INST.model
    assert m is not None
    site_id = int(m.sensor_objid[sensor_ids[0]])
    root_body = int(m.site_bodyid[site_id])
    if root_body <= 0:
        raise ValueError("wrench_sensor_robot_root_missing")
    result: set[int] = set()
    for candidate in range(1, int(m.nbody)):
        ancestor = candidate
        while ancestor > 0 and ancestor != root_body:
            ancestor = int(m.body_parentid[ancestor])
        if ancestor == root_body:
            result.add(candidate)
    return result


def _sensor_contact_pair_is_expected(
    body1_id: int,
    body2_id: int,
    sensor_subtree_body_ids: set[int],
    expected_contact_body_id: int | None,
) -> bool:
    """Ignore unrelated contacts; accept only a tool-subtree/authorized-target pair."""
    body1_in_sensor = body1_id in sensor_subtree_body_ids
    body2_in_sensor = body2_id in sensor_subtree_body_ids
    if not body1_in_sensor and not body2_in_sensor:
        return True
    if expected_contact_body_id is None:
        return False
    return (
        body1_in_sensor
        and not body2_in_sensor
        and body2_id == expected_contact_body_id
    ) or (
        body2_in_sensor
        and not body1_in_sensor
        and body1_id == expected_contact_body_id
    )


def _actuator_force_limit_reached(
    force: float, force_limited: bool, force_range: tuple[float, float]
) -> bool:
    """Detect force clipping conservatively, including values just outside a bound."""
    if not force_limited:
        return False
    low, high = (float(value) for value in force_range)
    force = float(force)
    if (
        not math.isfinite(force)
        or not math.isfinite(low)
        or not math.isfinite(high)
        or low >= high
    ):
        raise ValueError("actuator_force_limit_invalid")
    tolerance = max(1e-8, 1e-6 * max(1.0, abs(low), abs(high), high - low))
    return force <= low + tolerance or force >= high - tolerance


def _sensor_subtree_actuator_saturations(sensor_ids: tuple[int, int]) -> list[str]:
    """Return force-limited robot actuators at a force bound on each physics step."""
    m, d = INST.model, INST.data
    assert m is not None and d is not None
    site_id = int(m.sensor_objid[sensor_ids[0]])
    sensor_body = int(m.site_bodyid[site_id])
    robot_root = sensor_body
    while robot_root > 0 and int(m.body_parentid[robot_root]) > 0:
        robot_root = int(m.body_parentid[robot_root])
    if robot_root <= 0:
        raise ValueError("wrench_sensor_robot_root_missing")

    saturated: list[str] = []
    for actuator_id in range(int(m.nu)):
        if not bool(m.actuator_forcelimited[actuator_id]):
            continue
        transmission_type = int(m.actuator_trntype[actuator_id])
        if transmission_type not in (0, 1):
            name = m.actuator(actuator_id).name or f"actuator_{actuator_id}"
            raise ValueError(
                f"wrench_force_limited_actuator_transmission_unsupported:{name}"
            )
        joint_id = int(m.actuator_trnid[actuator_id][0])
        if joint_id < 0 or joint_id >= int(m.njnt):
            name = m.actuator(actuator_id).name or f"actuator_{actuator_id}"
            raise ValueError(f"wrench_force_limited_actuator_joint_invalid:{name}")
        actuator_body = int(m.jnt_bodyid[joint_id])
        ancestor = actuator_body
        while ancestor > 0 and ancestor != robot_root:
            ancestor = int(m.body_parentid[ancestor])
        if ancestor != robot_root:
            continue
        force_range = tuple(float(value) for value in m.actuator_forcerange[actuator_id])
        if _actuator_force_limit_reached(
            float(d.actuator_force[actuator_id]), True, force_range
        ):
            saturated.append(m.actuator(actuator_id).name or f"actuator_{actuator_id}")
    return saturated


def _has_unexpected_sensor_subtree_contact(
    sensor_subtree_body_ids: set[int], expected_contact_body_id: int | None
) -> bool:
    m, d = INST.model, INST.data
    assert m is not None and d is not None
    for contact_index in range(int(d.ncon)):
        contact = d.contact[contact_index]
        body1_id = int(m.geom_bodyid[int(contact.geom1)])
        body2_id = int(m.geom_bodyid[int(contact.geom2)])
        if not _sensor_contact_pair_is_expected(
            body1_id, body2_id, sensor_subtree_body_ids, expected_contact_body_id
        ):
            return True
    return False


def _sensor_is_quiescent(sensor_ids: tuple[int, int]) -> bool:
    """Require the instrumented subtree's joints to have settled, not merely zero velocity."""
    m, d = INST.model, INST.data
    assert m is not None and d is not None
    site_id = int(m.sensor_objid[sensor_ids[0]])
    sensor_body = int(m.site_bodyid[site_id])

    def is_ancestor_or_same(ancestor: int, body_id: int) -> bool:
        while body_id > 0:
            if body_id == ancestor:
                return True
            body_id = int(m.body_parentid[body_id])
        return ancestor == 0

    dof_count_by_joint_type = {0: 6, 1: 3, 2: 1, 3: 1}
    relevant_velocities = []
    relevant_accelerations = []
    for joint_id in range(int(m.njnt)):
        joint_body = int(m.jnt_bodyid[joint_id])
        if not (
            is_ancestor_or_same(joint_body, sensor_body)
            or is_ancestor_or_same(sensor_body, joint_body)
        ):
            continue
        dof_address = int(m.jnt_dofadr[joint_id])
        dof_count = dof_count_by_joint_type.get(int(m.jnt_type[joint_id]), 0)
        relevant_velocities.extend(
            float(d.qvel[dof_address + offset]) for offset in range(dof_count)
        )
        relevant_accelerations.extend(
            float(d.qacc[dof_address + offset]) for offset in range(dof_count)
        )
    return (
        bool(relevant_velocities)
        and _finite(relevant_velocities)
        and _finite(relevant_accelerations)
        and max((abs(value) for value in relevant_velocities), default=0.0) <= 1e-3
        and max((abs(value) for value in relevant_accelerations), default=0.0) <= 1e-2
    )


def _compensate_wrench_with_reference(
    sample: dict[str, Any],
    profile: dict[str, Any],
    reference: dict[str, Any] | None,
) -> tuple[list[float], list[float]]:
    """Remove bias and rotate the rigidly attached no-contact load into this sensor frame."""
    compensated = [
        float(sample["wrench"][axis]) - float(profile["bias"][axis])
        for axis in range(6)
    ]
    error = [float(value) for value in sample["error_bounds"]]
    if reference is None:
        return compensated, error

    reference_wrench = [
        float(reference["wrench"][axis]) - float(profile["bias"][axis])
        for axis in range(6)
    ]
    reference_rotation = np.empty(9, dtype=float)
    current_rotation = np.empty(9, dtype=float)
    mujoco.mju_quat2Mat(reference_rotation, np.asarray(reference["sensor_quaternion"], dtype=float))
    mujoco.mju_quat2Mat(current_rotation, np.asarray(sample["sensor_quaternion"], dtype=float))
    reference_rotation = reference_rotation.reshape((3, 3))
    current_rotation = current_rotation.reshape((3, 3))
    force_world = reference_rotation @ np.asarray(reference_wrench[:3], dtype=float)
    torque_world = reference_rotation @ np.asarray(reference_wrench[3:], dtype=float)
    reference_bounds = np.asarray(reference["error_bounds"], dtype=float)
    force_error_world = np.abs(reference_rotation) @ reference_bounds[:3]
    torque_error_world = np.abs(reference_rotation) @ reference_bounds[3:]
    if profile["model_id"] == "mujoco-force-torque-sensor-gravity-baseline-v1":
        arm = np.asarray(reference["sensor_position"], dtype=float) - np.asarray(
            sample["sensor_position"], dtype=float
        )
        torque_world = torque_world + np.cross(arm, force_world)
        arm_error = np.abs(arm)
        torque_error_world += np.asarray(
            [
                arm_error[1] * force_error_world[2] + arm_error[2] * force_error_world[1],
                arm_error[2] * force_error_world[0] + arm_error[0] * force_error_world[2],
                arm_error[0] * force_error_world[1] + arm_error[1] * force_error_world[0],
            ],
            dtype=float,
        )
    force_current = current_rotation.T @ force_world
    torque_current = current_rotation.T @ torque_world
    for axis in range(3):
        compensated[axis] -= float(force_current[axis])
        compensated[axis + 3] -= float(torque_current[axis])

    force_error_current = np.abs(current_rotation.T) @ force_error_world
    torque_error_current = np.abs(current_rotation.T) @ torque_error_world
    for axis in range(3):
        error[axis] += float(force_error_current[axis])
        error[axis + 3] += float(torque_error_current[axis])
    if not _finite(compensated) or not _finite(error):
        raise ValueError("wrench_gravity_compensation_nonfinite")
    return compensated, error


def _validate_wrench_abort_contract(policy: dict[str, Any]) -> list[str]:
    contract = policy.get("abort_contract")
    if not isinstance(contract, dict):
        raise ValueError("wrench_abort_contract_missing")
    if contract.get("hold_mode") != "hold_current_joint_position":
        raise ValueError("wrench_abort_hold_mode")
    actuator_ids = contract.get("allowed_stop_actuator_ids")
    if (
        not isinstance(actuator_ids, list)
        or not actuator_ids
        or any(not isinstance(name, str) or not name.strip() for name in actuator_ids)
        or len(set(actuator_ids)) != len(actuator_ids)
    ):
        raise ValueError("wrench_stop_actuators")
    if (
        not math.isfinite(float(contract.get("max_time_to_stop_s", 0.0)))
        or float(contract["max_time_to_stop_s"]) <= 0.0
        or int(contract.get("max_stop_writes", 0)) < 1
        or float(contract["max_time_to_stop_s"]) > float(policy["max_abort_latency_s"])
    ):
        raise ValueError("wrench_abort_contract_limits")
    return list(actuator_ids)


def _validate_monitored_grant(msg: dict[str, Any], policy: dict[str, Any]) -> None:
    scope = policy["scope"]
    granted = msg.get("grant_wrench_scope")
    envelope = msg.get("grant_action_envelope")
    if (
        not str(msg.get("grant_id") or "")
        or not str(msg.get("grant_scope_digest") or "")
        or not str(msg.get("observation_epoch") or "")
        or not isinstance(granted, dict)
        or not isinstance(envelope, dict)
        or envelope.get("kind") != "linear_stroke"
        or granted.get("sensor_id") != scope.get("sensor_id")
        or granted.get("sensor_frame") != scope.get("sensor_frame")
        or granted.get("policy_digest") != scope.get("policy_digest")
        or granted.get("abort_contract_id") != scope.get("abort_contract_id")
        or not _finite_six(granted.get("caps"), positive=True)
        or any(
            struct.pack("<d", float(left)) != struct.pack("<d", float(right))
            for left, right in zip(granted["caps"], scope["caps"])
        )
        or envelope.get("wrench_limit") != granted
    ):
        raise ValueError("wrench_grant_scope_mismatch")
    requested = float(envelope.get("requested_m", 0.0))
    maximum = float(envelope.get("maximum_m", 0.0))
    if not math.isfinite(requested) or not math.isfinite(maximum) or requested <= 0.0 or maximum < requested:
        raise ValueError("wrench_grant_stroke_bounds")


def _validate_monitored_control_ramp(
    msg: dict[str, Any], policy: dict[str, Any], n: int
) -> dict[str, Any] | None:
    ramp = msg.get("control_ramp")
    if ramp is None:
        return None
    if not isinstance(ramp, dict):
        raise ValueError("wrench_control_ramp_shape")
    m, d = INST.model, INST.data
    assert m is not None and d is not None
    start = ramp.get("start_ctrl")
    target = ramp.get("target_ctrl")
    elapsed = int(ramp.get("elapsed_steps", -1))
    total = int(ramp.get("total_steps", 0))
    if (
        not isinstance(start, list)
        or not isinstance(target, list)
        or len(start) != int(m.nu)
        or len(target) != int(m.nu)
        or not _finite(start)
        or not _finite(target)
        or elapsed < 0
        or total <= 0
        or elapsed + n > total
        or total > 100_000
    ):
        raise ValueError("wrench_control_ramp_bounds")
    allowed = set(_validate_wrench_abort_contract(policy))
    allowed_ids = set()
    for actuator_name in allowed:
        actuator_id = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name))
        if actuator_id < 0:
            raise ValueError(f"wrench_control_ramp_actuator_unknown:{actuator_name}")
        allowed_ids.add(actuator_id)
    for index, (start_value, target_value) in enumerate(zip(start, target)):
        if index not in allowed_ids:
            if abs(float(target_value) - float(start_value)) > 1e-12:
                raise ValueError(f"wrench_control_ramp_out_of_scope:{index}")
            continue
        low, high = (float(value) for value in m.actuator_ctrlrange[index])
        if (
            not bool(m.actuator_ctrllimited[index])
            or float(start_value) < low - 1e-10
            or float(start_value) > high + 1e-10
            or float(target_value) < low - 1e-10
            or float(target_value) > high + 1e-10
        ):
            raise ValueError(f"wrench_control_ramp_ctrlrange:{index}")
        if (
            int(m.actuator_trntype[index]) not in (0, 1)
            or int(m.actuator_biastype[index]) != 1
        ):
            raise ValueError(f"wrench_control_ramp_actuator_not_position_joint:{index}")
        joint_id = int(m.actuator_trnid[index][0])
        if (
            joint_id < 0
            or joint_id >= m.njnt
            or int(m.jnt_type[joint_id]) not in (2, 3)
        ):
            raise ValueError(f"wrench_control_ramp_joint_not_scalar:{index}")
    progress = elapsed / total
    eased = progress * progress * (3.0 - 2.0 * progress)
    for index, (start_value, target_value) in enumerate(zip(start, target)):
        expected = float(start_value) + (float(target_value) - float(start_value)) * eased
        correction_bound = float(policy.get("max_position_tracking_correction_rad", 0.0))
        allowed_error = correction_bound + 1e-8 if index in allowed_ids else 1e-8
        if abs(float(d.ctrl[index]) - expected) > allowed_error:
            raise ValueError(f"wrench_control_ramp_stale_start:{index}")
    return {
        "start_ctrl": start,
        "target_ctrl": target,
        "elapsed_steps": elapsed,
        "total_steps": total,
        "tracking_actuator_ids": sorted(allowed_ids),
    }


def _hold_current_position(policy: dict[str, Any]) -> tuple[bool, list[str]]:
    """Apply one bounded safe write to the registered scalar position joints."""
    m, d = INST.model, INST.data
    assert m is not None and d is not None
    actuator_names = _validate_wrench_abort_contract(policy)
    before = [float(value) for value in d.ctrl]
    updates: dict[int, float] = {}
    for name in actuator_names:
        actuator_id = int(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
        if actuator_id < 0:
            raise ValueError(f"wrench_stop_actuator_unknown:{name}")
        if int(m.actuator_trntype[actuator_id]) not in (0, 1) or int(m.actuator_biastype[actuator_id]) != 1:
            raise ValueError(f"wrench_stop_actuator_not_position_joint:{name}")
        joint_id = int(m.actuator_trnid[actuator_id][0])
        if joint_id < 0 or joint_id >= m.njnt or int(m.jnt_type[joint_id]) not in (2, 3):
            raise ValueError(f"wrench_stop_joint_not_scalar:{name}")
        if not bool(m.actuator_ctrllimited[actuator_id]):
            raise ValueError(f"wrench_stop_control_unbounded:{name}")
        qpos_address = int(m.jnt_qposadr[joint_id])
        target = float(d.qpos[qpos_address])
        low, high = (float(value) for value in m.actuator_ctrlrange[actuator_id])
        if not math.isfinite(target) or target < low - 1e-10 or target > high + 1e-10:
            raise ValueError(f"wrench_stop_target_out_of_range:{name}")
        if actuator_id in updates:
            raise ValueError(f"wrench_stop_duplicate_actuator:{name}")
        updates[actuator_id] = target
    next_ctrl = list(before)
    for actuator_id, target in updates.items():
        next_ctrl[actuator_id] = target
    safe_before = INST.safe_write_count
    ordinary_before = INST.ctrl_write_count
    _set_ctrl(next_ctrl, write=False, safe=True)
    changed_outside_scope = any(
        index not in updates and float(d.ctrl[index]) != before[index]
        for index in range(len(before))
    )
    verified = (
        not changed_outside_scope
        and INST.safe_write_count == safe_before + 1
        and INST.ctrl_write_count == ordinary_before
        and all(abs(float(d.ctrl[index]) - target) <= 1e-10 for index, target in updates.items())
        and len(updates) == len(actuator_names)
    )
    return verified, actuator_names


def _trip_wrench_monitor(
    reason: dict[str, Any], policy: dict[str, Any], triggered_at_s: float
) -> dict[str, Any]:
    contract = policy["abort_contract"]
    write_before = INST.safe_write_count
    stopped_at_s = float(INST.data.time)
    stop_verified = False
    stop_actuator_ids: list[str] = []
    stop_error: str | None = None
    try:
        stop_verified, stop_actuator_ids = _hold_current_position(policy)
    except Exception as error:
        stop_verified = False
        stop_error = f"{type(error).__name__}:{error}"
    stopped_at_s = float(INST.data.time)
    write_count = INST.safe_write_count - write_before
    stop_verified = bool(
        stop_verified
        and write_count <= int(contract["max_stop_writes"])
        and stopped_at_s - triggered_at_s <= float(contract["max_time_to_stop_s"]) + 1e-12
    )
    INST.wrench_abort_latched = True
    return {
        "reason": reason,
        "triggered_at_s": triggered_at_s,
        "stopped_at_s": stopped_at_s,
        "stop_write_count": write_count,
        "contract_id": str(contract.get("contract_id") or ""),
        "stop_verified": stop_verified,
        "stop_actuator_ids": stop_actuator_ids,
        "stop_error": stop_error,
    }


def _monitor_reason(
    sample: dict[str, Any] | None,
    previous_capture_s: float | None,
    now_s: float,
    monitor_started_s: float,
    policy: dict[str, Any],
    profile: dict[str, Any],
    gravity_reference: dict[str, Any] | None,
) -> dict[str, Any] | None:
    if sample is None:
        if now_s - monitor_started_s > float(policy["max_sample_age_s"]):
            return {"kind": "missing_sample"}
        return None
    capture_s = float(sample["capture_s"])
    receive_s = float(sample["receive_s"])
    age_s = now_s - capture_s
    if receive_s > now_s or capture_s > now_s:
        return None
    if not math.isfinite(age_s) or age_s > float(policy["max_sample_age_s"]):
        return {"kind": "stale_sample"}
    if previous_capture_s is not None:
        gap_s = capture_s - previous_capture_s
        if gap_s <= 0.0 or gap_s > float(policy["max_inter_sample_gap_s"]):
            return {"kind": "gap_exceeded"}
    try:
        if not _finite_six(sample.get("wrench")) or not _finite_six(sample.get("error_bounds")):
            return {"kind": "monitor_fault"}
        scope = policy["scope"]
        compensated_wrench, compensated_error = _compensate_wrench_with_reference(
            sample, profile, gravity_reference
        )
        for axis in range(6):
            upper = (
                abs(compensated_wrench[axis])
                + compensated_error[axis]
                + float(policy["max_rise_rate"][axis])
                * (age_s + float(policy["max_sensing_latency_s"]) + float(policy["max_abort_latency_s"]))
            )
            if not math.isfinite(upper):
                return {"kind": "monitor_fault"}
            if upper >= float(scope["caps"][axis]):
                return {
                    "kind": "limit",
                    "axis": axis,
                    "measured_upper_bound": upper,
                    "cap": float(scope["caps"][axis]),
                }
    except Exception:
        return {"kind": "monitor_fault"}
    return None


def _load(msg: dict[str, Any]) -> dict[str, Any]:
    _drop_runtime(True)
    INST.ctrl_write_count = 0
    INST.safe_write_count = 0
    INST.last_ctrl = []
    INST.warnings = []
    INST.lost_features = []
    INST.usd_experimental = False
    INST.asset_names = []
    INST.mesh_asset_by_id = {}
    INST.wrench_sequence_by_profile = {}
    INST.wrench_step_by_profile = {}
    INST.wrench_latest_by_profile = {}
    INST.wrench_pending_by_profile = {}
    INST.wrench_last_received_capture_by_profile = {}
    INST.wrench_monitor_started_by_profile = {}
    INST.wrench_gravity_reference_by_profile = {}
    INST.wrench_free_space_by_profile = {}
    INST.wrench_abort_latched = False
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
    seed = int(msg.get("seed", 0))
    INST.load_seed = seed
    wrench_sensor_frame = msg.get("wrench_sensor_frame")
    if not extras and wrench_sensor_frame is None and path in _BASE_MODEL:
        INST.model = _BASE_MODEL[path]
        INST.mesh_asset_by_id = dict(_BASE_MESH_ASSETS.get(path, {}))
        INST.data = mujoco.MjData(INST.model)
        INST.data.qpos[:] = INST.model.qpos0
        INST.data.qvel[:] = 0
        mujoco.mj_forward(INST.model, INST.data)
        INST.last_ctrl = [float(x) for x in INST.data.ctrl]
        inspect = dict(_BASE_INSPECT[path])
        inspect["load_seed"] = seed
        inspect["model_cache"] = "hit"
        return {
            "ok": True,
            "inspect": inspect,
            "state": INST.state(),
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
        }
    try:
        spec = _load_spec(path, assets)
        _add_scenario_objects(spec, extras)
        if wrench_sensor_frame is not None:
            _add_wrench_sensors(spec, str(wrench_sensor_frame))
        meshdir = str(getattr(spec.compiler, "meshdir", "") or "")
        for mesh in getattr(spec, "meshes", []) or []:
            mesh_id = int(getattr(mesh, "id", -1))
            mesh_file = str(getattr(mesh, "file", "") or "")
            if mesh_id >= 0 and mesh_file:
                asset_path = os.path.normpath(os.path.join(meshdir, mesh_file))
                INST.mesh_asset_by_id[mesh_id] = asset_path.replace("\\", "/")
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
    inspect["model_cache"] = "miss"
    if not extras and wrench_sensor_frame is None:
        _BASE_MODEL[path] = INST.model
        _BASE_INSPECT[path] = dict(inspect)
        _BASE_MESH_ASSETS[path] = dict(INST.mesh_asset_by_id)
    else:
        gc.collect()
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
    if INST.wrench_abort_latched and (
        cmd in {"set_ctrl", "set_safe_ctrl", "step_with_wrench_profile", "step_monitored"}
        or (cmd == "step" and int(msg.get("n", 1)) > 0)
    ):
        return {"ok": False, "error": "WRENCH_ABORT_LATCHED", "metal": False}
    if cmd == "inspect":
        return {"ok": True, "inspect": INST.inspect()}
    if cmd == "local_linear":
        return _local_linear()
    if cmd == "reset":
        mujoco.mj_resetData(INST.model, INST.data)
        if msg.get("keyframe") is not None and INST.model.nkey > 0:
            k = int(msg["keyframe"])
            k = max(0, min(k, INST.model.nkey - 1))
            kq = INST.model.key_qpos[k]
            nq = min(int(kq.shape[0]), int(INST.model.nq))
            INST.data.qpos[:nq] = kq[:nq]
            if hasattr(INST.model, "key_ctrl"):
                kc = INST.model.key_ctrl[k]
                nu = min(int(kc.shape[0]), int(INST.model.nu))
                INST.data.ctrl[:nu] = kc[:nu]
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
        # A randomized benchmark reset places the robot at an observed pose.
        # Initialize position-servo targets to that same pose so the first
        # monitored step does not create an artificial startup transient.
        for actuator_id in range(int(INST.model.nu)):
            joint_id = int(INST.model.actuator_trnid[actuator_id][0])
            if (
                int(INST.model.actuator_trntype[actuator_id]) not in (0, 1)
                or int(INST.model.actuator_biastype[actuator_id]) != 1
                or joint_id < 0
                or joint_id >= INST.model.njnt
                or int(INST.model.jnt_type[joint_id]) not in (2, 3)
                or not bool(INST.model.actuator_ctrllimited[actuator_id])
            ):
                continue
            qpos_address = int(INST.model.jnt_qposadr[joint_id])
            value = float(INST.data.qpos[qpos_address])
            lower, upper = INST.model.actuator_ctrlrange[actuator_id]
            if float(lower) <= value <= float(upper):
                INST.data.ctrl[actuator_id] = value
        INST.last_ctrl = [float(value) for value in INST.data.ctrl]
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
    if cmd == "step_monitored":
        profile = msg.get("profile")
        policy = msg.get("policy")
        profile_digest = str(msg.get("profile_digest") or "").lower()
        validated = False
        samples: list[dict[str, Any]] = []
        control_correction_max_rad = 0.0
        precontact_baseline_record = None
        precontact_baseline_status = None
        monitor_gravity_reference_status = None
        gravity_reference = None
        try:
            if not isinstance(profile, dict) or not isinstance(policy, dict):
                raise ValueError("wrench_monitor_profile_missing")
            if profile.get("limit_policy") != policy:
                raise ValueError("wrench_monitor_policy_mismatch")
            step_interval = _wrench_profile_steps(profile, profile_digest, float(INST.model.opt.timestep))
            _validate_wrench_abort_contract(policy)
            _validate_monitored_grant(msg, policy)
            # Resolve the sensor and every stop target before advancing physics.
            sensor_ids = _wrench_sensor_ids(profile)
            sensor_subtree_body_ids = _sensor_subtree_body_ids(sensor_ids)
            expected_contact_name = msg.get("expected_contact_body")
            expected_contact_body_id = None
            if expected_contact_name is not None:
                if not isinstance(expected_contact_name, str) or not expected_contact_name.strip():
                    raise ValueError("wrench_expected_contact_body_invalid")
                expected_contact_body_id = int(
                    mujoco.mj_name2id(
                        INST.model, mujoco.mjtObj.mjOBJ_BODY, expected_contact_name
                    )
                )
                if expected_contact_body_id <= 0 or expected_contact_body_id in sensor_subtree_body_ids:
                    raise ValueError("wrench_expected_contact_body_unknown_or_instrumented")
            _hold_target_names = _validate_wrench_abort_contract(policy)
            for actuator_name in _hold_target_names:
                actuator_id = int(mujoco.mj_name2id(INST.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name))
                joint_id = int(INST.model.actuator_trnid[actuator_id][0])
                if (
                    int(INST.model.actuator_trntype[actuator_id]) not in (0, 1)
                    or int(INST.model.actuator_biastype[actuator_id]) != 1
                    or joint_id < 0
                    or joint_id >= INST.model.njnt
                    or int(INST.model.jnt_type[joint_id]) not in (2, 3)
                    or not bool(INST.model.actuator_ctrllimited[actuator_id])
                ):
                    raise ValueError(f"wrench_stop_actuator_invalid:{actuator_name}")
            n = int(msg.get("n", 0))
            if n <= 0 or n > 100_000:
                raise ValueError("wrench_step_count")
            control_ramp = _validate_monitored_control_ramp(msg, policy, n)
            validated = True
            profile_digest = profile_digest.lower()
            sequence = int(INST.wrench_sequence_by_profile.get(profile_digest, 0))
            step_count = int(INST.wrench_step_by_profile.get(profile_digest, 0))
            latest = INST.wrench_latest_by_profile.get(profile_digest)
            pending = list(INST.wrench_pending_by_profile.get(profile_digest, []))
            gravity_reference = INST.wrench_gravity_reference_by_profile.get(profile_digest)
            previous_capture_s = INST.wrench_last_received_capture_by_profile.get(profile_digest)
            monitor_started_s = float(INST.wrench_monitor_started_by_profile.setdefault(profile_digest, INST.data.time))
            drop_wrench_samples = bool(msg.get("drop_wrench_samples", False))
            request_precontact_baseline = bool(msg.get("request_precontact_baseline", False))
            request_monitor_gravity_reference = bool(
                msg.get("request_monitor_gravity_reference", False)
            )
            if (
                not drop_wrench_samples
                and not request_precontact_baseline
                and not request_monitor_gravity_reference
                and latest is None
                and not pending
                and sequence == 0
                and step_count == 0
            ):
                mujoco.mj_forward(INST.model, INST.data)
                initial = _sample_tool_wrench(profile, profile_digest, sequence, sensor_ids)
                samples.append(initial)
                pending.append(initial)
                if not _sensor_subtree_has_contact(sensor_ids) and _sensor_is_quiescent(sensor_ids):
                    gravity_reference = dict(initial)
                sequence += 1
            if not drop_wrench_samples and request_monitor_gravity_reference:
                if _sensor_subtree_has_contact(sensor_ids):
                    monitor_gravity_reference_status = "contact_present"
                elif not _sensor_is_quiescent(sensor_ids):
                    monitor_gravity_reference_status = "sensor_not_quiescent"
                else:
                    # An explicit request refreshes the reference for the
                    # current settled motion phase. A home-pose reference can
                    # be stale after the arm moves to its pre-contact pose.
                    monitor_gravity_reference_status = "captured"
                    reference_candidates = list(pending)
                    if latest is not None:
                        reference_candidates.append(latest)
                    reference_record = next(
                        (
                            candidate
                            for candidate in reversed(reference_candidates)
                            if abs(float(candidate["capture_s"]) - float(INST.data.time)) <= 1e-12
                        ),
                        None,
                    )
                    if reference_record is None:
                        reference_record = _sample_tool_wrench(
                            profile, profile_digest, sequence, sensor_ids
                        )
                        samples.append(reference_record)
                        pending.append(reference_record)
                        sequence += 1
                    gravity_reference = dict(reference_record)
            if not drop_wrench_samples and request_precontact_baseline:
                mujoco.mj_forward(INST.model, INST.data)
                if _sensor_subtree_has_contact(sensor_ids):
                    precontact_baseline_status = "contact_present"
                elif not _sensor_is_quiescent(sensor_ids):
                    precontact_baseline_status = "sensor_not_quiescent"
                else:
                    precontact_baseline_status = "captured"
                    baseline_candidates = list(pending)
                    if latest is not None:
                        baseline_candidates.append(latest)
                    baseline_record = next(
                        (
                            candidate
                            for candidate in reversed(baseline_candidates)
                            if abs(float(candidate["capture_s"]) - float(INST.data.time)) <= 1e-12
                        ),
                        None,
                    )
                    if baseline_record is None:
                        baseline_record = _sample_tool_wrench(
                            profile, profile_digest, sequence, sensor_ids
                        )
                        samples.append(baseline_record)
                        pending.append(baseline_record)
                        sequence += 1
                    precontact_baseline_record = dict(baseline_record)
                    if gravity_reference is None:
                        gravity_reference = dict(baseline_record)
            abort = None
            for step_offset in range(n):
                if control_ramp is not None:
                    progress = (
                        control_ramp["elapsed_steps"] + step_offset + 1
                    ) / control_ramp["total_steps"]
                    eased_progress = progress * progress * (3.0 - 2.0 * progress)
                    ramped_ctrl = [
                        float(start) + (float(end) - float(start)) * eased_progress
                        for start, end in zip(
                            control_ramp["start_ctrl"], control_ramp["target_ctrl"]
                        )
                    ]
                    correction_bound = float(
                        policy.get("max_position_tracking_correction_rad", 0.0)
                    )
                    if correction_bound > 0.0:
                        for actuator_id in control_ramp["tracking_actuator_ids"]:
                            joint_id = int(INST.model.actuator_trnid[actuator_id][0])
                            qpos_address = int(INST.model.jnt_qposadr[joint_id])
                            desired = ramped_ctrl[actuator_id]
                            measured = float(INST.data.qpos[qpos_address])
                            correction = max(
                                -correction_bound,
                                min(
                                    correction_bound,
                                    float(policy["position_tracking_correction_gain"])
                                    * (desired - measured),
                                ),
                            )
                            low, high = (
                                float(value)
                                for value in INST.model.actuator_ctrlrange[actuator_id]
                            )
                            corrected = max(low, min(high, desired + correction))
                            control_correction_max_rad = max(
                                control_correction_max_rad, abs(corrected - desired)
                            )
                            ramped_ctrl[actuator_id] = corrected
                    _set_ctrl(ramped_ctrl, write=True)
                mujoco.mj_step(INST.model, INST.data)
                if not _finite(INST.data.qpos) or not _finite(INST.data.qvel):
                    abort = _trip_wrench_monitor({"kind": "monitor_fault"}, policy, float(INST.data.time))
                    break
                step_count += 1
                now_s = float(INST.data.time)
                saturated_actuator_ids = _sensor_subtree_actuator_saturations(sensor_ids)
                if saturated_actuator_ids:
                    abort = _trip_wrench_monitor(
                        {
                            "kind": "actuator_saturation",
                            "actuator_ids": saturated_actuator_ids,
                        },
                        policy,
                        now_s,
                    )
                    break
                if _has_unexpected_sensor_subtree_contact(
                    sensor_subtree_body_ids, expected_contact_body_id
                ):
                    abort = _trip_wrench_monitor(
                        {"kind": "unexpected_contact"}, policy, now_s
                    )
                    break
                if not drop_wrench_samples and step_count % step_interval == 0:
                    sampled = _sample_tool_wrench(profile, profile_digest, sequence, sensor_ids)
                    samples.append(sampled)
                    pending.append(sampled)
                    sequence += 1
                newly_available = []
                still_pending = []
                for candidate in pending:
                    if float(candidate["receive_s"]) <= now_s + 1e-12:
                        newly_available.append(candidate)
                    else:
                        still_pending.append(candidate)
                pending = still_pending
                for candidate in newly_available:
                    candidate_capture = float(candidate["capture_s"])
                    if previous_capture_s is not None:
                        candidate_gap = candidate_capture - previous_capture_s
                        if candidate_gap <= 0.0 or candidate_gap > float(policy["max_inter_sample_gap_s"]):
                            abort = _trip_wrench_monitor({"kind": "gap_exceeded"}, policy, now_s)
                            break
                    previous_capture_s = candidate_capture
                    latest = candidate
                if abort is not None:
                    break
                if (
                    not drop_wrench_samples
                    and request_monitor_gravity_reference
                    and monitor_gravity_reference_status == "sensor_not_quiescent"
                    and latest is not None
                    and step_count % step_interval == 0
                ):
                    if _sensor_subtree_has_contact(sensor_ids):
                        monitor_gravity_reference_status = "contact_present"
                    elif _sensor_is_quiescent(sensor_ids):
                        # A batched monitored settle may begin while gravity
                        # dynamics are still moving the arm. Capture only once
                        # a received sample coincides with a contact-free,
                        # quiescent sensor subtree.
                        monitor_gravity_reference_status = "captured"
                        gravity_reference = dict(latest)
                if (
                    not drop_wrench_samples
                    and request_precontact_baseline
                    and precontact_baseline_status == "sensor_not_quiescent"
                    and latest is not None
                    and step_count % step_interval == 0
                ):
                    if _sensor_subtree_has_contact(sensor_ids):
                        precontact_baseline_status = "contact_present"
                    elif _sensor_is_quiescent(sensor_ids):
                        precontact_baseline_status = "captured"
                        precontact_baseline_record = dict(latest)
                        if gravity_reference is None:
                            gravity_reference = dict(latest)
                reason = _monitor_reason(
                    latest,
                    None,
                    now_s,
                    monitor_started_s,
                    policy,
                    profile,
                    gravity_reference,
                )
                if reason is not None:
                    abort = _trip_wrench_monitor(reason, policy, now_s)
                    break
            if (
                abort is not None
                and abort["reason"].get("kind") == "limit"
                and latest is not None
            ):
                abort["triggering_sensor_position"] = latest.get("sensor_position")
                abort["triggering_sensor_orientation"] = latest.get("sensor_quaternion")
            INST.wrench_sequence_by_profile[profile_digest] = sequence
            INST.wrench_step_by_profile[profile_digest] = step_count
            INST.wrench_latest_by_profile[profile_digest] = latest
            INST.wrench_pending_by_profile[profile_digest] = pending
            INST.wrench_last_received_capture_by_profile[profile_digest] = previous_capture_s
            if gravity_reference is not None:
                INST.wrench_gravity_reference_by_profile[profile_digest] = gravity_reference
            state = INST.state()
            return {
                "ok": True,
                "state": state,
                "samples": samples,
                "sensor_id": str(profile["sensor_id"]),
                "profile_digest": profile_digest,
                "precontact_baseline": precontact_baseline_record,
                "precontact_baseline_status": precontact_baseline_status,
                "monitor_gravity_reference_status": monitor_gravity_reference_status,
                "triggering_sample": (
                    latest
                    if abort is not None and abort["reason"].get("kind") == "limit"
                    else None
                ),
                "monitor_gravity_reference": gravity_reference,
                "control_correction_max_rad": control_correction_max_rad,
                "evidence_status": EVIDENCE_STATUS,
                "abort": abort,
                "metal": False,
            }
        except Exception as exc:
            if not validated:
                return {"ok": False, "error": f"wrench_monitor:{exc}", "metal": False}
            try:
                fault = _trip_wrench_monitor({"kind": "monitor_fault"}, policy, float(INST.data.time))
                return {
                    "ok": True,
                    "state": INST.state(),
                    "samples": samples,
                    "sensor_id": str(profile["sensor_id"]),
                    "profile_digest": profile_digest,
                    "precontact_baseline": precontact_baseline_record,
                    "precontact_baseline_status": precontact_baseline_status,
                    "monitor_gravity_reference_status": monitor_gravity_reference_status,
                    "triggering_sample": (
                        latest
                        if fault is not None and fault["reason"].get("kind") == "limit"
                        else None
                    ),
                    "monitor_gravity_reference": gravity_reference,
                    "control_correction_max_rad": control_correction_max_rad,
                    "evidence_status": EVIDENCE_STATUS,
                    "abort": fault,
                    "monitor_error": str(exc),
                    "metal": False,
                }
            except Exception as stop_error:
                INST.wrench_abort_latched = True
                return {"ok": False, "error": f"wrench_monitor_fault:{exc};stop:{stop_error}", "metal": False}
    if cmd == "step_with_wrench_profile":
        try:
            profile = msg.get("profile")
            profile_digest = str(msg.get("profile_digest") or "")
            sample_interval_steps = _wrench_profile_steps(
                profile, profile_digest, float(INST.model.opt.timestep)
            )
            sensor_ids = _wrench_sensor_ids(profile)
            profile_digest = profile_digest.lower()
            n = int(msg.get("n", 1))
            if n < 0 or n > 100_000:
                raise ValueError("wrench_step_count")
            sequence = int(INST.wrench_sequence_by_profile.get(profile_digest, 0))
            step_count = int(INST.wrench_step_by_profile.get(profile_digest, 0))
            samples = []
            for _ in range(n):
                mujoco.mj_step(INST.model, INST.data)
                if not _finite(INST.data.qpos) or not _finite(INST.data.qvel):
                    return {"ok": False, "error": "wrench_step_divergent", "metal": False}
                step_count += 1
                if step_count % sample_interval_steps == 0:
                    samples.append(
                        _sample_tool_wrench(profile, profile_digest, sequence, sensor_ids)
                    )
                    sequence += 1
            INST.wrench_sequence_by_profile[profile_digest] = sequence
            INST.wrench_step_by_profile[profile_digest] = step_count
            return {
                "ok": True,
                "samples": samples,
                "sensor_id": str(profile["sensor_id"]),
                "profile_digest": profile_digest,
                "evidence_status": EVIDENCE_STATUS,
                "metal": False,
            }
        except Exception as exc:
            return {"ok": False, "error": f"wrench_sensor:{exc}", "metal": False}
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
    if cmd == "configure_body":
        body = str(msg.get("body") or "")
        found = None
        for i in range(INST.model.nbody):
            if INST.model.body(i).name == body:
                found = i
                break
        if found is None:
            return {"ok": False, "error": f"unknown_body:{body}"}
        consts_changed = False
        if msg.get("mass") is not None:
            INST.model.body_mass[found] = float(msg["mass"])
            consts_changed = True
        geoms = [
            g
            for g in range(INST.model.ngeom)
            if int(INST.model.geom_bodyid[g]) == found
        ]
        if msg.get("friction") is not None and geoms:
            fr = float(msg["friction"])
            for g in geoms:
                INST.model.geom_friction[g][0] = fr
                INST.model.geom_friction[g][1] = fr
                INST.model.geom_friction[g][2] = 0.01
        if msg.get("size") is not None and geoms:
            size = list(msg["size"])
            for g in geoms:
                for k in range(min(3, len(size))):
                    INST.model.geom_size[g][k] = float(size[k])
            consts_changed = True
        if consts_changed and hasattr(mujoco, "mj_setConst"):
            qpos_save = INST.data.qpos.copy()
            qvel_save = INST.data.qvel.copy()
            ctrl_save = INST.data.ctrl.copy()
            mujoco.mj_setConst(INST.model, INST.data)
            INST.data.qpos[:] = qpos_save
            INST.data.qvel[:] = qvel_save
            INST.data.ctrl[:] = ctrl_save
        if msg.get("hide"):
            pos = [8.0, 8.0, -1.0]
        else:
            pos = msg.get("pos")
        if pos is not None:
            jnt = int(INST.model.body_jntadr[found])
            if jnt >= 0 and int(INST.model.jnt_type[jnt]) == 0:
                adr = int(INST.model.jnt_qposadr[jnt])
                INST.data.qpos[adr : adr + 3] = pos
            else:
                INST.model.body_pos[found][:] = pos
        mujoco.mj_forward(INST.model, INST.data)
        return {"ok": True, "state": INST.state()}
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
    if cmd == "gravity_oracle":
        # Privileged post-hoc gravity generalized force. Never a predictor input.
        m, d = INST.model, INST.data
        if "qpos" in msg:
            q = msg["qpos"]
            if len(q) != m.nq:
                return {"ok": False, "error": f"qpos_dim:{len(q)}!={m.nq}"}
            d.qpos[:] = q
        d.qvel[:] = 0
        d.qacc[:] = 0
        d.ctrl[:] = 0
        mujoco.mj_fwdPosition(m, d)
        mujoco.mj_fwdVelocity(m, d)
        joints = []
        for i in range(m.njnt):
            j = m.joint(i)
            dof = int(m.jnt_dofadr[i])
            qadr = int(m.jnt_qposadr[i])
            jtype = int(m.jnt_type[i])
            qv = float(d.qpos[qadr]) if jtype in (2, 3) else None
            tau = float(d.qfrc_bias[dof]) if 0 <= dof < m.nv else None
            joints.append(
                {
                    "name": j.name or f"joint_{i}",
                    "dofadr": dof,
                    "qposadr": qadr,
                    "type": jtype,
                    "q": qv,
                    "qfrc_bias": tau,
                }
            )
        xipos = {}
        for i in range(m.nbody):
            name = m.body(i).name or f"body_{i}"
            xipos[name] = [float(x) for x in d.xipos[i]]
        return {
            "ok": True,
            "privileged": True,
            "predictor_forbidden": True,
            "qfrc_bias": [float(x) for x in d.qfrc_bias],
            "gravity": [float(x) for x in m.opt.gravity],
            "joints": joints,
            "xipos": xipos,
            "note": "post_hoc_oracle_not_for_predictor",
            "metal": False,
            "evidence_status": EVIDENCE_STATUS,
        }
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
