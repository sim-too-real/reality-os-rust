#!/usr/bin/env python3
"""Pinned fetch of official DeepMind MuJoCo Menagerie models. Not a fork copy."""

from __future__ import annotations

import hashlib
import json
import sys
import tarfile
import time
import urllib.request
from pathlib import Path

REPO = "https://github.com/google-deepmind/mujoco_menagerie"
SHA = "8161bba264d7fa7c99ca301e91e7fb44737676ad"
TARBALL = f"{REPO}/archive/{SHA}.tar.gz"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def fetch(rel_dir: str, dest: Path) -> dict:
    dest.mkdir(parents=True, exist_ok=True)
    marker = dest / "PROVENANCE.json"
    if marker.exists() and any(dest.glob("*.xml")):
        return json.loads(marker.read_text(encoding="utf-8"))
    cache = dest.parent.parent / ".cache"
    if not cache.exists():
        cache = dest.parent / ".cache"
    cache.mkdir(parents=True, exist_ok=True)
    tar_path = cache / f"menagerie-{SHA}.tar.gz"
    if not tar_path.exists():
        urllib.request.urlretrieve(TARBALL, tar_path)
    prefix = f"mujoco_menagerie-{SHA}/{rel_dir}/"
    with tarfile.open(tar_path, "r:gz") as tf:
        for member in tf.getmembers():
            name = member.name.replace("\\", "/")
            if not name.startswith(prefix):
                continue
            rel = name[len(prefix) :]
            if not rel or member.isdir():
                if member.isdir() and rel:
                    (dest / rel).mkdir(parents=True, exist_ok=True)
                continue
            member.name = rel
            tf.extract(member, dest)
    files = {}
    for p in dest.rglob("*"):
        if p.is_file() and p.name != "PROVENANCE.json":
            files[str(p.relative_to(dest)).replace("\\", "/")] = sha256_file(p)
    prov = {
        "source_repository": REPO,
        "source_commit": SHA,
        "model_path": rel_dir,
        "tarball_sha256": sha256_file(tar_path),
        "files": files,
        "import_timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    marker.write_text(json.dumps(prov, indent=2) + "\n", encoding="utf-8")
    return prov


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: fetch_menagerie.py <rel_dir> <dest>", file=sys.stderr)
        return 2
    rel_dir = sys.argv[1]
    dest = Path(sys.argv[2])
    prov = fetch(rel_dir, dest)
    print(json.dumps({"ok": True, "source_commit": prov["source_commit"], "files": len(prov["files"])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
