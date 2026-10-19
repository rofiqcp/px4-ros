#!/usr/bin/env python3
"""Check tracked files for accidentally committed experimental or private artifacts."""
from __future__ import annotations

import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
tracked = subprocess.check_output(["git", "-C", str(ROOT), "ls-files", "-z"])
paths = [pathlib.PurePosixPath(raw.decode()) for raw in tracked.split(b"\0") if raw]
bad = []
for path in paths:
    name = path.name.lower()
    parts = {part.lower() for part in path.parts}
    reason = None
    if any(part in parts for part in (".local-only", "simulation", "results", "backups", "datasets", "build", "install", "log", "__pycache__")):
        reason = "generated, local-only or experimental folder"
    elif path.parts[0] == "px4_firmware":
        reason = "firmware patch experiments live in separate repository"
    elif len(path.parts) >= 2 and path.parts[0] in ("scripts", "tools") and (name.startswith("test_") or name.startswith(("patch_", "migrate_")) or name == "sim_bench.py"):
        reason = "one-off test or patch script"
    elif re.search(r"20\d{6}\.md$", name) or ("audit" in name and name.endswith(".md")) or name.startswith("agent_next_") or name.startswith("test_report_"):
        reason = "dated audit report, not maintained docs"
    elif name.startswith(".env") and name != ".env.example":
        reason = "private configuration"
    elif name.endswith((".bag", ".db3", ".mcap", ".ulg", ".tlog", ".pcd", ".ply", ".pem", ".p12", ".pfx", ".log")):
        reason = "dataset, secret or log file"
    if reason:
        bad.append((str(path), reason))
if bad:
    for path, reason in bad:
        print(f"ERROR: {path}: {reason}", file=sys.stderr)
    sys.exit(1)
print(f"OK: {len(paths)} tracked files conform to the repository policy")
