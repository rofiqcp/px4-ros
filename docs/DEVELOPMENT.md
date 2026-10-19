# Development and repository hygiene

## Portable build

```bash
cd /path/to/px4-ros
./build.sh
source ./env.sh
./verify.sh
```

Build with a valid ROS 2 Humble installation. The wrappers place artifacts in a local cache. Do not copy `build/`, `install/`, `log/`, dataset folders or per-host ROS workspace overlays into Git.

## Before a commit

```bash
python3 tools/check_repo_hygiene.py
bash -n build.sh env.sh run.sh verify.sh tools/build_unified.sh tools/env_unified.sh
git status --short
git diff --check
git diff --cached --stat
```

Check the complete diff and confirm the proposed changes do not enable PX4 actuator/motion commands or alter safety defaults. A successful syntax check is not a substitute for a ROS build or an actual safety test.

## Documentation

Maintain one current `README.md`, one `AGENTS.md`, and a small set of topic docs. Keep previous logs, hardware verification evidence, historical audits, one-off scripts and exploratory notebooks in ignored `.local-only/` or outside this repository. Never mark a historical setup as currently verified without rerunning its check.

For two-way Git synchronization see [../SYNC_GUIDE.md](../SYNC_GUIDE.md).
