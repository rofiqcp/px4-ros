# Synchronize `px4-ros` on two Linux PCs

- GitHub (public): https://github.com/rofiqcp/px4-ros
- Branch: `v1` (GitHub default)
- Sirobo: `/media/sirobo/HDD/Project/px4-ros`
- MiniPC1: `/home/otomasi/Project/px4-ros`

On **either** machine before editing:

```bash
git status --short
git switch v1
git pull --ff-only origin v1
```

After modifying maintained source and passing checks:

```bash
python3 tools/check_repo_hygiene.py
git status --short
git add README.md AGENTS.md docs/ src/ tools/   # adjust to actual intended files
git diff --cached --stat
git commit -m "Describe the reviewed change"
git push origin v1
```

Then run `git pull --ff-only origin v1` on the other machine. Never run `git reset --hard`, `git clean -fdx` or force push to resolve local differences. Stop and preserve uncommitted changes first. Ignored local experiments, builds, data captures and historical documents do not sync through Git.
