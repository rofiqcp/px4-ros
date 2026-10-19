# Git synchronization: px4-ros

Repository: https://github.com/rofiqcp/px4-ros

- Sirobo: /media/sirobo/HDD/Project/px4-ros
- MiniPC1: /home/otomasi/Project/px4-ros
- Shared default branch: v1

Before editing:

    git switch v1
    git pull --ff-only origin v1

After editing and reviewing changes:

    git status --short
    git add <paths-to-source-and-docs>
    git commit -m "Describe the change"
    git push origin v1

Run git pull --ff-only origin v1 on the other computer.
Captured data, ROS builds, logs, backups, credentials and local host
artifacts are deliberately not synchronized. Avoid simultaneous edits
to the same files without pulling first.
