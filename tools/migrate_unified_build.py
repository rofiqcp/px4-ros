#!/usr/bin/env python3
"""Legacy one-time migration is no longer necessary: build wrappers are location-aware."""
from pathlib import Path

def main():
    root = Path(__file__).resolve().parents[1]
    print("Workspace already portable:", root)
    print("For ROS-PX4 build:", root / "build.sh")
    print("For combined LiDAR and PX4 build:", root / "tools/build_unified.sh")

if __name__ == "__main__":
    main()
