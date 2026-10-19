#!/usr/bin/env python3
"""PX4 1.17 Differential + Gazebo automated missions; simulation ONLY."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path("/media/sirobo/HDD/pixhawk")
SRC = Path("/media/sirobo/HDD/autopilot/PX4-Autopilot-v1.17.0")
SIM = ROOT / "simulation/cube2_px4_gazebo"
RESULT = SIM / "results"
BIN = SRC / "build/px4_sitl_default/bin/px4"
MODEL = SRC / "Tools/simulation/gz/models/rover_differential/model.sdf"
WORLD = SRC / "Tools/simulation/gz/worlds/rover.sdf"
ROUTES = ("line2", "corner3", "square4", "zigzag5")


def sim_pids():
    output = subprocess.check_output(["ps", "-eo", "pid=,args="], text=True)
    px4, gz = [], []
    expected_gz = f"gz sim --verbose=1 -r -s {WORLD}"
    for line in output.splitlines():
        split = line.strip().split(maxsplit=1)
        if len(split) != 2 or not split[0].isdigit():
            continue
        pid, args = int(split[0]), split[1]
        if args == str(BIN):
            px4.append(pid)
        if args == expected_gz:
            gz.append(pid)
    return px4, gz


def disarm_sim():
    if sim_pids()[0]:
        subprocess.run(
            [str(BIN.parent / "px4-commander"), "disarm"],
            cwd=BIN.parent, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, timeout=5, check=False)


def clean_sitl(make_process, known_px4, known_gz):
    disarm_sim()
    present_px4, present_gz = sim_pids()
    for pid in known_px4:
        if pid in present_px4:
            os.kill(pid, signal.SIGTERM)
    for pid in known_gz:
        if pid in present_gz:
            os.kill(pid, signal.SIGTERM)
    # Reap only the process group created by this regression.
    if make_process is not None and make_process.poll() is None:
        os.killpg(make_process.pid, signal.SIGTERM)
        try:
            make_process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(make_process.pid, signal.SIGKILL)
            make_process.wait(timeout=3)


def verify_cube_guards():
    # This call invokes only real-Cube read-only queries and local denial tests.
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "0"
    with open(RESULT / "cube_readonly_preflight.log", "w") as log:
        probe = subprocess.run(
            [sys.executable, str(ROOT / "scripts/probe_cube_usb_safe.py")],
            cwd=ROOT, env=env, stdin=subprocess.DEVNULL,
            stdout=log, stderr=subprocess.STDOUT, timeout=45, check=False)
    if probe.returncode != 0:
        raise RuntimeError("Physical Cube write-lock preflight FAILED; stop")
    status = json.loads(
        (ROOT / "results/cube_usb_mission_goto_readonly_20261009.json").read_text())
    if (status["observations"]["final_status"]["armed"] or
        any(status["write_permissions"].values())):
        raise RuntimeError("Real Cube is armed or permits writes")
    print("HARDWARE_CUBE_READONLY_GUARD_PASS", flush=True)


def run(routes):
    if not BIN.is_file() or not MODEL.is_file() or not WORLD.is_file():
        raise RuntimeError("Missing PX4 v1.17 SITL/Gazebo binary or model")
    if any(sim_pids()):
        raise RuntimeError("Existing PX4/Gazebo rover session found; will not commandeer it")
    RESULT.mkdir(parents=True, exist_ok=True)
    verify_cube_guards()
    env = os.environ.copy()
    env["HEADLESS"] = "1"
    env["ROS_DOMAIN_ID"] = "105"
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(
        [str(SRC / "Tools/simulation/gz/models"),
         str(SRC / "Tools/simulation/gz/worlds")])
    log = open(RESULT / "px4_gazebo.log", "w")
    proc = subprocess.Popen(
        ["make", "px4_sitl", "gz_rover_differential"],
        cwd=SRC, env=env, stdin=subprocess.DEVNULL,
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    log.close()
    owned_px4, owned_gz = [], []
    report = {"simulation":"PX4 v1.17 Differential + Gazebo Harmonic",
              "physical_cube_motor_commands":False, "physical_cube_firmware_changes":False,
              "ros_domain_id":105,"model":str(MODEL),"routes":{}}
    summary_path = RESULT / ("summary.json" if len(routes)==len(ROUTES)
                             else "summary_" + "_".join(routes) + ".json")
    try:
        deadline = time.monotonic() + 55
        while time.monotonic() < deadline:
            owned_px4, owned_gz = sim_pids()
            logtext = (RESULT / "px4_gazebo.log").read_text(errors="replace")
            if (len(owned_px4) == 1 and len(owned_gz) == 1 and
                "Startup script returned successfully" in logtext):
                break
            if proc.poll() is not None:
                raise RuntimeError("SITL command failed, log: " + logtext[-3000:])
            time.sleep(0.5)
        else:
            raise RuntimeError("PX4 and Gazebo not both ready: " + logtext[-3000:])
        print("GAZEBO_PX4_117_READY",owned_px4,owned_gz,flush=True)
        for route in routes:
            pathname = RESULT / f"{route}.log"
            with open(pathname,"w") as output:
                ran = subprocess.run(
                    [sys.executable,str(ROOT / "scripts/test_px4_117_multiroute_sitl.py"),
                     "--route",route,"--timeout","65"],
                    cwd=ROOT,env=env,stdin=subprocess.DEVNULL,
                    stdout=output,stderr=subprocess.STDOUT,timeout=105,check=False)
            record={"return_code":ran.returncode,"log":str(pathname)}
            if ran.returncode==0:
                saved=json.loads((ROOT / "results/planner_waypoints_20261009" /
                                  (route + ".json")).read_text())
                expected=list(range(saved["point_count"]))
                checks=saved["checks"]
                passed=(checks["waypoint_roundtrip"] and checks["all_waypoints_reached"]
                        and checks["ros_waypoint_events"] and saved["disarmed_sitl"]
                        and saved["unique_reached_sequence"]==expected
                        and saved["odometry_path_length_m"]>=1)
                if route=="line2":
                    passed=passed and checks.get("reject_latitude_out_of_range",False)
                    passed=passed and checks.get("reject_frame_unsupported",False)
                record.update(passed=bool(passed),count=saved["point_count"],
                              reached=expected,travel_m=saved["odometry_path_length_m"],
                              elapsed_s=saved["elapsed_s"])
            else:
                record.update(passed=False,
                              log_tail=pathname.read_text(errors="replace")[-1500:])
            report["routes"][route]=record
            summary_path.write_text(json.dumps(report,indent=2))
            print("REGRESSION",route,json.dumps(record),flush=True)
            if not record["passed"]:
                raise RuntimeError("Route failed: "+route)
        report["all_pass"]=True
        summary_path.write_text(json.dumps(report,indent=2))
        print("ALL_PX4_GAZEBO_DIFFERENTIAL_MISSIONS_PASS",len(routes),flush=True)
    finally:
        if not owned_px4 or not owned_gz:
            owned_px4,owned_gz=sim_pids()
        clean_sitl(proc,owned_px4,owned_gz)
        print("SITL_DISARMED_AND_CLEANED; REAL_CUBE_BRIDGE_UNTOUCHED",flush=True)


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--routes",nargs="+",choices=ROUTES,default=list(ROUTES))
    options=parser.parse_args()
    run(options.routes)
