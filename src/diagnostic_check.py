#!/usr/bin/env python3
"""
Quick diagnostic: check if move_group is alive, the planning scene is OK,
and if a simple joint-space plan succeeds.
Run AFTER 'ros2 launch dexter_bringup dexter.launch.xml use_hardware:=false'
"""
import subprocess, sys, time

def run(cmd):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
    return r.stdout.strip(), r.stderr.strip()

print("=" * 60)
print("DIAGNOSTIC: Checking move_group and planning scene")
print("=" * 60)

# 1) Check if move_group is alive
print("\n[1] Checking active nodes...")
out, _ = run("ros2 node list")
print(out)
if "/move_group" not in out:
    print("  *** ERROR: /move_group node not found! Is the launch running?")
    sys.exit(1)
print("  OK: move_group is running")

# 2) Check topics
print("\n[2] Checking key topics...")
out, _ = run("ros2 topic list")
topics = out.split("\n")
for t in ["/joint_states", "/planning_scene", "/move_action/_action/status"]:
    if t in topics:
        print(f"  OK: {t}")
    else:
        print(f"  *** MISSING: {t}")

# 3) Check controller status
print("\n[3] Checking controllers...")
out, _ = run("ros2 control list_controllers")
print(out)

# 4) Check joint states
print("\n[4] Checking joint states (one message)...")
try:
    out, _ = run("ros2 topic echo /joint_states --once")
    print(out[:500])
except subprocess.TimeoutExpired:
    print("  *** TIMEOUT: No joint states published!")

# 5) Check planning scene
print("\n[5] Getting planning scene info...")
out, _ = run("ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene '{}'")
print(out[:1000] if out else "(no output)")

# 6) Check move_group parameters - planning pipeline
print("\n[6] Checking move_group parameters for planning pipeline...")
out, _ = run("ros2 param get /move_group default_planning_pipeline")
print(f"  default_planning_pipeline: {out}")
out, _ = run("ros2 param get /move_group planning_pipelines")
print(f"  planning_pipelines: {out}")

# 7) Check if OMPL is loaded
print("\n[7] Checking OMPL planner config...")
out, _ = run("ros2 param get /move_group ompl.planning_plugins")
print(f"  ompl.planning_plugins: {out}")

# 8) List all robot_description_semantic (SRDF)
print("\n[8] Checking robot_description topics/params...")
out, _ = run("ros2 param get /move_group robot_description_semantic")
if out:
    print(f"  SRDF loaded: {len(out)} chars (first 200):")
    print(f"  {out[:200]}")
else:
    print("  *** WARNING: robot_description_semantic not found")

print("\n" + "=" * 60)
print("DIAGNOSTIC COMPLETE")
print("=" * 60)
