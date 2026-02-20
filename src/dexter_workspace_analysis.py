#!/usr/bin/env python3
"""
Diagnostic script: compute FK for the dexter_simplify robot to find its workspace.
Uses the URDF joint origins/axes to build the kinematic chain manually.
"""
import numpy as np
from itertools import product

def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])

def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])

def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])

def trans(x, y, z):
    T = np.eye(4)
    T[0,3], T[1,3], T[2,3] = x, y, z
    return T

def rpy_to_matrix(r, p, y):
    return rot_z(y) @ rot_y(p) @ rot_x(r)

def origin_transform(xyz, rpy):
    T = rpy_to_matrix(rpy[0], rpy[1], rpy[2])
    T[0,3], T[1,3], T[2,3] = xyz
    return T

# Kinematic chain from URDF (base -> part1 -> part2 -> ... -> tool_link)
# Each joint: (xyz, rpy, axis)  -- axis is always [0,0,1] for revolute
joints = [
    # Joint "base": base -> part1
    {"xyz": [-3.05311e-16, -7.91034e-16, 0.108], "rpy": [3.14159, 0, -5.55112e-16]},
    # Joint "part1": part1 -> part2
    {"xyz": [3.00454e-16, 0.04, -0.032557], "rpy": [-1.5708, 0, 3.14159]},
    # Joint "part2": part2 -> part3
    {"xyz": [0.0564001, 0.0849884, 0.0615], "rpy": [3.14159, -5.70983e-15, -2.15669]},
    # Joint "part3": part3 -> part4
    {"xyz": [-0.05133, 0.0124189, 0.021557], "rpy": [-1.5708, 5.58219e-15, -3.05124]},
    # Joint "part4": part4 -> part5
    {"xyz": [-5.55112e-17, -0.029, -0.057529], "rpy": [-1.5708, -1.5708, 0]},
    # Joint "part5": part5 -> part6
    {"xyz": [-0.0490579, -0.0379121, 0.028], "rpy": [-1.5708, -0.025367, -0.91286]},
]

# Fixed joint: part6 -> tool_link
tool_joint = {"xyz": [0, 0, -0.03], "rpy": [3.14159, 0, 0]}

def forward_kinematics(q):
    """Compute tool_link position given joint angles q[0..5]"""
    T = np.eye(4)
    for i, jnt in enumerate(joints):
        # Apply joint origin transform
        T_origin = origin_transform(jnt["xyz"], jnt["rpy"])
        T = T @ T_origin
        # Apply joint rotation (around z-axis)
        T = T @ rot_z(q[i])
    # Apply tool_link fixed joint
    T_tool = origin_transform(tool_joint["xyz"], tool_joint["rpy"])
    T = T @ T_tool
    return T

# ─── 1) Show home position (all zeros) ───
print("=" * 70)
print("HOME POSITION (all joints = 0)")
print("=" * 70)
T_home = forward_kinematics([0, 0, 0, 0, 0, 0])
pos = T_home[:3, 3]
print(f"  tool_link position: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
print(f"  distance from base: {np.sqrt(pos[0]**2 + pos[1]**2):.4f} m (horizontal)")
print()

# ─── 2) Show several named poses from SRDF ───
print("=" * 70)
print("SRDF NAMED POSES")
print("=" * 70)
named = {
    "home":  [0, 0, 0, 0, 0, 0],
    "pose1": [-1.6489, -0.4339, 0.8505, -0.0174, 0.9546, -1.5794],
    "pose2": [0.7116, 1.4406, 0.8157, -1.6836, 0.8505, 0],
}
for name, q in named.items():
    T = forward_kinematics(q)
    p = T[:3, 3]
    print(f"  {name:6s}: x={p[0]:+.4f}  y={p[1]:+.4f}  z={p[2]:+.4f}  "
          f"dist_horiz={np.sqrt(p[0]**2+p[1]**2):.4f}")
print()

# ─── 3) Monte-Carlo workspace sampling ───
print("=" * 70)
print("WORKSPACE (Monte-Carlo sampling, 500,000 random configs)")
print("=" * 70)
np.random.seed(42)
N = 500_000
qs = np.random.uniform(-np.pi, np.pi, size=(N, 6))
positions = np.zeros((N, 3))
for i in range(N):
    T = forward_kinematics(qs[i])
    positions[i] = T[:3, 3]

x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]
r_horiz = np.sqrt(x**2 + y**2)
r_total = np.sqrt(x**2 + y**2 + z**2)

print(f"  X range:  [{x.min():.4f}, {x.max():.4f}] m")
print(f"  Y range:  [{y.min():.4f}, {y.max():.4f}] m")
print(f"  Z range:  [{z.min():.4f}, {z.max():.4f}] m")
print(f"  Horizontal reach (sqrt(x²+y²)): [{r_horiz.min():.4f}, {r_horiz.max():.4f}] m")
print(f"  Total reach (sqrt(x²+y²+z²)):   [{r_total.min():.4f}, {r_total.max():.4f}] m")
print()

# ─── 4) Find some actually reachable sample poses ───
print("=" * 70)
print("SAMPLE REACHABLE POSES (for use with commander)")
print("=" * 70)
# Pick some joint configs and show their cartesian pose + orientation
test_configs = {
    "all_zero":        [0, 0, 0, 0, 0, 0],
    "j1=0.5":          [0, 0.5, 0, 0, 0, 0],
    "j1=-0.5":         [0, -0.5, 0, 0, 0, 0],
    "j1=0.5,j2=0.5":   [0, 0.5, 0.5, 0, 0, 0],
    "j1=1.0,j2=-0.5":  [0, 1.0, -0.5, 0, 0, 0],
    "reach_forward":   [0, 0.8, 0.4, 0, 0, 0],
    "reach_up":        [0, -0.5, -0.5, 0, 0, 0],
}
print(f"  {'Config':25s}  {'x':>8s} {'y':>8s} {'z':>8s}  orientation(rpy approx)")
for name, q in test_configs.items():
    T = forward_kinematics(q)
    p = T[:3, 3]
    # Extract rough RPY from rotation matrix
    R = T[:3,:3]
    pitch = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    roll  = np.arctan2(R[2,1], R[2,2])
    yaw   = np.arctan2(R[1,0], R[0,0])
    print(f"  {name:25s}  {p[0]:+.4f}  {p[1]:+.4f}  {p[2]:+.4f}  "
          f"r={roll:+.2f} p={pitch:+.2f} y={yaw:+.2f}")

print()
print("=" * 70)
print("PERCENTILE DISTRIBUTION")
print("=" * 70)
for pct in [5, 25, 50, 75, 95]:
    print(f"  {pct}th percentile:  "
          f"x=[{np.percentile(x, 100-pct):+.4f}, {np.percentile(x, pct):+.4f}]  "
          f"y=[{np.percentile(y, 100-pct):+.4f}, {np.percentile(y, pct):+.4f}]  "
          f"z=[{np.percentile(z, 100-pct):+.4f}, {np.percentile(z, pct):+.4f}]")

print()
print("=" * 70)
print("TIP: Use goToJointTarget first to verify the planner works,")
print("     then use the positions above as goToPoseTarget goals.")
print("=" * 70)
