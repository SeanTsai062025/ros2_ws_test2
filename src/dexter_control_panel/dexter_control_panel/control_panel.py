#!/usr/bin/env python3
"""
Dexter Robot Arm — Interactive Control Panel
=============================================
A Tkinter GUI with sliders AND text entry boxes for real-time joint control.

Two modes:
  • Joint Space  – sliders/entries directly set the 6 joint angles (always works)
  • Cartesian    – sliders/entries set x/y/z/roll/pitch/yaw offsets from home,
                   solved via IK and sent as joint commands

Sliders move the **orange goal state** in MoveIt RViz (via
/rviz/moveit/update_custom_goal_state).  The "Send Pose Command" button
publishes to /pose_command so that commander.cpp can plan and execute
the actual motion.
"""

import math
import threading
import tkinter as tk
from tkinter import ttk
from functools import partial

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from dexter_interfaces.msg import PoseCommand


# ── Robot constants ──────────────────────────────────────────────────
JOINT_NAMES = ["base", "part1", "part2", "part3", "part4", "part5"]

# Joint limits from URDF (all ±π)
JOINT_LOWER = [-math.pi] * 6
JOINT_UPPER = [+math.pi] * 6

# Home FK (all joints = 0): tool_link position & orientation
HOME_X, HOME_Y, HOME_Z = 0.0624, -0.0011, 0.3129
# Home RPY ≈ (0.946 rad, 1.539 rad, 0.946 rad) ≈ (54.2°, 88.2°, 54.2°)
HOME_ROLL, HOME_PITCH, HOME_YAW = 0.9462, 1.5394, 0.9462

# Cartesian slider ranges (meters / radians) — generous for exploration
CART_RANGES = {
    "x":     (-0.35, 0.35),
    "y":     (-0.35, 0.35),
    "z":     (-0.50, 0.50),
    "roll":  (-math.pi, math.pi),
    "pitch": (-math.pi, math.pi),
    "yaw":   (-math.pi, math.pi),
}


# ── Forward Kinematics (from URDF) ──────────────────────────────────
def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])

def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])

def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])

def _origin_tf(xyz, rpy):
    T = _rot_z(rpy[2]) @ _rot_y(rpy[1]) @ _rot_x(rpy[0])
    T[0,3], T[1,3], T[2,3] = xyz
    return T

_JOINTS_URDF = [
    {"xyz": [0, 0, 0.108],                     "rpy": [3.14159, 0, 0]},
    {"xyz": [0, 0.04, -0.032557],              "rpy": [-1.5708, 0, 3.14159]},
    {"xyz": [0.0564001, 0.0849884, 0.0615],    "rpy": [3.14159, 0, -2.15669]},
    {"xyz": [-0.05133, 0.0124189, 0.021557],   "rpy": [-1.5708, 0, -3.05124]},
    {"xyz": [0, -0.029, -0.057529],            "rpy": [-1.5708, -1.5708, 0]},
    {"xyz": [-0.0490579, -0.0379121, 0.028],   "rpy": [-1.5708, -0.025367, -0.91286]},
]
_TOOL_JOINT = {"xyz": [0, 0, -0.03], "rpy": [3.14159, 0, 0]}

def forward_kinematics(q):
    """Return 4x4 transform of tool_link given 6 joint angles."""
    T = np.eye(4)
    for i, jnt in enumerate(_JOINTS_URDF):
        T = T @ _origin_tf(jnt["xyz"], jnt["rpy"]) @ _rot_z(q[i])
    T = T @ _origin_tf(_TOOL_JOINT["xyz"], _TOOL_JOINT["rpy"])
    return T

def fk_position(q):
    """Return (x, y, z) of tool_link."""
    return forward_kinematics(q)[:3, 3]

def fk_rpy(q):
    """Return (roll, pitch, yaw) of tool_link."""
    R = forward_kinematics(q)[:3, :3]
    pitch = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
    roll  = math.atan2(R[2,1], R[2,2])
    yaw   = math.atan2(R[1,0], R[0,0])
    return roll, pitch, yaw


# ── Lightweight self-collision check (sphere approximation) ─────────
# Compute each link origin using FK chain; approximate each link body as a
# sphere centred on the link origin.  Pairs that are *not* in the SRDF
# allowed-collision-matrix are tested for overlap.
#
# Radii were estimated conservatively from the STL bounding boxes.
_LINK_NAMES_ALL = ["base", "part1", "part2", "part3", "part4", "part5",
                   "part6", "tool_link"]
_LINK_RADII = {
    "base":  0.050,
    "part1": 0.038,
    "part2": 0.048,
    "part3": 0.038,
    "part4": 0.032,
    "part5": 0.038,
    "part6": 0.025,
    "tool_link": 0.018,
}
# Pairs whose collisions are always allowed (adjacent / "Never" from SRDF)
_ACM_ALLOWED = {
    frozenset(("base", "part1")),
    frozenset(("part1", "part2")),
    frozenset(("part2", "part3")),
    frozenset(("part3", "part4")),
    frozenset(("part4", "part5")),
    frozenset(("part4", "part6")),    # reason="Never" in SRDF
    frozenset(("part5", "part6")),
    frozenset(("part6", "tool_link")),
    # Also skip immediate-neighbour-of-neighbour if they can never collide
    # due to geometry; keeps false-positive rate low for this small arm.
}

def _link_origins(q):
    """Return dict of link_name → xyz position (3,) for all links."""
    origins = {}
    T = np.eye(4)
    # base frame is at world origin
    origins["base"] = T[:3, 3].copy()
    for i, jnt in enumerate(_JOINTS_URDF):
        T = T @ _origin_tf(jnt["xyz"], jnt["rpy"]) @ _rot_z(q[i])
        origins[_LINK_NAMES_ALL[i + 1]] = T[:3, 3].copy()
    T = T @ _origin_tf(_TOOL_JOINT["xyz"], _TOOL_JOINT["rpy"])
    origins["tool_link"] = T[:3, 3].copy()
    return origins

def check_self_collision_spheres(q, safety_margin=0.003):
    """
    Fast sphere-based self-collision test.
    Returns list of colliding (link_a, link_b, distance, threshold) tuples.
    Empty list → no collision detected.
    """
    origins = _link_origins(q)
    collisions = []
    names = list(origins.keys())
    for i in range(len(names)):
        for j in range(i + 2, len(names)):  # skip self & immediate adjacent
            pair = frozenset((names[i], names[j]))
            if pair in _ACM_ALLOWED:
                continue
            ri = _LINK_RADII.get(names[i], 0.02)
            rj = _LINK_RADII.get(names[j], 0.02)
            dist = np.linalg.norm(origins[names[i]] - origins[names[j]])
            threshold = ri + rj + safety_margin
            if dist < threshold:
                collisions.append((names[i], names[j], dist, threshold))
    return collisions


# ── Numerical IK (Jacobian pseudo-inverse) ──────────────────────────
def _numerical_jacobian(q, eps=1e-6):
    """6x6 Jacobian: rows = [dx, dy, dz, droll, dpitch, dyaw]."""
    J = np.zeros((6, 6))
    p0 = fk_position(q)
    r0 = np.array(fk_rpy(q))
    for i in range(6):
        q_eps = list(q)
        q_eps[i] += eps
        p1 = fk_position(q_eps)
        r1 = np.array(fk_rpy(q_eps))
        J[:3, i] = (p1 - p0) / eps
        dr = r1 - r0
        # Wrap angle differences
        for k in range(3):
            while dr[k] > math.pi:  dr[k] -= 2*math.pi
            while dr[k] < -math.pi: dr[k] += 2*math.pi
        J[3:, i] = dr / eps
    return J

def solve_ik(target_xyz, target_rpy, q_init=None, max_iter=300, tol=1e-4):
    """
    Solve IK for desired (x,y,z,roll,pitch,yaw).
    Uses damped least-squares with multiple random restarts.
    Returns (success, joint_angles).
    """
    seeds = []
    if q_init is not None:
        seeds.append(np.array(q_init, dtype=float))
    seeds.append(np.zeros(6))
    # Add random restarts
    for _ in range(6):
        seeds.append(np.random.uniform(-math.pi, math.pi, 6))

    best_err = float('inf')
    best_q = np.zeros(6)
    best_ok = False

    for seed in seeds:
        q = seed.copy()
        for it in range(max_iter):
            p = fk_position(q)
            r = np.array(fk_rpy(q))

            err_pos = np.array(target_xyz) - p
            err_rot = np.array(target_rpy) - r
            for k in range(3):
                while err_rot[k] > math.pi:  err_rot[k] -= 2*math.pi
                while err_rot[k] < -math.pi: err_rot[k] += 2*math.pi

            err = np.concatenate([err_pos, err_rot])
            err_norm = np.linalg.norm(err)

            if err_norm < tol:
                return True, q.tolist()

            if err_norm < best_err:
                best_err = err_norm
                best_q = q.copy()
                best_ok = err_norm < tol * 10  # lenient fallback

            J = _numerical_jacobian(q)
            # Adaptive damping
            lam = 0.005 + 0.05 * min(err_norm, 1.0)
            dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(6), err)

            # Adaptive step size
            step = min(1.0, 0.3 / (np.linalg.norm(dq) + 1e-9))
            q += step * dq

            # Clamp to joint limits
            for i in range(6):
                q[i] = max(JOINT_LOWER[i], min(JOINT_UPPER[i], q[i]))

        if best_ok:
            return True, best_q.tolist()

    return best_ok, best_q.tolist()


# ── ROS 2 Node ──────────────────────────────────────────────────────
class ControlPanelNode(Node):
    def __init__(self):
        super().__init__("dexter_control_panel")

        # Publish RobotState to move the orange "Goal State" in MoveIt RViz
        self.goal_state_pub = self.create_publisher(
            RobotState, "/rviz/moveit/update_custom_goal_state", 10)

        # Publish PoseCommand for commander.cpp to plan & execute
        self.pose_pub = self.create_publisher(
            PoseCommand, "/pose_command", 10)

        # Plan / Execute via MoveIt RViz external comm
        self.plan_pub = self.create_publisher(
            Empty, "/rviz/moveit/plan", 10)
        self.execute_pub = self.create_publisher(
            Empty, "/rviz/moveit/execute", 10)

        self.current_joints = [0.0] * 6
        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10)

        # MoveIt state-validity service (for proper mesh-based collision check)
        self._sv_client = self.create_client(
            GetStateValidity, "/check_state_validity")
        self._sv_available = False
        self._check_sv_availability()

    def _check_sv_availability(self):
        """Non-blocking check whether the state-validity service exists."""
        self._sv_available = self._sv_client.wait_for_service(timeout_sec=0.5)
        if self._sv_available:
            self.get_logger().info(
                "Connected to /check_state_validity — using MoveIt collision checks")
        else:
            self.get_logger().warn(
                "/check_state_validity not available — "
                "using local sphere-based collision checks")

    def _joint_state_cb(self, msg: JointState):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]

    # ── Collision checking ───────────────────────────────────────────
    def check_collision(self, positions):
        """
        Check if the given joint positions result in self-collision.
        Returns (is_colliding: bool, details: str).
        Tries MoveIt service first, falls back to local sphere check.
        """
        # Try MoveIt service
        if self._sv_available:
            result = self._check_via_moveit(positions)
            if result is not None:
                return result

        # Fall back to local sphere-based check
        collisions = check_self_collision_spheres(positions)
        if collisions:
            pairs = ", ".join(f"{a}\u2194{b}" for a, b, d, t in collisions)
            return True, f"Self-collision: {pairs}"
        return False, ""

    def _check_via_moveit(self, positions):
        """Call /check_state_validity asynchronously.  Returns None on failure."""
        if not self._sv_client.service_is_ready():
            self._sv_available = False
            return None
        req = GetStateValidity.Request()
        req.robot_state.joint_state.name = list(JOINT_NAMES)
        req.robot_state.joint_state.position = [float(p) for p in positions]
        req.robot_state.is_diff = False
        req.group_name = "arm"
        try:
            future = self._sv_client.call_async(req)
            # Wait for the background spin thread to complete the call
            import time
            deadline = time.monotonic() + 0.3
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if future.done():
                resp = future.result()
                if not resp.valid:
                    contacts = resp.contacts
                    if contacts:
                        pairs = ", ".join(
                            f"{c.contact_body_1}\u2194{c.contact_body_2}"
                            for c in contacts[:3])
                        return True, f"Collision (MoveIt): {pairs}"
                    return True, "Collision detected (MoveIt)"
                return False, ""
        except Exception as e:
            self.get_logger().warn(f"State validity check failed: {e}")
            self._sv_available = False
        return None

    def send_goal_state(self, positions):
        """Publish a RobotState to update the orange goal state in RViz."""
        rs = RobotState()
        rs.joint_state.name = list(JOINT_NAMES)
        rs.joint_state.position = [float(p) for p in positions]
        rs.is_diff = True
        self.goal_state_pub.publish(rs)

    def send_pose_command(self, x, y, z, roll_deg, pitch_deg, yaw_deg,
                          cartesian_path=False):
        """Publish a PoseCommand (RPY in degrees — commander converts to rad)."""
        msg = PoseCommand()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.roll = float(roll_deg)
        msg.pitch = float(pitch_deg)
        msg.yaw = float(yaw_deg)
        msg.cartesian_path = bool(cartesian_path)
        self.pose_pub.publish(msg)
        self.get_logger().info(
            f"Sent /pose_command: pos=({x:.4f}, {y:.4f}, {z:.4f}) "
            f"rpy=({roll_deg:.1f} deg, {pitch_deg:.1f} deg, {yaw_deg:.1f} deg) "
            f"cartesian={cartesian_path}")

    def send_plan(self):
        """Trigger MoveIt planning via RViz external comm."""
        self.plan_pub.publish(Empty())
        self.get_logger().info("Sent plan request to /rviz/moveit/plan")

    def send_execute(self):
        """Trigger execution of the last planned trajectory via RViz external comm."""
        self.execute_pub.publish(Empty())
        self.get_logger().info("Sent execute request to /rviz/moveit/execute")


# ── Tkinter GUI ─────────────────────────────────────────────────────
class ControlPanelGUI:
    SLIDER_LENGTH = 350
    ENTRY_WIDTH = 8
    WINDOW_TITLE = "Dexter Control Panel"

    def __init__(self, ros_node: ControlPanelNode):
        self.node = ros_node
        self.ik_seed = [0.0] * 6  # warm-start IK from last solution
        self._cart_timer_id = None  # debounce timer for cartesian slider
        self._in_collision = False  # current collision state
        self._collision_detail = ""
        self._build_ui()

    # ── helpers ──────────────────────────────────────────────────────
    @staticmethod
    def _set_entry(entry, text):
        """Replace entry text without triggering extra callbacks."""
        entry.delete(0, tk.END)
        entry.insert(0, text)

    # ── UI Construction ──────────────────────────────────────────────
    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title(self.WINDOW_TITLE)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.resizable(True, True)

        style = ttk.Style()
        style.configure("Header.TLabel", font=("Helvetica", 10, "bold"))

        # Collision indicator bar (top of window)
        self.collision_frame = tk.Frame(self.root, bg="#228B22", height=28)
        self.collision_frame.pack(fill="x", side="top")
        self.collision_frame.pack_propagate(False)
        self.collision_var = tk.StringVar(value="\u2705 No Collision")
        self.collision_label = tk.Label(
            self.collision_frame, textvariable=self.collision_var,
            font=("Helvetica", 10, "bold"), fg="white", bg="#228B22",
            anchor="center")
        self.collision_label.pack(fill="both", expand=True)

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=8)

        # Tab 1: Joint Space
        joint_frame = ttk.Frame(notebook, padding=10)
        notebook.add(joint_frame, text="  Joint Space  ")
        self._build_joint_tab(joint_frame)

        # Tab 2: Cartesian Pose
        cart_frame = ttk.Frame(notebook, padding=10)
        notebook.add(cart_frame, text="  Cartesian (IK)  ")
        self._build_cartesian_tab(cart_frame)

        # Status bar
        self.status_var = tk.StringVar(
            value="Ready — adjust sliders or type values to move the robot")
        status_bar = ttk.Label(self.root, textvariable=self.status_var,
                               relief="sunken", anchor="w", padding=4)
        status_bar.pack(fill="x", side="bottom")

    # ── Collision check + visual feedback ────────────────────────────
    def _update_collision_state(self, positions):
        """
        Run collision check for *positions* (radians).
        Updates the collision indicator bar and internal state.
        Returns True if colliding.
        """
        colliding, detail = self.node.check_collision(positions)
        self._in_collision = colliding
        self._collision_detail = detail
        if colliding:
            self.collision_var.set(f"\u26d4 COLLISION: {detail}")
            self.collision_frame.configure(bg="#CC0000")
            self.collision_label.configure(bg="#CC0000", fg="white")
        else:
            self.collision_var.set("\u2705 No Collision")
            self.collision_frame.configure(bg="#228B22")
            self.collision_label.configure(bg="#228B22", fg="white")
        return colliding

    # ════════════════════════════════════════════════════════════════
    #  Joint Space Tab
    # ════════════════════════════════════════════════════════════════
    def _build_joint_tab(self, parent):
        header = ttk.Label(parent,
            text="Direct joint control — use sliders or type values (degrees)",
            style="Header.TLabel")
        header.grid(row=0, column=0, columnspan=5, pady=(0, 10), sticky="w")

        self.joint_sliders = []
        self.joint_entries = []
        labels = ["J1 (base)", "J2 (part1)", "J3 (part2)",
                  "J4 (part3)", "J5 (part4)", "J6 (part5)"]

        for i, name in enumerate(labels):
            row = i + 1
            ttk.Label(parent, text=name, width=12).grid(
                row=row, column=0, sticky="e", padx=(0, 8))

            var = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(parent, from_=math.degrees(JOINT_LOWER[i]),
                               to=math.degrees(JOINT_UPPER[i]),
                               orient="horizontal",
                               length=self.SLIDER_LENGTH,
                               variable=var,
                               command=partial(self._on_joint_slider, i))
            slider.grid(row=row, column=1, sticky="ew", padx=4)

            # Text entry (degrees)
            entry = ttk.Entry(parent, width=self.ENTRY_WIDTH, justify="center")
            entry.insert(0, "0.0")
            entry.grid(row=row, column=2, padx=4)
            entry.bind("<Return>", partial(self._on_joint_entry, i))
            entry.bind("<FocusOut>", partial(self._on_joint_entry, i))

            ttk.Label(parent, text="\u00b0", width=2).grid(
                row=row, column=3, sticky="w")

            self.joint_sliders.append(var)
            self.joint_entries.append(entry)

        parent.columnconfigure(1, weight=1)

        # Buttons
        btn_frame = ttk.Frame(parent)
        btn_frame.grid(row=len(labels)+1, column=0, columnspan=5, pady=12)

        ttk.Button(btn_frame, text="\u2302 Home (all zero)",
                   command=self._joint_home).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="Read Current",
                   command=self._joint_read_current).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="SRDF pose1",
                   command=lambda: self._joint_preset(
                       [-1.6489, -0.4339, 0.8505, -0.0174, 0.9546, -1.5794])
                   ).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="SRDF pose2",
                   command=lambda: self._joint_preset(
                       [0.7116, 1.4406, 0.8157, -1.6836, 0.8505, 0.0])
                   ).pack(side="left", padx=4)

        # FK readout
        fk_frame = ttk.LabelFrame(parent, text="FK \u2192 Tool Pose", padding=6)
        fk_frame.grid(row=len(labels)+2, column=0, columnspan=5,
                      sticky="ew", pady=4)
        self.fk_var = tk.StringVar(value="")
        ttk.Label(fk_frame, textvariable=self.fk_var,
                  font=("Courier", 10)).pack()

        # Send pose command (from joint FK)
        cmd_frame = ttk.LabelFrame(parent,
            text="Send to /pose_command (MoveIt planner)", padding=6)
        cmd_frame.grid(row=len(labels)+3, column=0, columnspan=5,
                       sticky="ew", pady=4)

        self.jt_pose_preview = tk.StringVar(value="")
        ttk.Label(cmd_frame, textvariable=self.jt_pose_preview,
                  font=("Courier", 9), wraplength=600).pack(fill="x")

        jpc_btns = ttk.Frame(cmd_frame)
        jpc_btns.pack(pady=(4, 0))
        self.jt_cartesian_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(jpc_btns, text="cartesian_path",
                        variable=self.jt_cartesian_var).pack(side="left", padx=4)
        ttk.Button(jpc_btns, text="\U0001f4e4 Send Pose Command",
                   command=self._send_pose_from_joints).pack(side="left", padx=4)

        # Plan / Execute buttons (joint tab)
        plan_exec_frame = ttk.LabelFrame(parent,
            text="Plan & Execute (MoveIt RViz)", padding=6)
        plan_exec_frame.grid(row=len(labels)+4, column=0, columnspan=5,
                             sticky="ew", pady=4)
        pe_btns = ttk.Frame(plan_exec_frame)
        pe_btns.pack()
        ttk.Button(pe_btns, text="\u25b6 Plan",
                   command=self._on_plan).pack(side="left", padx=6)
        ttk.Button(pe_btns, text="\u26a1 Execute",
                   command=self._on_execute).pack(side="left", padx=6)
        ttk.Button(pe_btns, text="\U0001f680 Plan & Execute",
                   command=self._on_plan_and_execute).pack(side="left", padx=6)

        # initialise FK display
        self._update_fk([0.0] * 6)

    # -- joint slider callback --
    def _on_joint_slider(self, idx, _val=None):
        deg = self.joint_sliders[idx].get()
        self._set_entry(self.joint_entries[idx], f"{deg:.1f}")
        positions = [math.radians(s.get()) for s in self.joint_sliders]
        self._update_fk(positions)
        colliding = self._update_collision_state(positions)
        if colliding:
            self.status_var.set(
                f"\u26d4 COLLISION — {self._collision_detail}")
        else:
            self.node.send_goal_state(positions)
            self.status_var.set(
                f"Joint [{JOINT_NAMES[idx]}] = {positions[idx]:+.3f} rad "
                f"({deg:+.1f}\u00b0)")

    # -- joint entry callback --
    def _on_joint_entry(self, idx, event=None):
        text = self.joint_entries[idx].get().strip()
        try:
            deg = float(text)
        except ValueError:
            self.status_var.set(f"Invalid value: '{text}' \u2014 enter a number (degrees)")
            return
        lo = math.degrees(JOINT_LOWER[idx])
        hi = math.degrees(JOINT_UPPER[idx])
        deg = max(lo, min(hi, deg))
        self._set_entry(self.joint_entries[idx], f"{deg:.1f}")
        self.joint_sliders[idx].set(deg)  # triggers _on_joint_slider

    # -- FK display --
    def _update_fk(self, q):
        pos = fk_position(q)
        r, p, y = fk_rpy(q)
        rd, pd, yd = math.degrees(r), math.degrees(p), math.degrees(y)
        self.fk_var.set(
            f"x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}  |  "
            f"r={rd:+.1f}\u00b0  p={pd:+.1f}\u00b0  y={yd:+.1f}\u00b0")
        self.jt_pose_preview.set(
            f"x: {pos[0]:.5f}, y: {pos[1]:.5f}, z: {pos[2]:.5f}, "
            f"roll: {rd:.1f}, pitch: {pd:.1f}, yaw: {yd:.1f}")

    # -- send pose command (from joint FK) --
    def _send_pose_from_joints(self):
        positions = [math.radians(s.get()) for s in self.joint_sliders]
        # Re-check collision before sending
        colliding = self._update_collision_state(positions)
        if colliding:
            self.status_var.set(
                f"\u26d4 BLOCKED — cannot send: {self._collision_detail}")
            return
        pos = fk_position(positions)
        r, p, y = fk_rpy(positions)
        rd, pd, yd = math.degrees(r), math.degrees(p), math.degrees(y)
        cart = self.jt_cartesian_var.get()
        self.node.send_pose_command(pos[0], pos[1], pos[2], rd, pd, yd,
                                    cartesian_path=cart)
        self.status_var.set(
            f"\U0001f4e4 Sent /pose_command: ({pos[0]:.4f}, {pos[1]:.4f}, "
            f"{pos[2]:.4f}) rpy=({rd:.1f}\u00b0, {pd:.1f}\u00b0, {yd:.1f}\u00b0) "
            f"cartesian={cart}")

    # -- presets --
    def _joint_home(self):
        for i, s in enumerate(self.joint_sliders):
            s.set(0.0)
            self._set_entry(self.joint_entries[i], "0.0")
        positions = [0.0] * 6
        self._update_collision_state(positions)
        self.node.send_goal_state(positions)
        self._update_fk(positions)
        self.status_var.set("Returned to Home position")

    def _joint_read_current(self):
        for i, val in enumerate(self.node.current_joints):
            deg = math.degrees(val)
            self.joint_sliders[i].set(deg)
            self._set_entry(self.joint_entries[i], f"{deg:.1f}")
        self._update_fk(self.node.current_joints)
        self._update_collision_state(self.node.current_joints)
        self.status_var.set("Sliders synced to current joint state")

    def _joint_preset(self, joints_rad):
        for i, val in enumerate(joints_rad):
            deg = math.degrees(val)
            self.joint_sliders[i].set(deg)
            self._set_entry(self.joint_entries[i], f"{deg:.1f}")
        self._update_fk(joints_rad)
        colliding = self._update_collision_state(joints_rad)
        if not colliding:
            self.node.send_goal_state(joints_rad)
            self.status_var.set("Preset applied")
        else:
            self.status_var.set(
                f"\u26d4 Preset in collision: {self._collision_detail}")

    # ════════════════════════════════════════════════════════════════
    #  Cartesian (IK) Tab
    # ════════════════════════════════════════════════════════════════
    def _build_cartesian_tab(self, parent):
        header = ttk.Label(parent,
            text="Cartesian offsets from Home \u2192 IK \u2192 joint angles",
            style="Header.TLabel")
        header.grid(row=0, column=0, columnspan=5, pady=(0, 6), sticky="w")

        note = ttk.Label(parent,
            text="\u26a0 IK may fail for some poses \u2014 status bar shows result",
            foreground="gray")
        note.grid(row=1, column=0, columnspan=5, pady=(0, 10), sticky="w")

        self.cart_sliders = {}
        self.cart_entries = {}
        self.cart_units = {}   # (lo_disp, hi_disp, display_unit_string)
        cart_params = [
            ("x",     "m"),
            ("y",     "m"),
            ("z",     "m"),
            ("roll",  "deg"),
            ("pitch", "deg"),
            ("yaw",   "deg"),
        ]

        for i, (name, unit) in enumerate(cart_params):
            row = i + 2
            lo, hi = CART_RANGES[name]
            if unit == "m":
                lo_disp, hi_disp = lo * 1000, hi * 1000  # mm
                disp_unit = "mm"
            else:
                lo_disp, hi_disp = math.degrees(lo), math.degrees(hi)
                disp_unit = "\u00b0"

            ttk.Label(parent, text=f"{name} ({disp_unit})", width=12).grid(
                row=row, column=0, sticky="e", padx=(0, 8))

            var = tk.DoubleVar(value=0.0)
            slider = ttk.Scale(parent, from_=lo_disp, to=hi_disp,
                               orient="horizontal",
                               length=self.SLIDER_LENGTH,
                               variable=var,
                               command=partial(self._on_cart_slider, name))
            slider.grid(row=row, column=1, sticky="ew", padx=4)

            # Text entry
            entry = ttk.Entry(parent, width=self.ENTRY_WIDTH, justify="center")
            entry.insert(0, "0.0")
            entry.grid(row=row, column=2, padx=4)
            entry.bind("<Return>", partial(self._on_cart_entry, name))
            entry.bind("<FocusOut>", partial(self._on_cart_entry, name))

            ttk.Label(parent, text=disp_unit, width=3).grid(
                row=row, column=3, sticky="w")

            self.cart_sliders[name] = var
            self.cart_entries[name] = entry
            self.cart_units[name] = (lo_disp, hi_disp, disp_unit)

        parent.columnconfigure(1, weight=1)

        # Buttons
        btn_frame = ttk.Frame(parent)
        btn_frame.grid(row=len(cart_params)+2, column=0, columnspan=5, pady=12)
        ttk.Button(btn_frame, text="\u2302 Reset to Home",
                   command=self._cart_home).pack(side="left", padx=4)

        # IK target (absolute pose)
        tgt_frame = ttk.LabelFrame(parent,
            text="IK Target (absolute pose)", padding=6)
        tgt_frame.grid(row=len(cart_params)+3, column=0, columnspan=5,
                       sticky="ew", pady=4)
        self.ik_target_var = tk.StringVar(
            value=f"pos=({HOME_X:.4f}, {HOME_Y:.4f}, {HOME_Z:.4f}) m  |  "
                  f"rpy=({math.degrees(HOME_ROLL):+.1f}\u00b0, "
                  f"{math.degrees(HOME_PITCH):+.1f}\u00b0, "
                  f"{math.degrees(HOME_YAW):+.1f}\u00b0)")
        ttk.Label(tgt_frame, textvariable=self.ik_target_var,
                  font=("Courier", 9)).pack()

        # IK solution
        ik_frame = ttk.LabelFrame(parent,
            text="IK Solution \u2192 Joint Angles (deg)", padding=6)
        ik_frame.grid(row=len(cart_params)+4, column=0, columnspan=5,
                      sticky="ew", pady=4)
        self.ik_result_var = tk.StringVar(value="(move sliders to solve IK)")
        ttk.Label(ik_frame, textvariable=self.ik_result_var,
                  font=("Courier", 9)).pack()

        # Send Pose Command
        cmd_frame = ttk.LabelFrame(parent,
            text="Send to /pose_command (MoveIt planner)", padding=6)
        cmd_frame.grid(row=len(cart_params)+5, column=0, columnspan=5,
                       sticky="ew", pady=4)

        self.cart_pose_preview = tk.StringVar(value="")
        ttk.Label(cmd_frame, textvariable=self.cart_pose_preview,
                  font=("Courier", 9), wraplength=600).pack(fill="x")

        cpc_btns = ttk.Frame(cmd_frame)
        cpc_btns.pack(pady=(4, 0))
        self.cart_cartesian_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(cpc_btns, text="cartesian_path",
                        variable=self.cart_cartesian_var).pack(side="left", padx=4)
        ttk.Button(cpc_btns, text="\U0001f4e4 Send Pose Command",
                   command=self._send_pose_from_cart).pack(side="left", padx=4)

        # Plan / Execute buttons (cartesian tab)
        plan_exec_frame = ttk.LabelFrame(parent,
            text="Plan & Execute (MoveIt RViz)", padding=6)
        plan_exec_frame.grid(row=len(cart_params)+6, column=0, columnspan=5,
                             sticky="ew", pady=4)
        pe_btns = ttk.Frame(plan_exec_frame)
        pe_btns.pack()
        ttk.Button(pe_btns, text="\u25b6 Plan",
                   command=self._on_plan).pack(side="left", padx=6)
        ttk.Button(pe_btns, text="\u26a1 Execute",
                   command=self._on_execute).pack(side="left", padx=6)
        ttk.Button(pe_btns, text="\U0001f680 Plan & Execute",
                   command=self._on_plan_and_execute).pack(side="left", padx=6)

        self._update_cart_preview()

    # -- cart slider callback (debounced) --
    def _on_cart_slider(self, name, _val=None):
        val = self.cart_sliders[name].get()
        self._set_entry(self.cart_entries[name], f"{val:.1f}")
        self.status_var.set("Solving IK...")
        if self._cart_timer_id is not None:
            self.root.after_cancel(self._cart_timer_id)
        self._cart_timer_id = self.root.after(80, self._solve_and_send_cart)

    # -- cart entry callback --
    def _on_cart_entry(self, name, event=None):
        text = self.cart_entries[name].get().strip()
        try:
            val = float(text)
        except ValueError:
            self.status_var.set(f"Invalid value: '{text}' \u2014 enter a number")
            return
        lo, hi, _u = self.cart_units[name]
        val = max(lo, min(hi, val))
        self._set_entry(self.cart_entries[name], f"{val:.1f}")
        self.cart_sliders[name].set(val)  # triggers _on_cart_slider

    # -- compute absolute pose from sliders --
    def _get_cart_absolute_pose(self):
        """Return (tx, ty, tz, tr_rad, tp_rad, tw_rad)."""
        vals = {n: v.get() for n, v in self.cart_sliders.items()}
        tx = HOME_X + vals["x"] / 1000.0
        ty = HOME_Y + vals["y"] / 1000.0
        tz = HOME_Z + vals["z"] / 1000.0
        tr = HOME_ROLL  + math.radians(vals["roll"])
        tp = HOME_PITCH + math.radians(vals["pitch"])
        tw = HOME_YAW   + math.radians(vals["yaw"])
        return tx, ty, tz, tr, tp, tw

    def _update_cart_preview(self):
        tx, ty, tz, tr, tp, tw = self._get_cart_absolute_pose()
        rd, pd, yd = math.degrees(tr), math.degrees(tp), math.degrees(tw)
        self.cart_pose_preview.set(
            f"x: {tx:.5f}, y: {ty:.5f}, z: {tz:.5f}, "
            f"roll: {rd:.1f}, pitch: {pd:.1f}, yaw: {yd:.1f}")

    # -- IK solve + send --
    def _solve_and_send_cart(self):
        self._cart_timer_id = None
        tx, ty, tz, tr, tp, tw = self._get_cart_absolute_pose()
        rd, pd, yd = math.degrees(tr), math.degrees(tp), math.degrees(tw)

        ok, q = solve_ik([tx, ty, tz], [tr, tp, tw],
                         q_init=self.ik_seed, max_iter=150, tol=1e-3)

        self.ik_target_var.set(
            f"pos=({tx:.4f}, {ty:.4f}, {tz:.4f}) m  |  "
            f"rpy=({rd:+.1f}\u00b0, {pd:+.1f}\u00b0, {yd:+.1f}\u00b0)")
        self._update_cart_preview()

        if ok:
            self.ik_seed = list(q)
            # Check collision before sending
            colliding = self._update_collision_state(q)
            if colliding:
                q_deg = [math.degrees(v) for v in q]
                self.ik_result_var.set(
                    f"\u26d4 J1={q_deg[0]:+6.1f}  J2={q_deg[1]:+6.1f}  "
                    f"J3={q_deg[2]:+6.1f}  J4={q_deg[3]:+6.1f}  "
                    f"J5={q_deg[4]:+6.1f}  J6={q_deg[5]:+6.1f}  COLLISION!")
                self.status_var.set(
                    f"\u26d4 IK solved but COLLISION — {self._collision_detail}")
            else:
                self.node.send_goal_state(q)
                q_deg = [math.degrees(v) for v in q]
                self.ik_result_var.set(
                    f"J1={q_deg[0]:+6.1f}  J2={q_deg[1]:+6.1f}  "
                    f"J3={q_deg[2]:+6.1f}  J4={q_deg[3]:+6.1f}  "
                    f"J5={q_deg[4]:+6.1f}  J6={q_deg[5]:+6.1f}")
                self.status_var.set(
                    f"IK \u2713  target=({tx:.3f}, {ty:.3f}, {tz:.3f}) "
                    f"rpy=({rd:.0f}\u00b0, {pd:.0f}\u00b0, {yd:.0f}\u00b0)")
        else:
            self._update_collision_state([0.0] * 6)  # reset indicator
            self.ik_result_var.set(
                "\u2717 IK FAILED \u2014 pose may be unreachable or in collision")
            self.status_var.set(
                f"IK \u2717  target=({tx:.3f}, {ty:.3f}, {tz:.3f}) "
                f"rpy=({rd:.0f}\u00b0, {pd:.0f}\u00b0, {yd:.0f}\u00b0) "
                "\u2014 UNREACHABLE")

    # -- send pose command (from Cartesian) --
    def _send_pose_from_cart(self):
        # Check collision with current IK solution
        if self._in_collision:
            self.status_var.set(
                f"\u26d4 BLOCKED — cannot send: {self._collision_detail}")
            return
        tx, ty, tz, tr, tp, tw = self._get_cart_absolute_pose()
        rd, pd, yd = math.degrees(tr), math.degrees(tp), math.degrees(tw)
        cart = self.cart_cartesian_var.get()
        self.node.send_pose_command(tx, ty, tz, rd, pd, yd,
                                    cartesian_path=cart)
        self.status_var.set(
            f"\U0001f4e4 Sent /pose_command: ({tx:.4f}, {ty:.4f}, {tz:.4f}) "
            f"rpy=({rd:.1f}\u00b0, {pd:.1f}\u00b0, {yd:.1f}\u00b0) "
            f"cartesian={cart}")

    # -- reset --
    def _cart_home(self):
        for name, var in self.cart_sliders.items():
            var.set(0.0)
            self._set_entry(self.cart_entries[name], "0.0")
        self.ik_seed = [0.0] * 6
        positions = [0.0] * 6
        self._update_collision_state(positions)
        self.node.send_goal_state(positions)
        self.ik_target_var.set(
            f"pos=({HOME_X:.4f}, {HOME_Y:.4f}, {HOME_Z:.4f}) m  |  "
            f"rpy=({math.degrees(HOME_ROLL):+.1f}\u00b0, "
            f"{math.degrees(HOME_PITCH):+.1f}\u00b0, "
            f"{math.degrees(HOME_YAW):+.1f}\u00b0)")
        self.ik_result_var.set("Home position")
        self._update_cart_preview()
        self.status_var.set("Returned to Home position")

    # ════════════════════════════════════════════════════════════════
    #  Plan / Execute  (shared by both tabs)
    # ════════════════════════════════════════════════════════════════
    def _on_plan(self):
        """Plan to the current orange goal state shown in RViz."""
        if self._in_collision:
            self.status_var.set(
                "\u26d4 Cannot plan — goal is in collision!")
            return
        self.node.send_plan()
        self.status_var.set(
            "\u25b6 Planning requested — check RViz for the trajectory preview")

    def _on_execute(self):
        """Execute the last planned trajectory."""
        self.node.send_execute()
        self.status_var.set("\u26a1 Execute requested — robot is moving")

    def _on_plan_and_execute(self):
        """Plan, then automatically execute after a short delay."""
        if self._in_collision:
            self.status_var.set(
                "\u26d4 Cannot plan — goal is in collision!")
            return
        self.node.send_plan()
        self.status_var.set(
            "\U0001f680 Planning… will execute automatically in 2 s")
        # Give MoveIt time to plan and display the trajectory, then execute
        self.root.after(2000, self._on_execute)

    # ── Run ──────────────────────────────────────────────────────────
    def _on_close(self):
        self.root.destroy()

    def run(self):
        def ros_spin():
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.05)
        spin_thread = threading.Thread(target=ros_spin, daemon=True)
        spin_thread.start()
        self.root.mainloop()


# ── Entry point ─────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = ControlPanelNode()
    gui = ControlPanelGUI(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
