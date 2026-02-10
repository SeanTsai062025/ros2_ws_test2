# Full Pipeline Test: MoveIt → Servo42D Motors

## Your Current Architecture (What Exists Today)

```
                        YOUR EXISTING PIPELINE
                        ═════════════════════

  ┌─────────────────────────────────────────────────────────────┐
  │  Terminal 1: ros2 launch dexter_bringup dexter.launch.xml  │
  │                                                             │
  │  Launches:                                                  │
  │   ├── robot_state_publisher  (publishes URDF transforms)   │
  │   ├── ros2_control_node      (controller manager)          │
  │   │     └── hardware: mock_components/GenericSystem  ◄──── FAKE HARDWARE
  │   ├── joint_state_broadcaster                              │
  │   ├── arm_controller (JointTrajectoryController)           │
  │   ├── move_group     (MoveIt planning)                     │
  │   └── rviz2          (visualization)                       │
  └─────────────────────────────────────────────────────────────┘
                              │
                              │ /follow_joint_trajectory action
                              │
  ┌─────────────────────────────────────────────────────────────┐
  │  Terminal 2: ros2 run dexter_commander_cpp commander        │
  │                                                             │
  │   Subscribes to:                                           │
  │    ├── /joint_command    (Float64MultiArray, 6 joint rads) │
  │    └── /pose_command     (PoseCommand, xyz + rpy)          │
  │                                                             │
  │   Calls MoveIt to plan + execute                           │
  └─────────────────────────────────────────────────────────────┘
                              │
                              │ You send:
                              │
  ┌─────────────────────────────────────────────────────────────┐
  │  Terminal 3: ros2 topic pub ...                             │
  │                                                             │
  │  ros2 topic pub -1 /joint_command \                        │
  │    example_interfaces/msg/Float64MultiArray \               │
  │    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}"              │
  │                                                             │
  │  NOTE: values are in RADIANS, not degrees!                 │
  │  10.0 radians = 573° = INVALID (way beyond joint limits)  │
  └─────────────────────────────────────────────────────────────┘
```

## THE GAP: Where Does the Bridge Fit?

Right now your pipeline **stops at mock hardware**:

```
  /joint_command (radians)
         │
         ▼
  Commander (MoveIt plan + execute)
         │
         ▼ /follow_joint_trajectory action
  arm_controller (JTC in ros2_control)
         │
         ▼ position commands
  mock_components/GenericSystem    ◄── FAKE! Instantly "moves" to target
         │
         ╳ NOTHING GOES TO CAN BUS
         ╳ NO REAL MOTORS MOVE
```

Your `dexter_hardware_bridge.py` is a **SEPARATE** node that listens
on `/velocity_controller/commands`. It is NOT connected to ros2_control's
JTC. They are two independent systems right now.

## HOW TO CONNECT THEM (Two Options)

### Option A: Quick Bridge (What You Have Now, Just Wire It Up)

The simplest approach: Keep mock hardware in ros2_control, but
**forward** the JTC position commands to your bridge.

Problem: JTC with mock hardware uses POSITION interface, but your
bridge expects VELOCITY commands. These are different things.

### Option B: Replace Mock Hardware (The Correct Approach) ◄── RECOMMENDED

Replace `mock_components/GenericSystem` with your Python bridge
acting as the actual hardware interface. Since writing a full C++
hardware interface is complex, the practical solution is:

1. Keep ros2_control with mock hardware for MoveIt planning
2. Also run your hardware bridge
3. The bridge subscribes to the JTC's `/arm_controller/joint_trajectory`
   topic to get the trajectory, then executes it on real hardware

## STEP-BY-STEP: Full Pipeline Test

### Step 0: Understand the Values

**IMPORTANT**: Your `/joint_command` values are in RADIANS, not degrees!
```
0.1 rad  =   5.7°   ← small, safe motion
0.5 rad  =  28.6°   ← moderate motion
1.0 rad  =  57.3°   ← large motion
10.0 rad = 573°     ← IMPOSSIBLE! Way beyond any joint limit!
```

Check your joint limits:
```bash
ros2 topic echo /robot_description --once | grep -A2 "limit"
```

### Step 1: Launch MoveIt Stack (Terminal 1)

```bash
source /opt/ros/jazzy/setup.bash
source ~/dexter_test_2/ros2_ws/install/setup.bash

ros2 launch dexter_bringup dexter.launch.xml
```

This starts: robot_state_publisher, ros2_control (with MOCK hardware),
JTC, MoveIt move_group, and RViz.

Wait until you see MoveIt ready in the terminal.

### Step 2: Launch Hardware Bridge (Terminal 2) ◄── NEW!

```bash
# First, bring up CAN interface:
sudo ip link set can0 up type can bitrate 500000

# Then launch the bridge:
source /opt/ros/jazzy/setup.bash
source ~/dexter_test_2/ros2_ws/install/setup.bash

ros2 launch dexter_hardware hardware_bridge.launch.xml use_sim:=false
```

This starts: dexter_hardware_bridge.py which:
- Publishes /joint_states from real encoder feedback
- Subscribes to /velocity_controller/commands

### Step 3: Launch Commander (Terminal 3)

```bash
source /opt/ros/jazzy/setup.bash
source ~/dexter_test_2/ros2_ws/install/setup.bash

ros2 run dexter_commander_cpp commander
```

### Step 4: Monitor CAN Bus (Terminal 4)

```bash
candump can0
```

### Step 5: Send a SMALL Test Command (Terminal 5)

```bash
source /opt/ros/jazzy/setup.bash

# SMALL safe motion — 0.1 radians (~5.7 degrees) on last joint
ros2 topic pub -1 /joint_command \
  example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}"
```

## WHAT HAPPENS (Current Data Flow)

```
  Step 5: You publish /joint_command {data: [0,0,0,0,0,0.1]}
              │
              ▼
  Step 3: Commander receives it
              │ Calls arm_->setJointValueTarget([0,0,0,0,0,0.1])
              │ Calls arm_->plan(plan) → MoveIt plans trajectory
              │ Calls arm_->execute(plan)
              ▼
  MoveIt sends trajectory via /follow_joint_trajectory action
              │
              ▼
  Step 1: arm_controller (JTC) receives trajectory
              │ Interpolates at 100Hz
              │ Sends position commands to mock hardware
              ▼
  mock_components/GenericSystem
              │ Instantly "moves" to target position
              │ Reports back to JTC: "I'm at the position"
              │ JTC publishes /joint_states via joint_state_broadcaster
              ▼
  RViz shows the robot moving (in simulation)


  MEANWHILE (DISCONNECTED):
  Step 2: dexter_hardware_bridge.py
              │ Listening on /velocity_controller/commands
              │ Nobody is publishing to that topic
              │ Bridge does nothing
              ╳ Motors don't move
```

## THE PROBLEM

The pipeline has a **disconnect**:
- ros2_control JTC sends **position** commands to **mock hardware**
- Your bridge listens for **velocity** commands on a **different topic**
- Nobody connects them

## THE FIX: Forward JTC Trajectory to Real Hardware

The simplest working solution: modify the bridge to subscribe to the
JTC's action topic and execute the trajectory itself.

### What Needs to Change

Instead of subscribing to `/velocity_controller/commands`, the bridge
should subscribe to `/arm_controller/follow_joint_trajectory` result,
or better: watch `/arm_controller/joint_trajectory` and forward
trajectory points to the motors using absolute position mode (0xFE).

### Recommended Architecture

```
  /joint_command (radians)
         │
         ▼
  Commander (MoveIt plan + execute)
         │
         ▼ /follow_joint_trajectory action
  arm_controller (JTC in ros2_control)
         │
         ├─▶ mock hardware (keeps MoveIt/RViz happy)
         │
         └─▶ /arm_controller/joint_trajectory (topic)  ◄── NEW
                    │
                    ▼
         dexter_hardware_bridge.py  ◄── MODIFIED
                    │
                    ├─ Read trajectory waypoints
                    ├─ Send 0xFE absolute position commands to motors
                    └─ Read 0x31 encoder feedback
                    │
                    ▼
              Servo42D Motors (REAL MOVEMENT!)
```

## QUICK TEST WITHOUT BRIDGE CHANGES

If you just want to verify the MoveIt→Commander→Planning pipeline
works (without real motors), you only need:

```bash
# Terminal 1: Launch everything
ros2 launch dexter_bringup dexter.launch.xml

# Terminal 2: Launch commander
ros2 run dexter_commander_cpp commander

# Terminal 3: Send command
ros2 topic pub -1 /joint_command \
  example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}"
```

You should see the robot move **in RViz**. This confirms the upper
half of the pipeline works. The bridge integration is the next step.

## SUMMARY OF TERMINALS

| Terminal | Command | Purpose |
|:--------:|---------|---------|
| 1 | `ros2 launch dexter_bringup dexter.launch.xml` | MoveIt + ros2_control + RViz |
| 2 | `ros2 launch dexter_hardware hardware_bridge.launch.xml use_sim:=false` | CAN bridge (real hardware) |
| 3 | `ros2 run dexter_commander_cpp commander` | MoveIt commander node |
| 4 | `candump can0` | Monitor CAN frames |
| 5 | `ros2 topic pub ...` | Send commands |

## CRITICAL WARNINGS

1. **Values are RADIANS!** `10.0` radians = 573° = WAY beyond limits!
   Use small values: 0.1, 0.2, 0.5 max

2. **The bridge is NOT connected to JTC yet** — this is the next
   development step

3. **Start with RViz-only testing** (no hardware bridge) to verify
   planning works first

4. **CAN interface must be up** before launching the bridge:
   `sudo ip link set can0 up type can bitrate 500000`
