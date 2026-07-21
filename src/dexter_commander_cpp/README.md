# Dexter Commander

`dexter_commander_cpp` accepts joint-space and Cartesian targets with one
whole-trajectory speed scaling value. The accepted range is `0.2` through
`1.0`. Values outside that range are rejected.

The configured joint maximums are 2.0 rad/s and 2.0 rad/s², so the principal
scaling values have these meanings:

| `speed_scaling` | Maximum joint velocity | Maximum joint acceleration |
|---:|---:|---:|
| 0.2 | 0.4 rad/s | 0.4 rad/s² |
| 0.5 | 1.0 rad/s | 1.0 rad/s² |
| 0.8 | 1.6 rad/s | 1.6 rad/s² |
| 1.0 | 2.0 rad/s | 2.0 rad/s² |

These are upper bounds used during trajectory generation. Individual joints
may move more slowly depending on the path and which joint limits the
coordinated motion.

## Joint command

Joint positions are radians and ordered as `base`, `part1`, `part2`, `part3`,
`part4`, and `part5`:

```bash
ros2 topic pub -1 /joint_command dexter_interfaces/msg/JointCommand \
  "{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed_scaling: 0.5}"
```

The former array interface remains temporarily available on
`/joint_command_legacy`. It uses scaling `0.5`, retaining the former 1.0 rad/s
and 1.0 rad/s² maximums:

```bash
ros2 topic pub -1 /joint_command_legacy example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## Pose command

Position is in metres and roll, pitch, and yaw are degrees:

```bash
ros2 topic pub -1 /pose_command dexter_interfaces/msg/PoseCommand \
  "{x: 0.0624, y: -0.0011, z: 0.31329, roll: 54.2, pitch: 88.2, yaw: 54.2, cartesian_path: false, speed_scaling: 0.8}"
```

The message default is `speed_scaling: 0.5`, preserving the robot's former
1.0 rad/s and 1.0 rad/s² maximums when the field is omitted. Specifying the
value explicitly is recommended. The scaling is applied when a trajectory is
planned; it does not change the speed of a trajectory that is already
executing.
