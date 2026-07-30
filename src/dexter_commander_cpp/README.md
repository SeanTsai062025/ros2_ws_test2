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

The example above is useful for occasional testing, but it is not a
low-latency command source. Each invocation of `ros2 topic pub -1` starts a new
Python process, creates a new ROS 2 node and publisher, waits for DDS discovery,
publishes once, and exits. That startup happens before `dexter_commander_cpp`
can receive the message and can take around one second on the robot computer.

## Persistent topic architecture

`dexter_commander_cpp` creates its `/joint_command` subscription once during
startup and keeps it alive until the commander exits. The process which
produces commands must likewise be a long-running ROS 2 node which creates one
publisher and reuses it:

```text
ossia score
    -> long-running ossia/ROS 2 bridge
       -> persistent /joint_command publisher
          -> persistent dexter_commander subscription
             -> MoveIt plan and execute
```

The publisher cannot live in `dexter_commander_cpp`: a ROS 2 publisher is the
sending endpoint, and another process cannot write messages through a
publisher owned by the commander. Putting a publisher there would only allow
the commander to publish its own messages. The bridge is the command producer,
so it must own the publisher.

The future ossia bridge should:

1. Start once and create a ROS 2 node.
2. Create one reliable, volatile publisher with queue depth 10 for
   `dexter_interfaces/msg/JointCommand` on `/joint_command`.
3. Convert each incoming ossia command to a `JointCommand` and call `publish()`
   on that existing publisher.
4. Keep the node and publisher alive while commands are expected.

DDS discovery then happens once when the bridge and commander start. Each
later ossia command uses the established topic endpoints. No interactive
Dexter CLI is required.

There is no form of `ros2 topic pub -1` that can inject a new value through a
publisher owned by another process. Until the persistent bridge is available,
the one-shot command above remains suitable for functional tests, but every
invocation necessarily pays process-startup and discovery cost. A
`ros2 topic pub --rate ...` process can keep one publisher alive, but it only
repeats its configured message and is not a general varying-command interface.

The former array interface remains temporarily available on
`/joint_command_legacy`. It uses scaling `0.5`, retaining the former 1.0 rad/s
and 1.0 rad/s² maximums:

```bash
ros2 topic pub -1 /joint_command_legacy example_interfaces/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

## Correlated show execution

`JointCommand` also carries optional `show_run_id`, `node_id`, `command_id`,
and `attempt` fields. Commander copies them to
`/arm_command_result` (`dexter_interfaces/msg/CommandResult`) only after
planning and trajectory execution reaches a terminal result. A command
received while the arm is active is returned as `REJECTED`.

The show orchestrator cancels an exact active command on `/arm_cancel` using a
`DeviceCommand` with matching run and command IDs. `/arm_stop`
(`std_msgs/msg/Empty`) remains available as an uncorrelated emergency software
stop. Neither interface replaces the physical E-stop.

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
