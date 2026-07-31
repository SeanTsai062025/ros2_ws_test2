# Robot Show Orchestrator

This package runs validated YAML shows without replacing the existing control
paths:

- `/joint_command` → `dexter_commander_cpp` → MoveIt
- `/device_command` → `uart_bridge_node` → ESP32

Every run/action receives generated `show_run_id`, internal `node_id`, and
`command_id` values. YAML node `id` fields are optional display labels and may
repeat. Results are accepted only when their correlation IDs match an in-flight
action. Runtime and validation errors include the YAML file and line number.
Sequence, parallel (`join: all`), delay, timeout, soft pause, resume, and stop
are supported.

The complete Traditional Chinese YAML authoring reference is available at
[`docs/YAML_REFERENCE_zh-TW.md`](docs/YAML_REFERENCE_zh-TW.md).

## Safety model

The rail has no position sensor or encoder. `TOP` and `BOTTOM` in YAML are
trusted initial assumptions, and successful movement means only that the ESP32
generated the requested number of steps. A timeout, cancel, UART failure, or
unconfirmed stop changes the device state to `UNKNOWN`.

Before real operation:

1. Replace the example arm positions with reviewed, collision-safe positions.
2. Verify the physical rail positions match the YAML requirements.
3. Verify the E-stop and hardware safety chain.
4. Start with `safety_confirmed:=true` only after those checks.

## Build

```bash
cd /home/sean/dexter_test_2/ros2_ws
conda run -n dexter_ros2 bash -lc \
  'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
source install/setup.bash
```

The ESP32 must be flashed with the correlated protocol implementation in
`/home/sean/dexter_test_2/desk_motor_TMC2209`.

## Run

Complete stack in simulation:

```bash
ros2 launch dexter_bringup robot_show.launch.py \
  use_hardware:=false \
  show_file:=/absolute/path/to/show.yaml \
  safety_confirmed:=true
```

For real hardware, use `use_hardware:=true`. Starting remains explicit:

```bash
ros2 service call /show_orchestrator/start std_srvs/srv/Trigger {}
```

Controls and status:

```bash
ros2 service call /show_orchestrator/pause std_srvs/srv/Trigger {}
ros2 service call /show_orchestrator/resume std_srvs/srv/Trigger {}
ros2 service call /show_orchestrator/stop std_srvs/srv/Trigger {}
ros2 topic echo /show_orchestrator/state
```

To load another file while idle, set `show_file` and call reload:

```bash
ros2 param set /show_orchestrator show_file /absolute/path/to/show.yaml
ros2 service call /show_orchestrator/reload std_srvs/srv/Trigger {}
```

Legacy manual `/motor_command` and metadata-free `/joint_command` publishers
remain supported, but their replies cannot provide cross-run correlation.
