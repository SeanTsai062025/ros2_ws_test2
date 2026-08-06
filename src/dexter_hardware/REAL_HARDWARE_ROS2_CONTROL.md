# Dexter real-hardware ros2_control interface

## Control flow

At 60 Hz, controller manager executes this order:

1. `DexterSystem::read()` issues one 0x31 request and receives its reply before
   querying the next motor,
   then commits a state update only after all six fresh,
   checksum-valid responses.
   It waits for one overall deadline and then performs a bounded non-blocking
   drain so frames already queued by SocketCAN survive a userspace scheduling pause.
2. JointTrajectoryController samples the trajectory and compares it with those
   physical position and filtered velocity states.
3. `DexterSystem::write()` forwards JTC's desired position as an absolute 0xF5
   target. JTC's desired velocity selects the motor speed field.
4. `joint_state_broadcaster` publishes the same state interfaces on
   `/joint_states` for MoveIt and RViz.

JTC claims position and velocity command interfaces together. This is a
supported direct-forwarding combination and does not enable JTC's velocity or
effort PID adapter. There is no additional position PID in the plugin.

`use_hardware:=false` selects `mock_components/GenericSystem`.
`use_hardware:=true` selects `dexter_hardware/DexterSystem`. The Python hardware
bridge and its separate joint-state publisher/remap are not launched.

## CAN transaction and freshness rules

The production 0x31 request window is one: each reply must be received and
validated before the next motor is queried. The window remains configurable up
to six for diagnostics. Hardware CAN
captures on the current installation showed that six-request bursts—and then
three-request windows—could produce protocol-error escalation to error-passive
and drop the last reply with the adapter's original bit timing. At the overall
batch deadline, the client performs a bounded non-blocking queue drain before
reporting a missing ID.

Optional pacing is available only for diagnostic multi-request windows. It is
zero in the production serialized configuration because the reply itself gates
the next request.
The receive path matches motor ID, command byte, exact response length, and MKS
checksum. Interleaved F5 start/complete statuses, replies from other IDs, bad
checksums, duplicates, malformed responses, and unrelated frames are classified
and ignored.

MKS 0x31 has no request sequence number. To avoid accepting a late response as
a later measurement, the driver permits only one outstanding request per motor,
drains frames queued before each batch, and latches the stream unsynchronized
after any timeout. It does not issue another encoder batch until the hardware
lifecycle is reconfigured and the bus has passed a configurable quiet interval.

An incomplete six-joint read is never committed. The exported state is marked
unavailable, motor stop commands are sent, diagnostics report the failed joint,
and `read()` returns `ERROR` so controller manager stops normal execution. Old
positions are not restamped as a successful new hardware cycle.

Encoder velocity is a timestamped finite difference with a configurable
first-order filter. Zero is used only for the first sample during initialization;
the plugin does not continuously publish a fabricated zero velocity.

F5 commands are sent only when the quantized target tick or derived speed field
changes. An existing absolute target remains active inside the motor, so this
preserves mid-motion retargeting while preventing Part 5 idle status-frame
flooding.

## Startup and shutdown safety

- Configure and activate each require successful reads from all six encoders.
- Position commands are seeded from measured joint positions and velocity
  commands from zero before writes are enabled.
- Command interfaces are seeded again when a motion controller claims them.
- A write in the controller-activation cycle is deliberately deferred. A new,
  complete encoder batch must arrive after the mode switch before F5 writes are
  permitted. The first target therefore cannot be zero/home or based on stale
  pre-switch feedback, and unchanged measured targets do not generate F5 traffic.
- A configurable per-cycle position step limit rejects discontinuous commands.
  A new trajectory may safely rebase from the previous command to fresh encoder
  feedback when an idle/backdrivable joint has moved; a large target away from
  both the previous command and measured position is still rejected.
- Encoder loss, unsynchronized transactions, non-finite commands, command-step
  violations, 24-bit target overflow, and CAN transmit errors issue F6 zero-speed
  stops and return a hardware error.
- Deactivate, error, cleanup, shutdown, and destruction explicitly stop all six
  motors. Cleanup and shutdown then close the SocketCAN descriptor.
- `/diagnostics` reports read counts, ignored frame categories, batch latency,
  write-gate state, and per-joint encoder age.

The plugin requires all six joints. Partial fake state is deliberately not
supported in hardware mode because it could allow JTC to evaluate success from
non-physical feedback.

## Calibration and driver behavior

The plugin preserves the bridge's calibrated values:

| Joint | CAN ID | Gear | command sign | encoder sign | speed scale |
|---|---:|---:|---:|---:|---:|
| base | 1 | 30:1 | +1 | +1 | 1 |
| part1 | 2 | 30:1 | -1 | +1 | 1 |
| part2 | 3 | 30:1 | -1 | -1 | 1 |
| part3 | 4 | 30:1 | +1 | +1 | 1 |
| part4 | 5 | 30:1 | +1 | +1 | 1 |
| part5 | 6 | 1:1 | -1 | -1 | 8 |

All encoders use 16,384 ticks/revolution. Part 5 is verified/configured as
SR_CLOSE, 800 mA, 128 subdivisions, with a speed-field scale of 8 and
acceleration field 0.

The motor's approximately 2 kHz interpolation and position PID remain the
low-level actuator loop. JTC is the 60 Hz sampler and tracking monitor. Because
the driver interpolates toward each 0xF5 target, physical feedback will normally
lag the current JTC reference by a small, speed-dependent phase delay. The
plugin intentionally does not add the former software look-ahead: actual-state
path and goal tolerances now expose excessive lag instead of hiding it.

Hardware parameters are passed through the control xacro and bringup launch:

- `encoder_timeout_us` (default 16000): fault ceiling for a complete six-motor
  encoder batch. Long-running production traces measured normal serialized
  batches around 9.8-12.4 ms. The 60 Hz controller period is 16.67 ms, leaving
  margin for the complete batch, controller update, and CAN writes. Frames
  already queued by SocketCAN receive one bounded non-blocking drain.
- `encoder_request_window` (default 1): maximum simultaneous 0x31 requests.
  Do not increase this on the tested MKS daisy chain; values 3 and 6 reproduced
  protocol-error bursts and missing replies.
- `encoder_request_spacing_us` (default 0): optional pacing inside a diagnostic
  multi-request window; it has no effect with the safe one-request window.
- `max_speed_field` (3000), `min_speed_field` (10), and
  `fallback_speed_field` (300)
- `acceleration_field` (0)
- `velocity_filter_alpha` (0.25)
- `max_command_step_rad` (0.05 per control cycle). At the configured 2 rad/s
  joint limit, a 60 Hz trajectory advances about 0.033 rad per cycle.

Part 5 retains separate minimum/fallback speed fields of 1 and acceleration 0.
JTC tracking tolerances and its finite 2 second goal-time allowance are in
`dexter_bringup/config/ros2_controllers.yaml`.

Controller Manager hardware-execution diagnostics use warning/error means of
12/16 ms and standard deviations of 1.5/3 ms. These reflect the measured USB-CAN
batch latency while retaining margin inside each 16.67 ms control period.

For dependable operation, controller manager must be allowed to use its
requested FIFO real-time scheduling priority. If it logs `Operation not
permitted`, install the supplied limits file and add the ROS user to its group:

```bash
sudo groupadd --force realtime
sudo usermod -aG realtime "$USER"
sudo install -o root -g root -m 0644 \
  src/dexter_bringup/config/99-dexter-realtime.conf \
  /etc/security/limits.d/99-dexter-realtime.conf
```

Log out completely and back in (or reboot), then verify `ulimit -r` reports 99,
`ulimit -l` reports unlimited, and controller manager no longer emits the FIFO
warning. Consider a PREEMPT_RT kernel or CPU isolation only if measured overruns
remain after batching and FIFO scheduling are active.

## Software-only verification

The package tests cover checksums, signed 24/48-bit encoding, calibrated signs,
gear ratios, radians/ticks conversion, speed and target limits, activation
seeding, and receive queues containing interleaved, stale, duplicate, missing,
delayed, and out-of-order frames. A launch test holds simulated physical state
away from the goal and verifies FollowJointTrajectory aborts with
`GOAL_TOLERANCE_VIOLATED` rather than reporting success.

## Staged physical validation (only with explicit authorization)

1. Mechanically support the arm, clear the work envelope, provide a reachable
   emergency stop/power disconnect, and start with conservative current/speed.
2. With motor power disabled, verify CAN IDs, interface bitrate, grounding, and
   that no legacy bridge or motor controller process is running.
3. Power the drivers, but launch passive feedback only:
   `ros2 launch dexter_bringup dexter.launch.xml use_hardware:=true
   activate_arm_controller:=false use_rviz:=false`.
4. Do not send trajectories. Confirm all six encoder positions match the
   physical pose and direction, `/joint_states` matches ros2_control state, and
   `/diagnostics` remains fresh. Restart ROS away from home and confirm no F5
   target is transmitted during initialization/activation.
5. Deliberately test feedback loss with the arm supported. Confirm F6 stop
   frames, controller/hardware error, stale state invalidation, and no action
   success. Reconfigure only after the bus is quiet and healthy.
6. Activate JTC and command one small, low-speed joint displacement at a time.
   Compare JTC desired/actual/error and raw CAN captures; verify calibrated signs,
   path tolerance behavior, goal convergence, and change-only idle traffic.
7. Increase motion envelope and speed gradually. Tune motor speed/acceleration
   fields and JTC tolerances from measured tracking data; do not loosen
   tolerances to mask unexplained lag or missed feedback.

Do not perform steps 3-7 without the robot owner's explicit permission.
