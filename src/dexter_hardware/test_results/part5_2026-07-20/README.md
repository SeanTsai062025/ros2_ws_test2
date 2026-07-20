# Part 5 pulsing investigation — 2026-07-20

All physical tests connected the hardware bridge exclusively to `part5`
(CAN ID 6). No command or configuration frame was sent to CAN IDs 1–5.
The requested move was 0° to −90° through MoveIt → JTC → hardware bridge.

## Result

The pulse train was caused by the motor driver's `SR_vFOC` low-speed control
loop, not by JTC command resolution, CAN frequency, or repeated F5 position
updates. Part 5 is direct drive and moves below 10 RPM in this trajectory.
In that range, the driver's self-adapting vFOC current loop repeatedly slowed,
reversed, and released the motor. Changing only Part 5 to encoder-closed-loop
`SR_CLOSE` at the Servo42D documented default of 1600 mA removed the hunt.

The permanent Part 5 settings are:

- Work mode: `SR_CLOSE` (mode 4)
- Working current: 1600 mA
- Subdivisions: 128
- Internal subdivision interpolation: enabled (unchanged)
- Command: F5 absolute encoder-axis position
- Command and encoder frequency: 100 Hz
- F5 acceleration field: 0 (no motor-side ramp)
- F5 position target and speed: direct JTC reference, no Part 5 look-ahead
- Speed-field scale at 128 subdivisions: 8 (0.125 RPM per field unit)

## Measured evidence

| Test | Reversals while command >15°/s | Near-stops while command >15°/s | Measured/commanded speed, 5th–95th percentile | Motion envelope |
|---|---:|---:|---:|---|
| F6, SR_vFOC, 1000 mA | 12 | 31 | −0.04–2.29 | strongly pulsed |
| F6, SR_vFOC, 1600 mA | 3 | 11 | 0.14–2.10 | still pulsed |
| F6, SR_CLOSE, 1600 mA | 0 | 0 | 0.98–1.15 | continuous |
| Full JTC/F5, SR_vFOC, 1000 mA | — | — | encoder peak 146.7°/s | 16 distinct bursts |
| Full JTC/F5, SR_CLOSE, 1600 mA | — | — | encoder peak 81.7°/s | 1 continuous envelope |

In the original full-chain capture, JTC/F5 targets arrived every 9.94 ms and
changed by a median 0.330° (maximum 0.659°). The encoder nevertheless spent
45.6% of its 50 ms windows below 20°/s and surged to 146.7°/s. A separate F6
test with a smooth half-sine velocity command reproduced the same hunt, which
isolated the problem from F5 and JTC.

After the mode change, the same full-chain test retained 100 Hz commands and
0.330° median target increments, but the encoder followed them as one
continuous acceleration/deceleration envelope. Its largest 10 ms encoder
increment fell from 66 ticks to 37 ticks. The final captured position was
−89.54° for a −90.00° target.

The encoder resolution is 360° / 16384 = 0.02197° per tick. The apparent large
“steps” were therefore speed/torque oscillations, not 9° position quantization.

## Files

- `before_sr_vfoc.csv`: continuous 100 Hz command and encoder trace before the fix
- `after_sr_close.csv`: continuous 100 Hz command and encoder trace after the fix
- `before_after.svg`: position and 50 ms encoder-speed comparison

Each CSV contains elapsed time, F5 target, encoder position, tracking error,
10 ms encoder increment/velocity, 50 ms velocity/speed, speed field, and
acceleration field.
