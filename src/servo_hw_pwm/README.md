# servo_hw_pwm — Hardware PWM Servo Control for Raspberry Pi 5

Production-ready ROS 2 (Jazzy) C++ node that drives an MG996R servo motor
using **RP1 hardware PWM** on a Raspberry Pi 5.  Zero jitter — no software
timing loops.

## The Problem

On Pi 5 running Ubuntu (kernel 6.8.x-raspi), the legacy `dtoverlay=pwm`
from `config.txt` targets `brcm,bcm2835` and silently fails on the Pi 5's
RP1 I/O chip.  The RP1 `clk_pwm0` clock is never initialised (rate = 0),
so the sysfs `/sys/class/pwm/pwmchip0` interface refuses to set
period/duty_cycle.

## The Solution

This node bypasses the broken sysfs path entirely:

1. **Fixes `clk_pwm0`** at startup by copying the register configuration
   from `clk_pwm1` (which *is* running — used by the Pi 5 fan controller).
2. **Configures the PWM channel registers** directly via `/dev/mem` —
   the RP1's memory-mapped I/O.
3. **Sets GPIO 18** (physical pin 12) to alt function 3 (pwm0) via the
   RP1 pinctrl registers.

The result is a rock-steady 50 Hz hardware PWM signal with ~10-bit
duty-cycle resolution (6.144 MHz clock / 50 Hz = 122,880 ticks per period).

## Wiring

```
MG996R Servo        Pi 5 GPIO Header
───────────         ────────────────
Red   (V+)    ───→  External 5-6V supply (+)
Brown (GND)   ───→  External supply GND  AND  Pi GND (pin 6)
Orange(PWM)   ───→  GPIO 18 (physical pin 12)
```

> **Important:** Power the MG996R from an external supply, NOT from the
> Pi's 5V pins.  The MG996R draws up to 2.5 A under load.

## Build

```bash
cd ~/dexter_test_2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select servo_hw_pwm
colcon test  --packages-select servo_hw_pwm   # 6 unit tests
```

## Run

The node requires `/dev/mem` access (root):

```bash
source install/setup.bash
sudo -E env "PATH=$PATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" \
    ros2 run servo_hw_pwm servo_hw_pwm_node
```

Or with the launch file:

```bash
sudo -E env "PATH=$PATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" \
    ros2 launch servo_hw_pwm servo_hw_pwm.launch.py
```

## Publish Commands

From another terminal:

```bash
# Move to 0°
ros2 topic pub --once /servo_angle std_msgs/msg/Float64 "{data: 0.0}"

# Move to 90° (centre)
ros2 topic pub --once /servo_angle std_msgs/msg/Float64 "{data: 90.0}"

# Move to 180°
ros2 topic pub --once /servo_angle std_msgs/msg/Float64 "{data: 180.0}"
```

## Parameters

| Parameter          | Default | Description                                           |
|--------------------|---------|-------------------------------------------------------|
| `gpio_pin`         | 18      | RP1 GPIO number (0-27, bank 0)                        |
| `min_pulse_us`     | 500.0   | Pulse width at 0° (microseconds)                      |
| `max_pulse_us`     | 2500.0  | Pulse width at 180° (microseconds)                    |
| `pwm_frequency_hz` | 50.0    | PWM frequency (Hz)                                    |
| `initial_angle`    | -1.0    | Angle on startup (0-180°); negative = idle until first msg |

## Standalone Clock Fix

If you prefer sysfs for some other use, the standalone utility fixes
`clk_pwm0` so that `/sys/class/pwm/pwmchip0` becomes usable:

```bash
sudo install/servo_hw_pwm/lib/servo_hw_pwm/fix_pwm0_clock
# Then use sysfs as normal:
#   echo 0 | sudo tee /sys/class/pwm/pwmchip0/export
#   echo 20000000 | sudo tee /sys/class/pwm/pwmchip0/pwm0/period
#   ...
```

Note: the sysfs kernel driver still reports clock rate = 0 in its internal
state, so `period` writes may still fail.  The direct-register approach
in the main node avoids this issue entirely.

## Config.txt Cleanup

You can safely remove the legacy overlay from `/boot/firmware/config.txt`:

```
# REMOVE this line (does nothing on Pi 5):
# dtoverlay=pwm,pin=18,func=2
```

## Architecture

```
┌────────────────────────────┐
│  servo_hw_pwm_node (C++)   │
│                            │
│  /servo_angle subscriber   │
│         │                  │
│    angle_to_ticks()        │
│         │                  │
│  ┌──────▼──────────────┐   │
│  │  /dev/mem  mmap()   │   │
│  │                     │   │
│  │  RP1 CLK  registers │◄──── fix clk_pwm0 (copy from clk_pwm1)
│  │  RP1 PWM0 registers │◄──── set RANGE, DUTY, ENABLE
│  │  RP1 GPIO registers │◄──── set GPIO18 funcsel = alt3 (pwm0)
│  └─────────────────────┘   │
└────────────────────────────┘
         │
    GPIO 18 (physical pin 12)
         │
    ┌────▼────┐
    │ MG996R  │
    │  Servo  │
    └─────────┘
```
