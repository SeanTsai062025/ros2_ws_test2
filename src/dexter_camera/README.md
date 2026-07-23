# Dexter CSI camera

This ROS 2 Jazzy package publishes a Raspberry Pi CSI camera through
`camera_ros` and a workspace-local, Pi 5-compatible `libcamera`. A GStreamer
test pattern is included to verify the ROS publisher without camera hardware.

The camera at Amazon ASIN `B09VSRH14M` is an Arducam 8 MP Sony IMX219 module.
It is compatible with the Raspberry Pi Camera Module 2 driver. The default
launch configuration publishes:

- `/camera/image_raw` (`sensor_msgs/msg/Image`, `bgr8`)
- `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`)

## Raspberry Pi 5 setup

The Pi 5 needs a 22-pin-to-15-pin camera cable. Shut the Pi down and disconnect
power before inserting or reseating the cable. Either CAM/DISP connector can
carry a camera, but this setup uses **CAM/DISP 0**.

Because this is a third-party IMX219 board, configure its overlay explicitly:

```ini
# /boot/firmware/config.txt
camera_auto_detect=0
dtoverlay=imx219,cam0
```

Do not keep a second active `camera_auto_detect=1` line. Reboot after changing
the file.

Ubuntu packages required by the node:

```bash
sudo apt update
sudo apt install \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gir1.2-gst-plugins-base-1.0 \
  python3-gi \
  python3-colcon-meson \
  python3-ply \
  python3-pyqt5 \
  python3-yaml \
  ninja-build \
  ros-jazzy-camera-ros \
  ros-jazzy-rqt-image-view

sudo usermod -aG video,render "$USER"
```

The group change takes effect at the same reboot as the overlay. After reboot,
confirm that Linux created camera devices:

```bash
ls /dev/video* /dev/media*
```

Ubuntu 24.04's system `libcamera` 0.2 does not support the Raspberry Pi 5
PiSP pipeline, so `gst-launch-1.0 libcamerasrc ...` reports that no camera is
available even when the kernel detected the sensor.

The ROS repository supplies `libcamera` 0.7.1 with PiSP support, but its
upstream Pi 5 pipeline expects newer hyphenated media-entity names. Ubuntu
24.04's Raspberry Pi kernel exposes the older underscore names and six-pad CFE
layout. That combination also reports `no cameras available`.

This workspace therefore contains the matching `libcamera` 0.7.1 source with
a small compatibility patch that accepts both layouts. It installs into this
workspace only; it does not replace Ubuntu's or ROS's packaged libraries.

The `config.txt` change affects CSI camera detection at boot only; it does not
change ROS dependencies. The patched library affects `libcamera` consumers
only in terminals where this workspace's `install/setup.bash` has been
sourced. It keeps the same 0.7.1 ABI used by `camera_ros`.

## Build

From the ROS 2 workspace:

```bash
cd ~/dexter_test_2/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/dexter_camera --ignore-src -r -y

# Keep Conda active for the robot, but use Ubuntu's Python and pkg-config
# while compiling libcamera. These expose jinja2 and ROS's libpisp package.
export PATH="/usr/bin:/bin:$PATH"
export PKG_CONFIG_PATH="/opt/ros/jazzy/lib/pkgconfig:/usr/lib/aarch64-linux-gnu/pkgconfig:/usr/lib/pkgconfig:/usr/share/pkgconfig"

# The ROS libpisp package's headers live one directory above its pkg-config
# include flag. This supplies that header root only while compiling.
export CPLUS_INCLUDE_PATH="/opt/ros/jazzy/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"

colcon build --symlink-install \
  --packages-select libcamera dexter_camera \
  --meson-args \
    -Dpipelines=rpi/pisp \
    -Dipas=rpi/pisp \
    -Dcam=disabled \
    -Ddocumentation=disabled \
    -Dgstreamer=disabled \
    -Dlc-compliance=disabled \
    -Dpycamera=disabled \
    -Dqcam=disabled \
    -Dtest=false \
    -Dv4l2=disabled

source install/setup.bash
```

The build selects only `libcamera` and `dexter_camera`; it does not rebuild
the rest of the robot workspace. On a fresh checkout, recreate the patched
source with:

```bash
cd ~/dexter_test_2/ros2_ws/src
git clone --branch v0.7.1 --depth 1 \
  https://gitlab.freedesktop.org/camera/libcamera.git
git -C libcamera apply \
  ../dexter_camera/patches/libcamera-0.7.1-pi5-ubuntu24.patch
```

After sourcing the workspace, test the backend directly:

```bash
source /opt/ros/jazzy/setup.bash
source ~/dexter_test_2/ros2_ws/install/setup.bash

ros2 run camera_ros camera_node --ros-args \
  -p camera:=0 \
  -p format:=RGB888 \
  -p width:=1280 \
  -p height:=720 \
  -p FrameDurationLimits:="[33333,33333]"
```

Press `Ctrl+C` to stop it before launching `dexter_camera`; only one process
can own the camera.

## Run and view

On a desktop session, this command starts the camera and opens an image window:

```bash
ros2 launch dexter_camera camera.launch.py
```

The launch file exposes Ubuntu's system PyQt5 package to the viewer even when
the `dexter_ros2` Conda environment is active.

To publish without opening a local GUI:

```bash
ros2 launch dexter_camera camera.launch.py view:=false
```

You can then view the topic from another sourced terminal (or another ROS 2
machine on the same DDS network):

```bash
export PYTHONPATH=/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Useful overrides:

```bash
# Lower CPU and network use
ros2 launch dexter_camera camera.launch.py \
  width:=640 height:=480 fps:=30.0

# Verify the ROS publisher and viewer without camera hardware
ros2 launch dexter_camera camera.launch.py source:=test
```

Check the published stream:

```bash
ros2 topic hz /camera/image_raw
ros2 topic info /camera/image_raw
```

## Calibration

The node publishes an uncalibrated `CameraInfo` by default. After calibrating
the camera with ROS's `camera_calibration` package, pass the generated YAML:

```bash
ros2 launch dexter_camera camera.launch.py \
  camera_info_url:=file:///absolute/path/to/camera.yaml
```

## Troubleshooting

- **No `/dev/video*` or `/dev/media*`:** the sensor driver did not load. Power
  off, check both cable ends, confirm the 22-pin-to-15-pin cable, and verify
  `camera_auto_detect=0` plus `dtoverlay=imx219,cam0`.
- **System `gst-launch` says no supported camera:** this is expected with
  Ubuntu 24.04's libcamera 0.2 on Pi 5. Use `camera_ros` as shown above.
- **`camera_ros` says no cameras are available:** first confirm that the
  workspace was built with the patched `libcamera` and that
  `install/setup.bash` was sourced after `/opt/ros/jazzy/setup.bash`. Then
  check that `/proc/device-tree/.../imx219@10` exists and that the kernel log
  includes `Using sensor imx219 10-0010 for capture`.
- **Blank or out-of-focus image:** remove the protective film and rotate the
  M12 lens slightly to focus it; this IMX219 module does not have autofocus.
- **Calibration YAML not found:** this warning does not stop image streaming.
  Calibrate later if accurate camera geometry is needed.
- **No GUI over SSH:** run with `view:=false` and open `rqt_image_view` on a
  machine with a desktop and ROS 2 discovery to the Pi.
- **Ubuntu 24.04:** the package intentionally layers its compatible
  `libcamera` 0.7.1 build over Noble's older system library.
