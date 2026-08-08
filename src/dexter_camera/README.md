# Dexter CSI camera

This ROS 2 Jazzy package publishes a Raspberry Pi CSI camera through
`camera_ros` and a workspace-local, Pi 5-compatible `libcamera`. A GStreamer
test pattern is included to verify the ROS publisher without camera hardware.

The camera at Amazon ASIN `B09VSRH14M` is an Arducam 8 MP Sony IMX219 module.
It is compatible with the Raspberry Pi Camera Module 2 driver. The default
launch configuration publishes the camera stream plus gray-object detection:

- `/camera/image_raw` (`sensor_msgs/msg/Image`, `bgr8`)
- `/camera/camera_info` (`sensor_msgs/msg/CameraInfo`)
- `/gray_object/sector` (`std_msgs/msg/Int32`)
- `/gray_object/candidate_area` (`std_msgs/msg/Int32`, pixels)
- `/gray_object/debug_image` (`sensor_msgs/msg/Image`, `bgr8`)

The detector subscribes to the existing `/camera/image_raw` topic. It handles
both the live camera's `bgr8` encoding and the package test source's `rgb8`
encoding directly, without requiring OpenCV or `cv_bridge`.

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

## Run, detect, and view

On a desktop session, this command starts the camera and gray-object detector.
It opens the annotated debug stream in the live RGB threshold tuner:

```bash
ros2 launch dexter_camera camera.launch.py
```

The detector and tuner flip both image axes by default (a 180-degree view), so
the displayed orientation, object coordinates, sections, and click-to-sample
positions all agree. Each axis can be restored independently with
`flip_vertical:=false` or `flip_horizontal:=false`; use both options to keep
the camera's original orientation.

The image is on the left and six draggable bars set independent `R min/max`,
`G min/max`, and `B min/max` ranges on the right. Matching pixels are tinted
green immediately. Click the image to lock a pixel and retain its coordinates
and R/G/B values while moving the mouse to the controls. The locked color can
also fill all six limits automatically using the selected tolerance. No
rebuild or restart is needed while tuning. The launch file exposes Ubuntu's
system PyQt5 package even when the `dexter_ros2` Conda environment is active.

The area panel shows and tunes the `little guy` candidate in pixels. All three
named classes have independent `target ± tolerance` ranges. Thirty
uncompressed 1280x720 camera frames measured `cup` at 93212..93441 px,
`big guy` at 33670..34016 px, and `little guy` at 15803..15913 px. The current
targets/tolerances are 93300±4000, 33850±3000, and 15850±1500. Parameter
updates are rejected if any two inclusive ranges would overlap.

Area matching is evaluated for every same-color connected object. Every region
inside any named range is boxed, including multiple objects in the same range;
objects outside those ranges cannot hide a valid target.

Read the original integer result (kept as the `little guy` sector):

```bash
ros2 topic echo /gray_object/sector
```

The full image is divided clockwise into 24 sections of 15 degrees each:

- `0`: 12:00 to 12:30
- `6`: 3:00 to 3:30
- `12`: 6:00 to 6:30
- `18`: 9:00 to 9:30
- `23`: 11:30 to 12:00
- `-1`: no object satisfying both the RGB and area rules

The wedge interiors are intentionally not numbered. Only the `little guy`
section number is shown at the top left.

To run without opening a local GUI:

```bash
ros2 launch dexter_camera camera.launch.py view:=false
```

To use the original `rqt_image_view` window without tuning controls:

```bash
ros2 launch dexter_camera camera.launch.py viewer:=rqt
```

To show the unannotated camera stream initially, or disable processing:

```bash
ros2 launch dexter_camera camera.launch.py \
  viewer:=rqt view_topic:=/camera/image_raw

ros2 launch dexter_camera camera.launch.py \
  detect_gray_object:=false viewer:=rqt view_topic:=/camera/image_raw
```

You can view the debug topic from another sourced terminal (or another ROS 2
machine on the same DDS network):

```bash
export PYTHONPATH=/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}
ros2 run rqt_image_view rqt_image_view /gray_object/debug_image
```

The debug image retains normal camera colors in the valid area, tints every
pixel matching the current RGB ranges green, shows each classified region in
solid green, and draws its centroid, yellow bounding box, object name, and
integer centroid pixel `X/Y`. The image center is cyan, all radial boundaries
use a subdued creamy purple, and only the `little guy` section number appears
at top left.
The full image is valid for detection; there is no red exclusion mask. If
color matches but no named area range matches, pixels remain tinted green but
no named box is produced.

### Detection geometry and threshold tuning

All camera pixels always participate in color and area detection. The older
proportional T-shaped exclusion mask and its parameters have been removed, so
no launch-time setting can reactivate the former center or bottom exclusion.

Radial boundaries cover the complete 360-degree image at 15-degree intervals.
Each centroid is the geometric, area-weighted center of its 8-connected color
region in the full image.

All tuning values are collected, with comments, in
`config/gray_object_detector.yaml`. The corresponding defaults are also
clearly marked near the top of
`dexter_camera/gray_object_detector.py`. Direct RGB tuning is enabled by
default. Initial inclusive ranges are:

- red: `55..225`
- green: `55..225`
- blue: `55..225`

Click the target color, select a tolerance, and press the apply button for a
quick starting range; then fine-tune any of the six limits. The GUI publishes
changes on `/gray_object/rgb_thresholds` immediately and also synchronizes the
detector parameters. To preserve chosen values for future launches, copy them
into
`config/gray_object_detector.yaml`; source configuration changes still require
rebuilding `dexter_camera` and sourcing the workspace again.

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
ros2 topic info /gray_object/sector
ros2 topic info /gray_object/debug_image
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
