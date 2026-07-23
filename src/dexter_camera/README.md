# Dexter CSI camera

This ROS 2 Jazzy package publishes a Raspberry Pi CSI camera through
`libcamera` and GStreamer.

The camera at Amazon ASIN `B09VSRH14M` is an Arducam 8 MP Sony IMX219 module.
It is compatible with the Raspberry Pi Camera Module 2 driver. The default
launch configuration publishes:

- `/camera/image_raw` (`sensor_msgs/msg/Image`, RGB8)
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
  gstreamer1.0-libcamera \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gir1.2-gst-plugins-base-1.0 \
  libcamera-ipa \
  python3-gi \
  python3-yaml \
  ros-jazzy-rqt-image-view

sudo usermod -aG video,render "$USER"
```

The group change takes effect at the same reboot as the overlay. After reboot,
confirm that Linux created camera devices:

```bash
ls /dev/video* /dev/media*
```

Then test the camera stack independently of ROS:

```bash
gst-launch-1.0 libcamerasrc ! \
  video/x-raw,width=1280,height=720,framerate=30/1 ! \
  videoconvert ! autovideosink
```

Press `Ctrl+C` to stop it. Do not run this test and the ROS node at the same
time because only one process can own the camera.

## Build

From the ROS 2 workspace:

```bash
cd ~/dexter_test_2/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select dexter_camera
source install/setup.bash
```

## Run and view

On a desktop session, this command starts the camera and opens an image window:

```bash
ros2 launch dexter_camera camera.launch.py
```

To publish without opening a local GUI:

```bash
ros2 launch dexter_camera camera.launch.py view:=false
```

You can then view the topic from another sourced terminal (or another ROS 2
machine on the same DDS network):

```bash
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
- **`Could not find any supported camera`:** first fix detection with the
  GStreamer test above. Also confirm `libcamera-ipa` is installed.
- **Blank or out-of-focus image:** remove the protective film and rotate the
  M12 lens slightly to focus it; this IMX219 module does not have autofocus.
- **No GUI over SSH:** run with `view:=false` and open `rqt_image_view` on a
  machine with a desktop and ROS 2 discovery to the Pi.
- **Ubuntu 24.04:** Canonical only documents complete Raspberry Pi CSI camera
  stack support starting with Ubuntu 25.04. The ROS node works on Jazzy, but if
  the manual overlay and Noble's libcamera packages still cannot capture, use
  Ubuntu 25.04 or newer (which requires a ROS distribution compatible with that
  Ubuntu release), or Raspberry Pi OS with a matching ROS 2 installation.
