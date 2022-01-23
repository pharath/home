---
title: "ROS Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - ros
  - cheatsheet
toc: true
toc_sticky: true
---

# roslaunch

`roslaunch --ros-args /path/to/launchfile.launch`
- Display command-line arguments for this launch file

```xml
<launch>
  <!-- ros_args.launch -->
  <arg name="foo" default="true" doc="I pity the foo'."/>
  <arg name="bar" doc="Someone walks into this."/>
  <arg name="baz" default="false"/>
  <arg name="nop"/>
  <arg name="fix" value="true"/>
</launch>
```

```bash
$> roslaunch --ros-args ros_args.launch
Required Arguments:
  bar: Someone walks into this.
  nop: undocumented
Optional Arguments:
  baz (default "false"): undocumented
  foo (default "true"): I pity the foo'.
```

# image_view

- Display the images of an image topic:

```bash
rosrun image_view image_view image:=/sensor/camera1/image_raw
```

# some open source packages

## opencv_apps

- `sudo apt install ros-noetic-opencv-apps`

```bash
roslaunch opencv_apps hough_lines.launch image:=/sensor/camera1/image_raw
# view all arguments of a node in the launch file
roslaunch --args hough_lines /opt/ros/noetic/share/opencv_apps/launch/hough_lines.launch
# view all arguments of the launch file
roslaunch --ros-args /opt/ros/noetic/share/opencv_apps/launch/hough_lines.launch
```

