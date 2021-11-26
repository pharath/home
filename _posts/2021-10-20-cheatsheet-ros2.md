---
title: "ROS2 Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - ros2
  - ros
  - cheatsheet
---

# Install

1. Über `apt install` installieren (wie [hier](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) beschrieben) 
2. Dann, wie in [binary installation](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html) unter 
- "Installing and initializing rosdep" und 
- "Installing the missing dependencies" 
beschrieben, die restlichen dependencies installieren (ohne 2. funktioniert ROS2 nicht!).

# Create Package

```bash
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/src
$ rosdep install -i --from-path src --rosdistro eloquent -y
# check output of resdep install: "#All required rosdeps installed successfully"
# in root of ws, here: "dev_ws": 
$ colcon build
```
>**Important**: open a new terminal, separate from the one where you built the workspace (for **both**, underlay and overlay) !

```bash
# source underlay: 
$ source /opt/ros/eloquent/setup.bash
$ cd ~/dev_ws
# source overlay: 
$ . install/local_setup.bash
```

> Note: sourcing your main ROS 2 installation’s `setup` and then the `dev_ws` overlay’s `local_setup`, is the same as just sourcing `dev_ws`’s `setup`, because that includes the environment of the underlay it was created in.

```bash
$ ros2 run turtlesim turtlesim_node
```

# rosdep

Packages declare their dependencies in the **package.xml** file. This command walks through those declarations and installs the ones that are missing. 

# colcon

## build

### arguments

- `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
- `--symlink-install` saves you from having to rebuild every time you tweak python scripts
- `--event-handlers console_direct+` shows console output while building (can otherwise be found in the log directory)

# ros2 pkg

## create

### arguments

- `--build-type`: `ament_cmake`
- `--dependencies` will automatically add the necessary dependency lines to `package.xml` and `CMakeLists.txt`

# Gazebo

- `wget -q -R *index.html*,*.tar.gz --no-parent -r -x -nH http://models.gazebosim.org/cardboard_box/` download model cardboard_box
