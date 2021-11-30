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

## Components of SDF Models

Source: [http://gazebosim.org/tutorials?tut=build_model](http://gazebosim.org/tutorials?tut=build_model)

<blockquote>
<p><strong>Links:</strong> A link contains the physical properties of one body of the model. This can be a wheel, or a link in a joint chain. Each link may contain many collision and visual elements. Try to reduce the number of links in your models in order to improve performance and stability. For example, a table model could consist of 5 links (4 for the legs and 1 for the top) connected via joints. However, this is overly complex, especially since the joints will never move. Instead, create the table with 1 link and 5 collision elements.</p>

<blockquote>
<p><strong>Collision:</strong> A collision element encapsulates a geometry that is used for collision checking. This can be a simple shape (which is preferred), or a triangle mesh (which consumes greater resources). A link may contain many collision elements.</p>

<p><strong>Visual:</strong> A visual element is used to visualize parts of a link. A link may contain 0 or more visual elements.</p>

<p><strong>Inertial:</strong> The inertial element describes the dynamic properties of the link, such as mass and rotational inertia matrix.</p>

<p><strong>Sensor:</strong> A sensor collects data from the world for use in plugins. A link may contain 0 or more sensors.</p>

<p><strong>Light:</strong> A light element describes a light source attached to a link. A link may contain 0 or more lights.</p>
</blockquote>

<p><strong>Joints:</strong> A joint connects two links. A parent and child relationship is established along with other parameters such as axis of rotation, and joint limits.</p>

<p><strong>Plugins:</strong> A plugin is a shared library created by a third party to control a model.</p>
</blockquote>

# Troubleshooting

- `rosnode kill -a; killall -9 rosmaster; killall -9 roscore`, if nodes do not stop automatically
