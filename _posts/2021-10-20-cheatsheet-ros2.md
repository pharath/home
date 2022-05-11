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
toc: true
toc_sticky: true
---

# Install

1. Über `apt install` installieren (wie [hier](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) beschrieben) 
2. Dann, wie in [binary installation](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html) unter 
- "Installing and initializing rosdep" und 
    - bei älteren EOL distros geht `rosdep install --from-paths /opt/ros/eloquent/share` nur, wenn `rosdep update --include-eol-distros` ausgeführt wurde
- "Installing the missing dependencies" 

beschrieben, die restlichen dependencies installieren (ohne 2. funktioniert ROS2 nicht!).
3. colcon installieren: `sudo apt install python3-colcon-common-extensions`

# Uninstall

- `sudo apt remove ~nros-galactic-* && sudo apt autoremove`

# Create Package

```bash
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/src
$ source /opt/ros/eloquent/setup.bash
$ ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

## package.xml

- make sure to fill in the `<description>`, `<maintainer>` and `<license>` tags in package.xml
- Add a new line after the `ament_cmake` buildtool dependency and paste the following dependencies corresponding to your node’s include statements:

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

## CMakeLists.txt

```cmake
# Now open the CMakeLists.txt file. Below the existing dependency find_package(ament_cmake REQUIRED), add the lines:

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# After that, add the executable and name it talker so you can run your node using ros2 run:

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Finally, add the install(TARGETS…) section so ros2 run can find your executable:

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```

## Adding a node to CMakeLists.txt

E.g. in order to add the node `listener` write:

```cmake
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

## Build

```bash
$ rosdep install -i --from-path src --rosdistro eloquent -y
# check output of resdep install: "#All required rosdeps installed successfully"
# in root of ws, here: "dev_ws": 
$ cd ~/dev_ws
$ colcon build --packages-select my_package
```

## Run

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

- Packages declare their dependencies in the **package.xml** file. This command walks through those declarations and installs the ones that are missing. 
    - **Best practice**: check for dependencies every time you clone: 
        - From the root of your workspace (`dev_ws`), run the following command: `rosdep install -i --from-path src --rosdistro eloquent -y`
- `rosdep update`
- `rosdep install`
    - `rosdep install --from-paths /opt/ros/eloquent/share --ignore-src --rosdistro eloquent -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"`

# colcon

## Installation

- `sudo apt install python3-colcon-common-extensions`

## build

### arguments

- `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace (saves time)
- `--symlink-install` [doc](https://design.ros2.org/articles/ament.html#optional-symlinked-install)
    - praktisch für development, reduziert das ständige Recompiling
    - This enables the developer to change the resources in the source space and skipping the installation step in many situations.
        - C++: For CMake packages this is achieved by optionally overriding the CMake `install()` function. 
        - Python: For Python packages the **development mode** is used to install the package.
    - "saves you from having to rebuild every time you tweak python scripts"
- `--event-handlers console_direct+` shows console output while building (can otherwise be found in the log directory)
- `--cmake-args -DCMAKE_BUILD_TYPE=Release`
- `--parallel-workers NUMBER` [doc](https://colcon.readthedocs.io/en/released/reference/executor-arguments.html)
    - The maximum number of jobs to process in parallel. The default value is the number of logical CPU cores as reported by `os.cpu_count()`.

# ros2 pkg

## create

### arguments

- `--build-type`: `ament_cmake`
- `--dependencies` will automatically add the necessary dependency lines to `package.xml` and `CMakeLists.txt`

# Publisher-Subscriber

## Queue Size

from: [source](https://stackoverflow.com/a/60554760/12282296)

A real life example can be a scenario of a waiter and a kitchen to wash dishes. Suppose the costumers ends its meals and the waiter takes their dirty dishes to wash in the kitchen. He puts in a table. Whenever the dishwasher can, he goes to table and gets dishes and take to wash. In normal operation the table is never filled. But if someone else give another task to the dishwasher guy, the table will start to get full. Until some time the waiter can't place dishes anymore and leave tables dirty (problem in the system). But if table is artificially large there (let's say 1000 square units) the waiter will likely fulfill its job even if dishwasher is busy, considering that after some time he will be able to return to clean dishes.

# ros2 param

## Pass Parameter via command line

- `ros2 run package_name executable_name --ros-args -p param_name:=param_value`
    - e.g. `ros2 run demo_nodes_cpp parameter_blackboard --ros-args -p some_int:=42 -p "a_string:=Hello world" -p "some_lists.some_integers:=[1, 2, 3, 4]" -p "some_lists.some_doubles:=[3.14, 2.718]"`

# Passing ROS2 Arguments via cmd line

- [ros doc](https://docs.ros.org/en/foxy/How-To-Guides/Node-arguments.html#parameters)

## set log-level debug / show debug information

- [ros doc](https://docs.ros.org/en/foxy/Tutorials/Logging-and-logger-configuration.html)
    - `ros2 run logging_demo logging_demo_main --ros-args --log-level debug`

# Gazebo

## reopen the GUI window

from: [source](https://answers.gazebosim.org/question/541/what-is-a-simple-way-to-run-gazebo-with-no-graphics/?answer=544#post-id-544)

> When you launch gazebo, there are two programs that are run: gzserver and gzclient. The program gzserver does all the simulation, even rendering for camera simulation. The interactive window is opened by gzclient.
> To open a specific world, type gzserver specific.world.

I.e. simply reopen the GUI by entering the command `gzclient`.

## adding models

- `wget -q -R *index.html*,*.tar.gz --no-parent -r -x -nH http://models.gazebosim.org/cardboard_box/` download model cardboard_box

## Xacro Macros (.xacro)

- s. [http://wiki.ros.org/xacro#Macros](http://wiki.ros.org/xacro#Macros)
- nicht vergessen: `*origin` in `param="suffix *origin"` ist **kein** Parameter und muss ein separates Element sein (s. [Link](http://wiki.ros.org/xacro#Macros) oben)!

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

## Fiese Fehler

```bash
/usr/lib/gcc/x86_64-linux-gnu/7/../../../x86_64-linux-gnu/Scrt1.o: In function `_start':
(.text+0x20): undefined reference to `main'
collect2: error: ld returned 1 exit status
make[2]: *** [HoughLinesNode] Error 1
make[1]: *** [CMakeFiles/HoughLinesNode.dir/all] Error 2
make: *** [all] Error 2
---
Failed   <<< galaxis_HoughLines_pkg [12.8s, exited with code 2]
```

- this simply means you forgot to write a `main.cpp` or a `main()` function for your ROS node
