---
title: "Notes"
read_time: true 
excerpt_separator: "<!--more-->"
categories:
  - Notes 
tags:
  - notes
---

1. docker starten mit `./scripts/start_docker.sh`
2. mehrmals compilen mit `./compile.sh`
3. `source /opt/ros/eloquent/setup.bash` (check, ob alles stimmt: `ros2 run demo_nodes_cpp talker` und in anderem terminal (hier ohne docker) `ros2 run demo_nodes_cpp listener`)

4.-8. ist optional:
4. `sudo rosdep init`
5. `rosdep update`
6. `echo $ROS_DISTRO` um ros distro rauszufinden
7. Resolve dependencies `rosdep install -i --from-path src --rosdistro eloquent -y`
8. follow [Tutorial](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#tasks)

9. `source ./install/setup.bash` (check, ob alles stimmt: `ros2 pkg executables | vim -` zeigt jetzt neben den ROS2 standard packages u.a. verschiedene `galaxis_` packages an)
10. `ros2 run <package> <executable>` zB `ros2 run gal<tab1> <tab2>`, wobei `<tab1>` um package zu vervollständigen und `<tab2>` um executable auszuwählen.

