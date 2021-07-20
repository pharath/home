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
4. `sudo rosdep init`
5. `rosdep update`
6. `echo $ROS_DISTRO` um ros distro rauszufinden
7. Resolve dependencies `rosdep install -i --from-path src --rosdistro eloquent -y`
8. follow [Tutorial](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#tasks)
