---
title: "Notes"
read_time: false
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
(Statt 3. und 9. kann man auch ein script `phth_script.sh` erstellen, das dieselben commands enthält und dann in jedem tmux über `. ./phth_script.sh` oder `source ./phth_script.sh` sourcen **oder** über `tmux set-option -g default-command "bash --rcfile phth_script.sh"` (command muss in einer tmux session eingegeben werden!) in allen *weiteren* tmux panes und windows automatisch sourcen (dh. im aktuellen pane muss das script noch gesourct werden, aber alle neu geöffneten panes sourcen das script automatisch). Achtung: `bash ./phth_script.sh` funktioniert nicht, weil s. [hier](https://stackoverflow.com/questions/14744904/how-to-execute-script-in-the-current-shell-on-linux/14745127) oder [hier](https://stackoverflow.com/questions/50156206/source-bash-profile-do-not-works-inside-a-bash-script/50156308))
10. `ros2 run <package> <executable>` zB `ros2 run gal<tab1> <tab2>`, wobei `<tab1>` um package zu vervollständigen und `<tab2>` um executable auszuwählen.
11. Start simulation: auf local machine: `cd ~/git/galaxis-simulation/simulation` und hier `./start_container.sh`. Dann im container `./start_simulation.sh`.
12. Start bridge to simulation: auf local machine: `cd ~/git/galaxis-simulation/bridge` und hier `./start_container.sh`. Dann im container `./start_bridge.sh`.
13. Start control terminal: auf local machine: `cd ~/git/onboarding` und hier `./scripts/start_docker.sh`. Dann im container `tmux`.
14. in tmux: ROS2 control Beispiele:

```bash
ros2 topic pub --once /galaxis/simulation/remotecontrol messages/msg/SimulationVehicleControl "car_speed: 0.0
turnangle_front: 0.0
reset: true
remote_control: -1" 

ros2 topic pub /simulation/gazebo/model/dr_drift/set_pose gazebo_simulation/msg/SetModelPose "keys: [0,1]
values: [0.4,-0.2]"
```

# tmux

## send-keys

`C-m` ist dasselbe wie `Enter`.
