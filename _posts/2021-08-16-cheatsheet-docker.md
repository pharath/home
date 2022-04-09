---
title: "Docker Notes"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Notes
tags:
  - docker
  - notes
---

1. [Welcher CMD wird per default beim image Start ausgeführt?](#welcher-cmd-wird-per-default-beim-image-start-ausgeführt) 
 
# Welcher CMD wird per default beim image Start ausgeführt? 

Klicke auf den letzten der layers in der Liste links. Dann erscheint rechts der zugehörige vollständige CMD.

![image start CMD default config](https://i.ibb.co/QcRnxP8/Screenshot-from-2021-08-16-04-53-12.png)

# Beispiele

Run osrf/ros image with GUI support:
1. `xhost +local:root`
2. `docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix osrf/ros:eloquent-desktop`  

Start new bash shell in running container:
1. `xhost +local:root`
2. `docker exec -it <tab-tab>container bash` 


Start new bash shell in running container with color support:
1. `xhost +local:root`
2. `docker exec -it <tab-tab>container env TERM=xterm-256color bash`

# Webcam

- first enable x11 forwarding

- `docker run --ipc=host --device=/dev/video0`

- Check the webcam: `mplayer tv://device=/dev/video0`
