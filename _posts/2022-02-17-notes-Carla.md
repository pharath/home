---
title: "Carla Simulator"
excerpt: "Notes on the Carla Simulator"
header:
  teaser: /assets/images/lenet.png
  overlay_image: /assets/images/lenet.png
  overlay_filter: 0.6
  caption: "Photo credit: [**Yann LeCun**](http://yann.lecun.com/)"
  actions:
    - label: "Some Content"
[//]: # (      url: "https://htmlpreview.github.io/?https://github.com/pharath/home/blob/master/_posts_html/2021-09-23-Databases.html")
categories:
  - Carla
  - Simulator
  - Self_Driving
tags:
  - carla
  - simulator
  - self_driving
toc: true
toc_label: "Contents"

---

# Download

- [release list and doc list](https://github.com/carla-simulator/carla/blob/master/Docs/download.md)
    - package contains 
        - a precompiled version of the simulator, 
        - the Python API module and 
        - some scripts to be used as examples. 

# Install

## Install Client library

- `pip3 install --upgrade pip` (because pip3 version 20.3 or higher is required)
- `pip3 install carla==0.9.12`, when you use Carla version 0.9.12.

## Install dependencies for example scripts

- CARLA_0.9.12/PythonAPI/carla: `pip3 install -r carla/requirements.txt`
- CARLA_0.9.12/PythonAPI/examples: `pip3 install -r examples/requirements.txt`

# ROS_bridge

- after installation: `[bridge-1] ModuleNotFoundError: No module named 'derived_object_msgs'
    - `sudo apt install ros-foxy-derived-object-msgs`

# Imitation Learning

- download Carla 0.8.2
- copy folder `PythonClient/carla` from Carla 0.8.2 into `ws_170222/imitation-learning`
- `pip install numpy scipy tensorflow-gpu==1.1 pillow`
