---
title: "Notes on Deepstream, TAO Toolkit, NGC CLI"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Notes
  - Deepstream
  - TAO_Toolkit
  - NGC_CLI
tags:
  - notes
  - deepstream
  - tao_toolkit
  - ngc_cli
toc: true
toc_label: "Contents"

---

# GStreamer framework

- DeepStream SDK is based on the GStreamer framework.
- [Wikipedia](https://en.wikipedia.org/wiki/GStreamer)
    - GStreamer is a **pipeline-based** multimedia framework that links together a wide variety of media processing systems to complete complex workflows. 
        - For instance, GStreamer can be used to build a system that reads files in one format, processes them, and exports them in another. 
        - The formats and processes can be changed in a plug and play fashion.
    - GStreamer supports a wide variety of media-handling components, including simple audio playback, audio and video playback, recording, streaming and editing. 
    - **The pipeline design** serves as a base to create many types of multimedia applications such as video editors, transcoders, streaming media broadcasters and media players. 

# Install 

## TAO Toolkit

see [https://docs.nvidia.com/tao/tao-toolkit/text/tao_toolkit_quick_start_guide.html](https://docs.nvidia.com/tao/tao-toolkit/text/tao_toolkit_quick_start_guide.html)

# DeepStream Container

- [Nvidia Doc](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_docker_containers.html)
    - The dGPU container is called `deepstream` and the Jetson container is called `deepstream-l4t`.

## dGPU

### nvcr.io/nvidia/deepstream:5.1-21.02-devel

- devel docker (contains the entire SDK along with a development environment for building DeepStream applications and graph composer)

### nvcr.io/nvidia/deepstream:5.1-21.02-samples

- DeepStream samples docker (contains the runtime libraries, GStreamer plugins, reference applications and sample streams, models and configs)
    - [explanation of .cfg files](https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_sample_configs_streams.html)
- run webcam sample app: `deepstream-app -c /opt/nvidia/deepstream/deepstream-5.1/samples/configs/deepstream-app/source1_usb_dec_infer_resnet_int8.txt`

# Examples

## Jupyter Notebooks

- `pip3 install jupyter`
- `jupyter notebook --ip 0.0.0.0 --allow-root --port 8888`

## Nvidia Samples

### /opt/nvidia/deepstream/deepstream-5.1/sources/objectDetector_Yolo/

- see `README.md`
    - install dependencies first as described in `README.md`!
- compiled example: see `/home/bra-ket/Desktop/Team_Galaxis/phth/traffic-sign-detection/deepstream/erfolgreiche_tests/objectDetector_Yolo`

### apps/sample_apps/deepstream-test2/

- from deepstream_python_apps
- [doc](https://github.com/NVIDIA-AI-IOT/deepstream_python_apps)
- 4-class object detection (**not** YOLO!), tracking and attribute classification pipeline
- build first: 
    - `nvcc -V` (to see which CUDA_VER is installed)
    - `export CUDA_VER=11.1` (or whatever CUDA_VER is installed)
    - `cd apps/sample_apps/deepstream-test2`
    - `make`
    - run: `./deepstream-test2-app ../../../../samples/streams/sample_720p.h264`

## ROS2

### ros2_deepstream_ws

- see `~/ros2_deepstream_ws`

# NGC CLI

## `ngc registry model`

- [see chapter 7: ACCESSING THE MODEL REGISTRY](https://docs.nvidia.com/ngc/pdf/ngc-catalog-cli-user-guide.pdf)
- `ngc registry model list`
- `ngc registry model download-version nvidia/tao/pretrained_classification:resnet18 --dest $LOCAL_EXPERIMENT_DIR/pretrained_resnet18`
