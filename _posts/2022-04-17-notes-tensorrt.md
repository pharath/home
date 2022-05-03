---
title: "TensorRT"
excerpt: "Notes on TensorRT."
classes: wide
header:
  teaser: /assets/images/lenet.png
  overlay_image: /assets/images/lenet.png
  overlay_filter: 0.6
  caption: "Photo credit: [**Yann LeCun**](http://yann.lecun.com/)"
  actions:
    - label: "Some Content"
    - url: "https://htmlpreview.github.io/?https://github.com/pharath/home/blob/master/_posts_html/2021-09-23-Databases.html"
categories:
  - TensorRT
  - Machine_Learning
tags:
  - tensorrt
  - ml
toc: true
toc_label: "Contents"
last_modified_at: 2022-04-17T16:00:52-04:00

---

# Install

- WARNING: `apt upgrade` maybe upgrades `libcudnn8` which removes all tensorrt libs! `apt-mark hold` all `libcudnn8` packages before installation!
- Debian installation, else no samples, no trtexec, ... etc.
    - compile trtexec in samples and put `export PATH=/usr/src/tensorrt/bin${PATH:+:${PATH}}` in `.bashrc`
- [Install using pip wheel file](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing-pip)

## Check installation

- `import tensorrt as trt`
- `print(trt.__version__)`
- `assert trt.Builder(trt.Logger())` (there should be no output, if installed successfully)

# Conversion

- There are three main options for converting a model with TensorRT:
    - using TF-TRT
    - automatic ONNX conversion from .onnx files
    - manually constructing a network using the TensorRT API (either in C++ or Python)
- For converting TensorFlow models, the TensorFlow integration (**TF-TRT**) provides both model conversion and a high-level runtime API, and has the capability to fall back to TensorFlow implementations where TensorRT does not support a particular operator. For more information about supported operators, refer to the Supported Ops section in the NVIDIA TensorRT Support Matrix.
- A more performant option for automatic model conversion and deployment is to convert using **ONNX**. ONNX is a framework agnostic option that works with models in TensorFlow, PyTorch, and more. TensorRT supports automatic conversion from ONNX files using either the TensorRT API, or trtexec - the latter being what we will use in this guide. ONNX conversion is all-or-nothing, meaning all operations in your model must be supported by TensorRT (or you must provide custom plugins for unsupported operations). The end result of ONNX conversion is a singular TensorRT engine that allows less overhead than using TF-TRT.
- For the most performance and customizability possible, you can also construct TensorRT engines **manually using the TensorRT network definition API**.

## keras model (.h5) to SavedModel (.pb)

```python
import tensorflow as tf

model = tf.keras.models.load_model('/home/bra-ket/git/Galaxis/ws_270322_trafficsign/keras-YOLOv3-model-set/weights/yolov3_training_last.h5')
tf.saved_model.save(model, '/home/bra-ket/git/Galaxis/ws_270322_trafficsign/keras-YOLOv3-model-set/weights/yolov3_training_last_saved_model')
```

## tf SavedModel (.pb, .pbtxt) to .onnx

- `python -m tf2onnx.convert --saved-model weights/yolov3_training_last_saved_model/ --output weights/yolov3_training_last.onnx`

# Deployment using ONNX

- PyTorch natively supports ONNX export. For TensorFlow, the recommended method is tf2onnx. [TensorRT doc](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html#working_tf)

## Export to .onnx format

- see section "Conversion"

## Batch Size

- We set the batch size **during the original export process** to ONNX.
- Batch size can have a large effect on the optimizations TensorRT performs on our model. Generally speaking, at inference, we pick a **small batch size** when we want to **prioritize latency** and a **larger batch size** when we want to **prioritize throughput**. Larger batches take longer to process but reduce the average time spent on each sample.
- TensorRT is capable of handling the batch size **dynamically** if you don’t know until runtime what batch size you will need. That said, a fixed batch size allows TensorRT to make additional optimizations.

## Precision

- We set the precision that our TensorRT engine should use **at runtime**.
- Inference typically requires less numeric precision than training. With some care, lower precision can give you faster computation and lower memory consumption without sacrificing any meaningful accuracy. TensorRT supports TF32, FP32, FP16, and INT8 precisions.

## Convert to .trt engine

# Deployment using UFF (Universal Framework Format)

- deprecated (starting with TensorRT 7.0)
    - ONNX is now the recommended way
        - "Starting with TensorRT 7.0, the Universal Framework Format (UFF) is being deprecated. In this post, you learn how to deploy TensorFlow trained deep learning models using the **new TensorFlow-ONNX-TensorRT workflow**." [Nvidia Blog post](https://www.edge-ai-vision.com/2020/04/speeding-up-deep-learning-inference-using-tensorflow-onnx-and-tensorrt/)
