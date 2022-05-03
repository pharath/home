---
title: "Tensorflow"
excerpt: "Notes on Tensorflow."
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
  - Tensorflow 
  - Machine_Learning
tags:
  - tensorflow
  - ml
toc: true
toc_label: "Contents"
last_modified_at: 2021-09-23T16:00:52-04:00

---

# Compatibility (Tensorflow, Python version, Compiler, Build tools, cuDNN and CUDA)

- see [List](https://www.tensorflow.org/install/source#gpu)
- Check the CUDA version (paths might differ slightly depending on the cuda version): `cat /usr/local/cuda/version.txt`
- and cuDNN version: `grep CUDNN_MAJOR -A 2 /usr/local/cuda/include/cudnn.h`

# Check GPUs

## Tensorflow 2

- `tf.config.list_physical_devices('GPU')`
- `print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))`

# Porting (Tensorflow 1 to Tensorflow 2)

## General

- [List of Corresponding Commands](https://docs.google.com/spreadsheets/d/1FLFJLzg7WNP6JHODX5q8BDgptKafq_slHpnHVbJIteQ/edit#gid=0)

## GraphDef

```bash
Error: module 'tensorflow' has no attribute 'GraphDef'
```

- [source](https://stackoverflow.com/a/58222195)
    - Yeah, the syntax has changed in T2.0. Here's the correct piece:

```python
tf.compat.v1.GraphDef()   # -> instead of tf.GraphDef()
tf.compat.v2.io.gfile.GFile()   # -> instead of tf.gfile.GFile()
```

## Session

- `tf.compat.v1.Session()` instead of `tf.Session()`

# Tensorboard

- `tensorboard --logdir=dir_with_eventfiles` in bash terminal 
    - this will start the tensorboard server and then tensorboard can be viewed e.g. in firefox

# tfrecords

## Inspecting the first image in a tfrecords file

```python
import tensorflow as tf
raw_dataset = tf.data.TFRecordDataset("path-to-file")

for raw_record in raw_dataset.take(1):
    example = tf.train.Example()
    example.ParseFromString(raw_record.numpy())
    print(example)
```

- this also shows the other key-value pairs such as labels, bounding box coordinates, etc.

# Memory Formats

- [overview](https://oneapi-src.github.io/oneDNN/dev_guide_understanding_memory_formats.html)

## NCHW

- data format for **activations** 
- "NCHW" also describes the **order** in which the tensor values are laid out in memory
- [source](https://stackoverflow.com/a/67087270):
    - NCHW stands for: batch N, channels C, depth D, height H, width W
    - It is a way to store multidimensional arrays / data frames / matrix into memory, which can be considered as a 1-D array. 

## NHWC
