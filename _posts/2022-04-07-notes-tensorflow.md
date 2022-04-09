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
[//]: # (      url: "https://htmlpreview.github.io/?https://github.com/pharath/home/blob/master/_posts_html/2021-09-23-Databases.html")
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

# Compatibility of Tensorflow, Python version, Compiler, Build tools, cuDNN and CUDA

see [List](https://www.tensorflow.org/install/source#gpu)

Check the CUDA version (paths might differ slightly depending on the cuda version):

`cat /usr/local/cuda/version.txt`

and cuDNN version:

`grep CUDNN_MAJOR -A 2 /usr/local/cuda/include/cudnn.h`

# Check available GPUs

## For Tensorflow 2

`tf.config.list_physical_devices('GPU')`

`print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))`

# Porting Tensorflow 1 to Tensorflow 2

## General

[List of Corresponding Commands](https://docs.google.com/spreadsheets/d/1FLFJLzg7WNP6JHODX5q8BDgptKafq_slHpnHVbJIteQ/edit#gid=0)

## GraphDef

Error: module 'tensorflow' has no attribute 'GraphDef'

[source](https://stackoverflow.com/a/58222195)

Yeah, the syntax has changed in T2.0. Here's the correct piece:

```python
tf.compat.v1.GraphDef()   # -> instead of tf.GraphDef()
tf.compat.v2.io.gfile.GFile()   # -> instead of tf.gfile.GFile()
```

## Session

`tf.compat.v1.Session()` instead of `tf.Session()`
