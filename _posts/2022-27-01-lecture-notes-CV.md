---
title: "Machine Learning (Part 3)"
excerpt: "Notes on Computer Vision theory. Based on Stanford CS231n."
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
  - Lecture_Notes
  - Machine_Learning
  - Computer_Vision
tags:
  - lecture_notes
  - ml
  - cv
toc: true
toc_label: "Contents"

---

# CNNs

![conv_layer_params_cs231n_karpathy.png](/assets/images/conv_layer_params_cs231n_karpathy.png)

## Why is the size of $F$ always an odd number?

- usually $1$, $3$, $5$, $7$ and sometimes $11$
- you can use filters with even numbers, but it is not common
- the lowest people use is $3$, mainly for convenience
    - you want to apply the filter around a well-defined position that "exists" in the input volume (see traditional image processing)

## Why pad with zeros ?

- you do not want the outer padded pixels to contribute to the dot product
- but you can also use other padding techniques, but it is less common, in practice

