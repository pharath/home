---
title: "Machine Learning"
excerpt: "Page formatting sucks. Click on \"Content in HTML\" button below."
header:
  teaser: /assets/images/databases.png
  overlay_image: /assets/images/databases.png
  overlay_filter: 0.5 
  caption: "Photo credit: [**Jennifer Widom**](https://cs.stanford.edu/people/widom/)"
  actions:
    - label: "Content in HTML"
[//]: # (      url: "https://htmlpreview.github.io/?https://github.com/pharath/home/blob/master/_posts_html/2021-09-23-Databases.html")
categories:
  - Lecture_Notes
  - Machine_Learning
tags:
  - lecture_notes
  - ml
toc: true
toc_sticky: true
last_modified_at: 2021-09-23T16:00:52-04:00
---

# Overfitting problem

## Possible Solutions

- reduce number of model parameters
- increase number of data points N
- regularization (in statistics: shrinkage methods)
    - quadratic: 
        - weight decay (in Deep Learning)
        - L2 regularization = ridge regression (in statistics)
