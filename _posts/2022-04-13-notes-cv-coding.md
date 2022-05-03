---
title: "Notes on Computer Vision Coding"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Notes
  - Computer_Vision
  - OpenCV
tags:
  - notes
  - computer_vision
  - opencv
  - cv2
toc: true
toc_label: "Contents"

---

# pickle

- use the following code to inspect and view `.pickle` or `.p` datasets

```bash
import pickle
import matplotlib.pyplot as plt 

with open('train.p', 'rb') as data:
    # "im" is a dictionary! Get list of keys via im.keys() method.
    im = pickle.load(data)
    # without print there is no output in the terminal!
    #print(im["features"])
    # display image
    plt.imshow(im["features"][0])
    plt.show()
```

# Converting Images

## Mogrify

- `mogrify -format jpg *.ppm`

## Iterate through Folders

- e.g. when images are stored in subfolders and you want to run a convert on each of the images in each folder
- `for dir in ./*; do mogrify -format jpg $dir/*.ppm; done`
