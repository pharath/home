---
title: "Notes on CUDA"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Notes
  - CUDA
tags:
  - cuda
  - notes
toc: true
toc_label: "Contents"

---

# Troubleshooting

```bash
`fatal error: cuda_runtime_api.h: No such file or directory`
```

- Is `/usr/local/cuda` a symlink to `/usr/local/cuda-10.1`?
    - Use the location of this symlink as `CUDA_VER` in a docker container!
- What's the output of `echo $LD_LIBRARY_PATH`?
