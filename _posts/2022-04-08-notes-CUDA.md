---
title: "Notes on CUDA and cuDNN"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Notes
  - CUDA
  - cuDNN
tags:
  - cuda
  - cudnn
  - notes
toc: true
toc_label: "Contents"

---

# Currently installed

- CUDA 11.2 Update 2
- cuDNN 8.1.1.33

# Install

- install CUDA normally
- [install cuDNN](https://docs.nvidia.com/deeplearning/cudnn/archives/cudnn-811/install-guide/index.html#cudnn-package-manager-installation-overview) **without** `sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/${OS}/x86_64/ /"`, but use `sudo dpkg -i cuda-keyring_1.0-1_all.deb` instead [Nvidia Blog Post](https://developer.nvidia.com/blog/updating-the-cuda-linux-gpg-repository-key/)!
- [install nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html), sonst funktionieren die Container nicht!

# Troubleshooting

## 1

```bash
`fatal error: cuda_runtime_api.h: No such file or directory`
```

- Is `/usr/local/cuda` a symlink to `/usr/local/cuda-10.1`?
    - Use the location of this symlink as `CUDA_VER` in a docker container!
- What's the output of `echo $LD_LIBRARY_PATH`?

## 2

```bash
W: GPG error: https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64  InRelease: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY A4B469963BF863CC
E: The repository 'https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64  InRelease' is no longer signed.
N: Updating from such a repository can't be done securely, and is therefore disabled by default.
N: See apt-secure(8) manpage for repository creation and user configuration details.
```

- [Nvidia Blog Post](https://developer.nvidia.com/blog/updating-the-cuda-linux-gpg-repository-key/)
    - Hilft nicht !!! Einfach CUDA neu installieren.
        - In `sudo vim /etc/apt/sources.list` und `sudo vim /etc/apt/sources.list.d/` alle CUDA Repos entfernen !!!
            - **Warning**: `sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/${OS}/x86_64/ /"` fügt in `/etc/apt/sources.list` eine Zeile hinzu, die zu Warnungen in `sudo apt update` führt!
        - Evtl. `sudo rm /etc/apt/sources.list.d/cuda-ubuntu2004-11-2-local.list.save` (falls es existiert) notwendig, sowie `rm` auf alle anderen alten Cuda Reste

