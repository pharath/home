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

```bash
sudo apt-key del 7fa2af80
# Install signing key:
## Recommended way (cuda-keyring package):
# Set $distro = ubuntu2004/x86_64
wget https://developer.download.nvidia.com/compute/cuda/repos/$distro/$arch/cuda-keyring_1.0-1_all.deb
sudo dpkg -i cuda-keyring_1.0-1_all.deb
## Alternative way (manually):
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/$distro/$arch/3bf863cc.pub
```
