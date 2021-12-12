---
title: "Linux Networking Cheatsheet"
read_time: false
excerpt: "Some essential Linux networking commands"
header:
  teaser: /assets/images/linux_teaser.jpg
  overlay_image: /assets/images/linux_teaser.jpg
  overlay_filter: 0.5 
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - network
  - cheatsheet
---

# MacOS

```bash
networksetup -setairportpower en1 off

networksetup -setairportpower en1 on

Ifconfig

networksetup ( ohne parameter: zeigt alle möglichen parameter )

alias listssids='/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport /usr/local/bin/airport' (Achtung: ‘ und nicht “ als Anführungszeichen benutzen!)
=> diese Zeile in “vim ~/.bash_profile” einfügen
=> “listssids -s” kann dann alle SSIDs abrufen

networksetup -setairportnetwork en1 'bra-ket’s iPhone' 'muster_password' (Achtung: das alt-Apostroph in “bra-ket’s” muss über <cmd> + <shift> + < ‘ > eingegeben werden)
```

# Linux

## ssh

```bash
ssh -l xy123456 login-g.hpc.itc.rwth-aachen.de
```

## scp

```bash
scp xy123456@login-g.hpc.itc.rwth-aachen.de:~/experiments/some_data_on_remote_computer .

scp some_data_on_my_computer xy123456@login-g.hpc.itc.rwth-aachen.de:~/experiments/
```

## port forwarding für jupyter auf HPC Aachen:

```bash
# 1. on server: 
jupyter lab --no-browser --port=8889

# 2. on host: 
ssh -N -f -L localhost:8888:localhost:8889 -l xy123456 login-g.hpc.itc.rwth-aachen.de

# 3. und dann im Browser zuhause localhost:8888 aufmachen.
```

## ssh keys:

```bash
ssh-keygen -t key_type -C “commit mail address”

# Beispiel:
ssh-keygen -t ed25519 -C “muster@gmail.com” # Generiert private key file “id_ed25519” und public key file “id_ed25519.pub” in ~/.ssh/ (per default)

ssh-keygen -p -f ~/.ssh/id_ed25519 # bisheriges Passwort zum ssh key, der sich in id_ed25519 befindet, ändern 

ssh -T git@git.rwth-aachen.de # (sollte “Welcome to GitLab, @username!” ausgeben)
```
