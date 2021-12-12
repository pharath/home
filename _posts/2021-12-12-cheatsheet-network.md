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

# HPC

HPC Cluster
(ausführliche Dokumentation:
https://capi.sabio.itc.rwth-aachen.de/documents/0962a984657bb2e901657bc9e2810004/1jifm42ljvvhw/c010e5a1bed64d79aa9dd1fc62838bb0/primer-8.4.0.pdf)

Setup Routine für jedes Mal bevor Python 3 benutzt wird:

```bash
module switch intel gcc
export PYTHONPATH=$PYTHONPATH:$HOME/SomeDirForPythonInstall/lib/python3.9.1/site-packages
module load python/3.9.1
module list
```

aus S. 32 in Dok:

```bash
man module

module avail				list all available modules

module list				list currently loaded module files

module help python/3.9.1 	Manual zu modul ‘python/3.9.1’

module unload modulename	NICHT BENUTZEN ! (stattdessen “module switch” s.u.)
```

If you want to use another version of a software (e.g., another compiler), we strongly recommend switching between modules:

```bash
module switch oldmodule newmodule
```

Install python packages in `$HOME/SomeDirForPythonInstall`:
Python software often rely on GCC compilers and often do not work [out of the box] with Intel compilers; thus type `module switch intel gcc` before adding any Py-Modules.

```bash
$ module switch intel gcc
$ export PYTHONPATH=$PYTHONPATH:$HOME/SomeDirForPythonInstall/lib/python3.9.1/site-packages
$ pip3 install numpy
```

# SLURM

[https://help.itc.rwth-aachen.de/en/service/rhr4fjjutttf/article/387ec8d754244084aee155b5a54e9d70/](https://help.itc.rwth-aachen.de/en/service/rhr4fjjutttf/article/387ec8d754244084aee155b5a54e9d70/)

```bash
top # show running processes
sbatch jobscript.sh # submitting a jupyter-notebook server as a slurm batch job
squeue -u$(whoami) # show currently running jobs
scancel JOBID # Jupyter Server schließen
```

```bash
nvidia-smi				GPU Usage Info
```

# Für Jupyter Lab

Quelle: (https://ljvmiranda921.github.io/notebook/2018/01/31/running-a-jupyter-notebook/)[https://ljvmiranda921.github.io/notebook/2018/01/31/running-a-jupyter-notebook/]

```bash
export PYTHONPATH=$PYTHONPATH:$HOME/SomeDirForPythonInstall/lib/python3.9.1/site-packages
module switch intel gcc
module load python/3.9.1
pip3 install tensorflow
export PATH=/home/no411606/.local/bin:$PATH
pip3 install jupyterlab
```

```bash
# auf HPC server: 
jupyter lab --no-browser --port=8889
# auf meinem PC:
# 1. 
ssh -N -f -L localhost:8888:localhost:8889 -l no411606 login-g.hpc.itc.rwth-aachen.de (nur ausführen. Dieser command gibt keinen Output im Terminal !)
# 2. im Browser auf meinem PC:
# 2.1. localhost:8888 in der Adresszeile eingeben und starten
# 2.2. Als Passwort im Browser den Token aus der jupyterlab URL (im Output zum command “jupyter lab --no-browser --port=8889”) im HPC Terminal rauskopieren und einfügen
# 2.3. Passwort/Token bestätigen und jupyterlab startet.
# 3. Beim beenden: auf meinem PC: mit 
sudo netstat -lpn |grep :8888 
# pid suchen und mit 
kill some_pid # pid steht in der Spalte ganz rechts, zB. in "1239526/ssh" ist die pid vor dem slash
# beenden. Denn port auf HPC Seite (hier: 8889) wird automatisch frei beim Beenden von jupyter, aber port auf meinem PC (hier: 8888) nicht!
```
**Achtung**: Schau, dass die ports frei sind (mit (sudo) netstat -lpn |grep :8889 die pid suchen und mit “kill ABCDEF” port frei machen)! Sonst anderen port probieren.
