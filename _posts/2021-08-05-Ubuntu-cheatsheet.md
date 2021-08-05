---
title: "Ubuntu Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
- Cheatsheet
tags:
- ubuntu
- cheatsheet
---

1. [General Shortcuts](#general-shortcuts)

# General Shortcuts

## For Bluetooth

- Create .desktop file (".desktop" ending required!) with this content

```bash
[Desktop Entry]
Name=open_bluetooth_settings
Exec=gnome-control-center bluetooth
Terminal=false
Type=Application
```
- Make sure that your script is executable, like this:

```bash
sudo chmod +x /path/to/script.sh  
```

(if it does not execute, copy it to ~/Desktop/. There right-click on it and select "Allow Launching".)

![allow launching pic](https://i.ibb.co/2ZQfnGY/allow-launching.png)

- Move .desktop file to `/usr/share/applications/`. Press `Super` to open Activities and in the application grid view select "All" (instead of "Frequent") and search the created .desktop file there. Right-click on it and select "Add to Favorites". Now the .desktop file should be in the dock and can be accessed via shortcut `Super + <position_in_dock>`.

- It also won't work with if your script uses the sudo command, or anything else that requires user input.

If you want it to open a terminal window when you run it (if you needed to add input or watch the output) set Terminal to true.

```bash
Terminal=true
```
