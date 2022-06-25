---
title: "Ubuntu Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
toc: true
toc_sticky: true
categories:
- Cheatsheet
tags:
- ubuntu
- cheatsheet
---

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
# Network

## On the server computer:

1. Connect server and client via Ethernet cable.
2. On the computer, which is connected to the Internet, click the network icon in the panel and go to "Settings" at the bottom of the menu. [der Schieberegler von "Wired" beim Server sollte bereits aktiv sein, der Schieberegler beim Client sollte inaktiv sein (auch nachdem Kabel eingesteckt wurde)]
3. Click on the "Settings symbol" of your Wired Connection (Leave your wireless connection untouched).
4. On the "IPv4 Settings tab", select Method: "Shared to other computers". Click on "Apply" and close this window.
5. On the server reconnect by clicking on the Wired Network's toggle switch [dh. einmal aus und an], so it gets a new IP address. (The two computers must be connected by an ethernet cable for this step, so connect them now if you haven't already.) [Toggle Switch auf Client sollte noch inactive sein]
6. **Do not change IP settings. Leave everything as it is.**

## On the client computer:

1. Connect simply via toggle switch of Wired Connection. **IP settings are configured automatically!**

Mostly from: [Instructions](https://askubuntu.com/questions/359856/share-wireless-internet-connection-through-ethernet)

# Power

- [source](https://unix.stackexchange.com/a/317933) The Settings which determine what happens, when the Battery Level is critically low are in `/etc/UPower/UPower.conf`. There, the relevant entries are:
```bash
PercentageAction=2
CriticalPowerAction=HybridSleep
```

- for laptops: [source](https://www.kernel.org/doc/Documentation/ABI/testing/sysfs-class-power)
   - `cat /sys/class/power_supply/BAT0/capacity` outputs laptop battery level (in percent)
   - `cat /sys/class/power_supply/BAT0/status` outputs laptop battery status (values: "Unknown", "Charging", "Discharging", "Not charging", "Full")
   - `cat /sys/class/power_supply/BAT0/capacity_level` outputs laptop battery level description (values: "Unknown", "Critical", "Low", "Normal", "High", "Full"). From doc: "Coarse representation of battery capacity."

# Appearance

## Restart GUI

Press Alt + F2, then type `r` and press Enter.

other methods: [https://linuxconfig.org/how-to-restart-gui-on-ubuntu-20-04-focal-fossa](https://linuxconfig.org/how-to-restart-gui-on-ubuntu-20-04-focal-fossa)

## Dock like MacOS


```bash
gsettings set org.gnome.shell.extensions.dash-to-dock extend-height false

gsettings set org.gnome.shell.extensions.dash-to-dock show-show-apps-button false 

gsettings set org.gnome.shell.extensions.dash-to-dock show-trash true
```

- add Launcher icon to dock: via .desktop icon to favorites using "[launch show-apps from command line](https://www.reddit.com/r/gnome/comments/emkaxp/invoking_gnome_show_applications_from_command_line/)"

## Hide Dock permanently

see [source](https://www.linuxuprising.com/2018/08/how-to-remove-or-disable-ubuntu-dock.html) Option 3

Hide:

```bash
gsettings set org.gnome.shell.extensions.dash-to-dock autohide false

gsettings set org.gnome.shell.extensions.dash-to-dock dock-fixed false

gsettings set org.gnome.shell.extensions.dash-to-dock intellihide false
```

Show:

```bash
gsettings set org.gnome.shell.extensions.dash-to-dock autohide true

gsettings set org.gnome.shell.extensions.dash-to-dock dock-fixed true

gsettings set org.gnome.shell.extensions.dash-to-dock intellihide true
```
# Default Applications

## Config files

- `/usr/share/applications/defaults.list` (root)
- `/etc/gnome/defaults.list` (root)
- `~/.config/mimeapps.list` (local user)

## Change default app

```bash
xdg-mime default some_app some_filetype
```

E.g. 

```bash
xdg-mime default org.kde.okular.desktop application/pdf
```

creates an entry in the local MIME database `~/.config/mimeapps.list`.

## Change default mail client

If not already present, add
 
```bash
MimeType=x-scheme-handler/mailto;x-scheme-handler/mailspring;
```

to `~/.local/share/applications/mailspring_mailspring.desktop` and run 

```bash
update-desktop-database ~/.local/share/applications
```

There should be a new line `x-scheme-handler/mailto=mailspring_mailspring.desktop` in `~/.config/mimeapps.list` now.

## "Open with ..." and set default app for a given file

```bash
mimeopen -d myfile.pdf
```

will give you a list of applications that can open the file, and (the `-d` flag) will also update the default application for you. *Note*: After running this command the default works for `mimeopen myfile.pdf` only. `xdg-open myfile.pdf` and `nautilus` defaults need to be specified via `xdg-mime default some_app some_filetype` (see above)!

# Converter

## pptx (or ppt) to pdf

- `libreoffice --headless --invisible --convert-to pdf *.ppt`

# Webcam

## Troubleshooting

### Flickering

- caused by 50Hz vs 60Hz powerline frequency differences (see [source](https://blog.christophersmart.com/2017/02/07/fixing-webcam-flicker-in-linux-with-udev/))
    - **solution**: try `v4l2-ctl --set-ctrl power_line_frequency=0` or `v4l2-ctl --set-ctrl power_line_frequency=1`
