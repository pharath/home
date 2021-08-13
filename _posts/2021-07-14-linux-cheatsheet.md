---
title: "Linux Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - linux
  - cheatsheet
---

1. [Apps](#apps)
2. [My aliases](#my-aliases)
3. [General Commands](#general-commands)
4. [apt, apt-get, snap, dpkg](#apt-apt-get-snap-dpkg)
5. [System information](#system-information)
6. [chmod, Groups, docker](#chmod-groups-docker)
7. [Bash](#bash) 
8. [Unzipping](#unzipping)
9. [Manage Drives (hard drive, usb flash drive)](#manage-drives-hard-drive-usb-flash-drive)
10. [System information 2](#system-information)
11. [Retrieval commands curl und wget](#retrieval-commands-curl-und-wget)
12. [gpg](#gpg)

# Apps:

| command | description |
| :---: | :---: |
htop |				activity monitor (sieht besser aus als “top”)
hardinfo	|		hardware info

<hr>

| :---: | :---: |
ffmpeg | mp4 to mp3 converter
ffmpeg -i foo.mp4 bar.mp3 | convert foo.mp4 to bar.mp3

<hr>

| :---: | :---: |
pyTranscriber | generates subtitles for `.mp3` files via Google Speech Recognition API using Autosub (GUI)

<hr>

| :---: | :---: |
goldendict |		dict for fast lookup (ctrl + c + c)

<hr>

| :---: | :---: |
pycharm-community |

<hr>

| :---: | :---: |
docker |
xhost +local:root |	enable GUI for docker
docker login registry.git.rwth-aachen.de |
docker pull |

see also [other commands](##Groups_docker)

<hr>

| :---: | :---: |
retext markdown_file.md	|			edit markdown_file.md
retext —preview markdown_file.md |		preview markdown_file.md
|**Tipp:**| Shortcuts: s. Menu -> File und Edit
ctrl + e |								preview on/off
ctrl + l |								live preview on/off (die live updates brauchen manchmal bisschen)

<hr>

| :---: | :---: |
eog	|			picture viewer (shortcuts: https://help.gnome.org/users/eog/stable/index.html.en)

<hr>

| :---: | :---: |
pinta	|		picture editor (shortcuts: https://www.pinta-project.com/user-guide/shortcuts/)

<hr>

| :---: | :---: |
gedit |			texteditor
zum Lesen:	| unter F10/Preferences/Font & Colors/ Font ändern zu “TeX Gyre Termes Math Regular”
ctrl + h	|		find and replace (halte im “Find & Replace”-Fenster `alt` gedrückt für schnelle Auswahl der Optionen)
F10	|			menu (u.a. Shortcuts)
F1	|			help, Shortcut overview

<hr>

| :---: | :---: |
kazam |			screen recorder

<hr>

| :---: | :---: |
joplin |			Notes
alt + entsprechende Taste im menu |				im Menu stehen alle Shortcuts !
ctrl + l |			change view (editor/markdown viewer/both)
F10	|			show all notebooks sidebar
F11	|			show all Notes sidebar
ctrl + shift + l |		focus note selection
ctrl + shift + b |	focus body
ctrl + shift + n |	focus title

<hr>

| :---: | :---: |
dconf-editor | zB `gsettings set org.gnome.desktop.interface clock-show-weekday true` geht irgendwie nicht, stattdessen in dconf-editor zu org.gnome.desktop.interface navigieren und clock-show-weekday aktivieren. 

<hr>

| :---: | :---: |
lm-sensors | get CPU temperature (using command `sensors`)

<hr>

| :---: | :---: |
telegram-desktop | Telegram
zoom-client |
discord |

<hr>

| :---: | :---: |
ticker | stock monitor

<hr>

| :---: | :---: |
Tor-Browser-Bundle Webdownload | installation: see [here](https://wiki.ubuntuusers.de/Tor/Installation/#Tor-Browser-Bundle-Webdownload)

<hr>

| :---: | :---: |
inxi -Fxz |        inxi  - Command line system information script for console and IRC

<hr>

| :---: | :---: |
cuda | [installation](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#removing-cuda-tk-and-driver) via .deb file 
nvidia-cuda-toolkit | manuell installiert mit `sudo apt install nvidia-cuda-toolkit`, nachdem cuda per .deb file installiert wurde 
nvidia-docker2 | [installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

# My aliases

`alias listssids='/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport /usr/local/bin/airport'`

In ~/.bashrc (Ubuntu default):
`alias l='ls -CF'` <br />
`alias la='ls -A'` <br />
`alias ll='ls -alF'` <br />
(die restlichen Ubuntu alias gehen nicht bei Macbook Pro Mid 2010 wegen Doppelbindestrich Argument —color=auto)

In ~/.bash_aliases:

`alias phth_ticker='ticker --config ~/snap/ticker/common/.ticker.yaml'`

# General commands

| command | description |
| :---: | :---: |
ctrl + r + Suchbegriff	|	reverse search (mehrmals ctrl + r drücken, um zwischen den Suchbegriff enthaltenden commands auszuwählen, danach “->” um zu übernehmen bzw “enter” um auszuführen)
-|-
sudo -i	|				run single command with root privileges (does not require root password !)
sudo -s		|			run single command with root privileges (does not require root password !) + do not change user and working directory
-|-
su	|					switches to super user (root user) (requires root password !) (in Ubuntu: root account disabled by default for improved security)
-|-
xdg-open file	|			open file using default application
gio open file		|		same as xdg-open, but depends on what desktop the user has installed, whereas xdg-open is desktop agnostic
-|-
top	|					activity monitor
ps | wie `top`, aber keine real-time updates (dh. nur ein snapshot)

# apt, apt-get, snap, dpkg

## Difference between apt and apt-get + apt-cache:
- `apt` = most commonly used command options from `apt-get` and `apt-cache` see [here](https://itsfoss.com/apt-vs-apt-get-difference/)
- So with `apt`, you get all the necessary tools in one place. You won’t be lost under tons of command options. The main aim of `apt` is to provide an efficient way of handling packages in a way “pleasant for end users”.
- `apt`:
    - shows progress bar while installing or removing a program
    - prompts number of packages that can be upgraded when you update the repository database (i.e. `apt update`)
    - same can be achieved with apt-get (but you need additional options)

| command | description |
| :---: | :---: |
sudo apt update			|
sudo apt [-y] upgrade		|	-y oder —yes für automatic yes to prompts	
apt --help |
sudo apt autoremove |   remove not needed packages (NOTE: This command will remove all unused packages (orphaned dependencies). Explicitly installed packages will remain.)

## dpkg

| command | description |
| :---: | :---: |
| sudo dpkg -l \| less	| list all installed dpkg packages [meaning of tags ii, rc, ...](https://askubuntu.com/questions/18804/what-do-the-various-dpkg-flags-like-ii-rc-mean)
-|-
|**Tipp:**|AM BESTEN DIE FOLGENDEN 3 ALLE AUSFÜHREN, DA JEDER EINEN ANDEREN OUTPUT HAT !
sudo dpkg -l package	|		confirm whether package is already installed (wenn nicht installed, dann wird “no packages found matching package” angezeigt) (ACHTUNG: exakten Namen schreiben, zB “lua” findet “lua5.1” nicht !) 
sudo dpkg -l \| grep package	|	confirm whether package is already installed (wenn nicht installed, dann wird nichts angezeigt) (ACHTUNG regexp: zB “lua” findet “lua5.1” ! )
sudo dpkg-query -s package	| prüfe ob package installiert ist (ACHTUNG regexp: exakten Namen schreiben, zB “lua” findet “lua5.1” nicht !) und print weitere Informationen zum package

see also [how-to-show-history-of-installed-packages](https://www.linuxuprising.com/2019/01/how-to-show-history-of-installed.html)

| command | description |
| :---: | :---: |
grep " install \\| remove " /var/log/dpkg.log |			list recently installed OR removed packages (in the current month)
grep " install " /var/log/dpkg.log.1 |		list recently installed packages (in the previous month)
zgrep " install " /var/log/dpkg.log.2.gz |	list recently installed packages (go back 2 months, same for >2 months)
vim /var/log/apt/history.log | view apt history
-|-
sudo dpkg -i package_file.deb |	install package_file.deb
sudo apt remove package |		uninstall package_file.deb
-|-
dpkg -l \| grep ^..r   |   list all broken packages (**r** state (on the third field) means: reinst-required (package broken, reinstallation required))
-|-
sudo vim /var/lib/dpkg/info/nvidia-cuda-toolkit.list | in /var/lib/dpkg/info/ sind die installation files (.conffiles, .list, .md5sums) für alle packages (hier: nvidia-cuda-toolkit)

## snap

| command | description |
| :---: | :---: |
snap find *package* |
sudo snap install *package* |
sudo snap remove *package* |

# System information

| command | description |
| :---: | :---: |
cat /etc/os-release	 |	Ubuntu Version (lang)
lsb_release -a |			Ubuntu Version (kurz)
hostnamectl |				Ubuntu Version (mittel) mit Linux Kernel Version
-|-
uname -r | 				Linux Kernel Version
uname -a |				print system information

# chmod, Groups, docker

<a name="Groups_docker"></a>

| command | description |
| :---: | :---: |
chmod *<permissions> <file>* |     is an abbreviation of change mode. A file's mode is the set of permissions attached to it that control access. Zu *<permissions>*: s. [here](https://askubuntu.com/tags/chmod/info).
-|-
groups |					list all groups the currently logged in user belongs to (first group is the primary group)
groups user |				same as “groups”, but for specific user “user”
id |						“
id user	|				“
-|-
less /etc/group	|		view all groups present on the system
cat /etc/group |			“
getent group |				“
-|-
getent group docker |		list all members of group docker
-|-
sudo groupadd docker |		add new group docker
sudo usermod -aG docker $USER |	add my user to the docker group
newgrp docker |			log out and log back in so that group membership is re-evaluated (nach group Änderungen); wenn das nicht geht, reboot
sudo ls /var/lib/docker/overlay2 | hier ist der Großteil aller docker image Daten
sudo du -sh $(ls /var/lib/docker/) | list size of all files and dirs in /var/lib/docker/

<hr>

| :---: | :---: |
sudo docker ps |
sudo docker images |
sudo docker commit 308aeb468339 tensorflow/tensorflow:latest-gpu-jupyter_braket | [Schritte](https://stackoverflow.com/a/64532554) 
sudo docker image rm 1ff0175b5104 | remove image with id 1ff0175b5104 

## container

###tensorflow/tensorflow

Quickstart: [examples for using the tags](https://hub.docker.com/r/tensorflow/tensorflow/) or [tensorflow.org examples](https://www.tensorflow.org/install/docker)

| :---: | :---: |
sudo docker run -it --rm --runtime=nvidia -v $(realpath ~/notebooks):/tf/notebooks -p 8888:8888 tensorflow/tensorflow:latest-gpu-jupyter_braket | mit jupyter, GPU support und mit meinen zusätzlichen (über apt installierten) packages

# bash

| command | description |
| :---: | :---: |
echo Variable |		display content of Variable
-|-	
$PATH |			Variable, die alle Pfade enthält, in denen Shell-Programme/Shell-Befehle (ls, echo, df, nautilus, etc.) gesucht werden
which python3 |		
whereis python3	|
-|-
find /opt/ -iname pattern |						find all files (hier: in dir /opt/ ), for which base of file name (path with leading dirs removed) matches shell pattern pattern (Achtung: pattern muss genau übereinstimmen! Falls Endung unbekannt, mit Sternchen `*` am Ende suchen, dh. `pattern\*` statt `pattern` suchen (wie bei `ls` Befehl).
find /opt/ -name pattern |						wie -iname, aber case-sensitive
find /opt/ -iname pattern -type f |					nur files suchen
find /opt/ -iname pattern -type d |					nur dirs suchen
find /opt/ ( -iname pattern1 -o -iname pattern2 ) |	-o für oder
find /opt/ -size +1G |							nur files, die über 1GB groß sind
-|-
which *Shell-program*	| display path of Shell-program
-|-
fn + links	|		scrolle nach ganz oben
-|-
cmd + oben	|	focus letzte input Zeile (zB gut, wenn man zB schnell hochscrollen will)
-|-
tty	|			zeigt Namen des aktiven terminals
-|-
ls -ltr /dev/ttys\*	| zeigt Namen aller aktiven terminals 
-|-
last | 				zeige letzte terminal logins
-|-
vim &lt;(ls -ltr)	|	zeige Output eines Befehls in vim (ACHTUNG: Leerzeichen hinter “vim” nicht vergessen!)
|oder:
ls -ltr \| vim - |
-|-
Befehl \| head -3 |	zeige oberste 3 Zeilen des Outputs
-|-
Befehl \| tail -3 |
-|-
du -sch ./folder \| sort -rh \| head -5	|	zeige disk usage (=size) of folder (-h für human readable; -s für zeige auch Subdirectories; -c für zeige grand total am Ende) (sort -rh für sortiere nach size, wobei -r für reverse und -h für compare human readable sizes)
-|-
whoami	|		print the user name associated with the current effective user ID 
-|-
nautilus .	|	öffne current directory in File Browser
-|-
\`# ein comment\` |	Kommentar in command line
-|-
pwd		|		zeige current working directory
-|-
mkdir -p /file/subfile/subsubfile	|	erstellt file und subfile automatisch, falls sie noch nicht existieren
-|-
**Tipp:** | IMMER -iv BENUTZEN ! (-i für bestätigen, -v für ausgeführte Aktion zeigen)
mv -iv |
rm -iv |
cp -iv |

# Unzipping

| command | description |
| :---: | :---: |
unzip file -d destination	|	unzip to destination
tar -C ./data/ -zxvf ~/Downloads/mnist.tgz | für .tgz (wobei -C target_location -zxvf source.tgz), .tar.gz
oder andersrum: |
tar -zxvf ~/Downloads/mnist.tgz -C ./data/ |
tar -C ./data/ -jxvf ~/Downloads/datei.tar.bz2 | für .tar.bz2 (dh. -j flag statt -z flag)
tar -C ~/ -xvf tor-browser-linux64-10.5.2_en-US.tar.xz | für .tar.xz

# Manage Drives (hard drive, usb flash drive)

| command | description |
| :---: | :---: |
diskutil list |
diskutil info /dev/disk2s2 |
-|-
sudo diskutil mountDisk /dev/disk2s2 | (Partitionsname disk2s2 steht in rechter Spalte bei diskutil list; /dev/disk2 mounted alle Unterpartitionen)
sudo diskutil umountDisk /dev/disk2s2 |
mount_ntfs -o r “/Volumes/Volume node” | (r für read-only; rw für read-write (NICHT MACHEN! Es gibt einen Grund warum das bei Mac per default nicht geht!)
-|-
df		|		zeige alle Laufwerke, ganz rechts steht die Location mit dem Inhalt des Datenträgers (zB /media/bra-ket/UBUNTU 20_0)
sudo fdisk -l	|	wie df, aber mehr Details
lsusb |
lsblk |

# System information:

| command | description |
| :---: | :---: |
lscpu |
lshw |
hwinfo —short |
lspci |
lsscsi |
lsusb |
inxi -Fx |
lsblk |
df -H |
pydf |
sudo fdisk -l |
mount \| column -t |
free -m |
-|-
sudo dmidecode -t processor |
sudo dmidecode -t memory |
sudo dmidecode -t bios |
-|-
cat /proc/cpuinfo |
cat /proc/meminfo |
cat /proc/version |
cat /proc/scsi/scsi |
cat /proc/partitions |
-|-
sudo hdparm -i /dev/sda |

# Retrieval commands curl und wget:

| command | description |
| :---: | :---: |
wget -O output_file -q https://checkip.amazonaws.com	|	-O output_file: benutze Minuszeichen “-“ statt output_file wenn output direkt in Terminal erscheinen soll; -q für quiet
curl -s https://checkip.amazonaws.com		|			-s für silent

# gpg

| command | description |
| :---: | :---: |
gpg --list-keys | list your keys
gpg --delete-keys A3C4F0F979CAA22CDBA8F512EE8CBC9E886DDD89 | delete key A3C4F0F979CAA22CDBA8F512EE8CBC9E886DDD89 from keyring
