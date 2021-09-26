---
title: "Linux Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
excerpt: "Some essential Linux commands"
header:
  teaser: /assets/images/linux_teaser.jpg
  overlay_image: /assets/images/linux_teaser.jpg
  overlay_filter: 0.5 
<!--  caption: "Photo credit: [**who**](https://www.who.com/)"
  actions:
    - label: "Learn more"
      url: "https://www.tesla.com/" -->
toc: true
toc_sticky: true
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
6. [chmod, Groups](#chmod-groups)
7. [Docker](#docker)
8. [Bash](#bash) 
9. [Unzipping](#unzipping)
10. [Manage Drives (hard drive, usb flash drive)](#manage-drives-hard-drive-usb-flash-drive)
11. [System information 2](#system-information)
12. [Retrieval commands curl und wget](#retrieval-commands-curl-und-wget)
13. [gpg](#gpg)
14. [cron](#cron)
15. [network](#network)
16. [GPU](#gpu)

# Apps:

| command | description |
| :---: | :---: |
htop |				activity monitor (sieht besser aus als “top”)
hardinfo	|		hardware info

<hr>

ffmpeg | mp4 to mp3 converter
ffmpeg -i foo.mp4 bar.mp3 | convert foo.mp4 to bar.mp3

<hr>

pyTranscriber | generates subtitles for `.mp3` files via Google Speech Recognition API using Autosub (GUI)

<hr>

goldendict |		dict for fast lookup (ctrl + c + c)

<hr>

pycharm-community |

<hr>

docker |

<hr>

retext markdown_file.md	|			edit markdown_file.md
retext —preview markdown_file.md |		preview markdown_file.md
|**Tipp:**| Shortcuts: s. Menu -> File und Edit
ctrl + e |								preview on/off
ctrl + l |								live preview on/off (die live updates brauchen manchmal bisschen)

<hr>

eog	|			picture viewer (shortcuts: https://help.gnome.org/users/eog/stable/index.html.en)

<hr>

pinta	|		picture editor (shortcuts: https://www.pinta-project.com/user-guide/shortcuts/)

<hr>

gedit |			texteditor
zum Lesen:	| unter F10/Preferences/Font & Colors/ Font ändern zu “TeX Gyre Termes Math Regular”
ctrl + h	|		find and replace (halte im “Find & Replace”-Fenster `alt` gedrückt für schnelle Auswahl der Optionen)
F10	|			menu (u.a. Shortcuts)
F1	|			help, Shortcut overview

<hr>

kazam |			screen recorder

<hr>

joplin |			Notes
alt + entsprechende Taste im menu |				im Menu stehen alle Shortcuts !
ctrl + l |			change view (editor/markdown viewer/both)
F10	|			show all notebooks sidebar
F11	|			show all Notes sidebar
ctrl + shift + l |		focus note selection
ctrl + shift + b |	focus body
ctrl + shift + n |	focus title

<hr>

dconf-editor | zB `gsettings set org.gnome.desktop.interface clock-show-weekday true` geht irgendwie nicht, stattdessen in dconf-editor zu org.gnome.desktop.interface navigieren und clock-show-weekday aktivieren. 

<hr>

lm-sensors | get CPU temperature (using command `sensors`)

<hr>

telegram-desktop | Telegram
zoom-client |
discord |

<hr>

ticker | stock monitor

<hr>

Tor-Browser-Bundle Webdownload | installation: see [here](https://wiki.ubuntuusers.de/Tor/Installation/#Tor-Browser-Bundle-Webdownload)

<hr>

inxi -Fxz |        inxi  - Command line system information script for console and IRC
inxi -G | get Graphics info, eg. display resolution, GPU, etc.

<hr>

cuda | [installation](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#removing-cuda-tk-and-driver) via .deb file 
nvidia-cuda-toolkit | manuell installiert mit `sudo apt install nvidia-cuda-toolkit`, nachdem cuda per .deb file installiert wurde 
nvidia-docker2 | [installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

<hr>

droidcam | use Android smartphone cam as Ubuntu webcam

<hr>

psensor | Unter "sensor preferences" im Tab "Application Indicator" das Kästchen "Display sensor in the label" aktivieren, damit ein bestimmter Wert im System Tray angezeigt wird.
conky | see [configuration](https://linuxconfig.org/ubuntu-20-04-system-monitoring-with-conky-widgets)

<hr>

mailspring | mail client similar to Apple Mail

<hr>

peek | screen2gif recorder

<hr>

sqlite3 | A command line interface for SQLite version 3
sqlitebrowser | light GUI editor for SQLite databases

<hr>

sudo apt-get install gnome-tweaks | GNOME tweak tool

<hr>

sudo apt-get install dconf-editor | dconf editor

<hr>

plank | dock similar to macOS

<hr>

whatsapp-for-linux | whatsapp

<hr>

1. sudo apt-get install compiz compiz-gnome compiz-plugins-extra | compiz Fenstermanager dependencies
2. sudo apt install compizconfig-settings-manager | compiz Fenstermanager

# My aliases

`alias listssids='/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport /usr/local/bin/airport'`

In ~/.bashrc (Ubuntu default):
`alias l='ls -CF'` <br />
`alias la='ls -A'` <br />
`alias ll='ls -alF'` <br />
(die restlichen Ubuntu alias gehen nicht bei Macbook Pro Mid 2010 wegen Doppelbindestrich Argument —color=auto)

In ~/.bash_aliases:

`alias phth_ticker='ticker --config ~/snap/ticker/common/.ticker.yaml'`

| command | description |
| :---: | :---: |
alias | List all aliases
type \<some_alias\> | check the meaning of a specific alias

# General commands

| command | description |
| :---: | :---: |
ctrl + r + Suchbegriff	|	reverse search (mehrmals ctrl + r drücken, um zwischen den Suchbegriff enthaltenden commands auszuwählen, danach “->” um zu übernehmen bzw “enter” um auszuführen)

<hr>

sudo -i	|				run single command with root privileges (does not require root password !)
sudo -s		|			run single command with root privileges (does not require root password !) + do not change user and working directory

<hr>

su	|					switches to super user (root user) (requires root password !) (in Ubuntu: root account disabled by default for improved security)

<hr>

xdg-open file	|			open file using default application
gio open file		|		same as xdg-open, but depends on what desktop the user has installed, whereas xdg-open is desktop agnostic

<hr>

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
sudo apt-mark auto $PACKAGES | mark $PACKAGES as "automatically installed", if accidentally marked as "manually installed"

## dpkg

| command | description |
| :---: | :---: |
| sudo dpkg -l \| less	| list all installed dpkg packages [meaning of tags ii, rc, ...](https://askubuntu.com/questions/18804/what-do-the-various-dpkg-flags-like-ii-rc-mean)

<hr>

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

<hr>

sudo dpkg -i package_file.deb |	install package_file.deb
sudo apt remove package |		uninstall package_file.deb

<hr>

dpkg -l \| grep ^..r   |   list all broken packages (**r** state (on the third field) means: reinst-required (package broken, reinstallation required))

<hr>

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

<hr>

uname -r | 				Linux Kernel Version
uname -a |				print system information

# chmod, Groups

| command | description |
| :---: | :---: |
chmod *permissions file* |     is an abbreviation of change mode. A files mode is the set of permissions attached to it that control access. Zu *permissions*: s. [here](https://askubuntu.com/tags/chmod/info).

<hr>

groups |					list all groups the currently logged in user belongs to (first group is the primary group)
groups user |				same as “groups”, but for specific user “user”
id |						“
id user	|				“

<hr>

less /etc/group	|		view all groups present on the system
cat /etc/group |			“
getent group |				“

<hr>

getent group docker |		list all members of group docker

<hr>

sudo groupadd docker |		add new group docker
sudo usermod -aG docker $USER |	add my user to the docker group
newgrp docker |			log out and log back in so that group membership is re-evaluated (nach group Änderungen); wenn das nicht geht, reboot

# docker

| command | description |
| :---: | :---: |
sudo ls /var/lib/docker/overlay2 | hier ist der Großteil aller docker image Daten
sudo du -sh $(ls /var/lib/docker/) | list size of all files and dirs in /var/lib/docker/

<hr>

| :---: | :---: |
xhost +local:root |	enable GUI for docker
docker login registry.git.rwth-aachen.de |
docker pull |

<hr>

| :---: | :---: |
sudo docker ps -a | -a flag: Show all containers (default shows just running)
sudo docker images | show all images

<hr>

| :---: | :---: |
sudo docker commit 308aeb468339 tensorflow/tensorflow:latest-gpu-jupyter_braket | [Schritte](https://stackoverflow.com/a/64532554) 

<hr>

| :---: | :---: |
sudo docker image rm 1ff0175b5104 | remove image with id 1ff0175b5104 
sudo docker rmi 1ff0175b5104 | alias for `docker image rm` [source](https://stackoverflow.com/a/63035352), see also [doc](http://manpages.ubuntu.com/manpages/bionic/man1/docker-rmi.1.html)

<hr>

| :---: | :---: |
sudo docker container ls -a |
sudo docker container stop 1ff0175b5104 | stoppt den container nur (dh. container Status: "Exited"), aber "docker ps -a" zeigt den container noch
sudo docker container rm 1ff0175b5104 | entfernt den container, dh. "docker ps -a" zeigt den container nicht mehr
sudo docker container kill 1ff0175b5104 | killt den container (Unterschied zu `docker container stop`: see [here](https://stackoverflow.com/a/66736836): "So ideally we always stop a container with the `docker stop` command in order to get the running process inside of it a little bit of time to shut itself down, otherwise if it feels like the container has locked up and it's not responding to the docker stop command then we could issue `docker kill` instead.")

<hr>

| :---: | :---: |
sudo docker run -d ... | start a container in detached mode [docs](https://docs.docker.com/engine/reference/run/#detached--d)

<hr>

| :---: | :---: |
sudo docker exec -it 6b594d9d60cc bash | start bash in container 6b594d9d60cc 

<hr>

| :---: | :---: |
sudo docker build --no-cache -t deep\_braket:v1 . | -t: REPO name and TAG name of image; --no-cache: [explanation](https://stackoverflow.com/a/35595021), ohne diesen flag wird Layer Caching benutzt (image updated die alte image-Version sozusagen nur und hat dependencies zur alten image-Version; die alte image-Version kann also nicht gelöscht werden!)

<hr>

| :---: | :---: |
sudo docker top 6b594d9d60cc | see all processes (incl. pids) in container 6b594d9d60cc 

<hr>

| :---: | :---: |
docker attach *double-tab* | attach to running container (double-tab shows names of running containers or use container id)
ctrl-p ctrl-q | detach from container


## Remove dangling images

**Dangling images** entstehen, wenn man ein neues image committet, das den Namen eines bereits existierenden images hat.
In `docker images` wird das alte image dann \<none\> genannt (sowohl REPOSITORY als auch TAG)

[source](https://stackoverflow.com/a/40791752)

docker images --filter dangling=true | lists all images that are dangling and has no pointer to it
docker rmi \`docker images --filter dangling=true -q\` | Removes all those images.

## container

**tensorflow/tensorflow**

Quickstart: [examples for using the tags](https://hub.docker.com/r/tensorflow/tensorflow/) or [tensorflow.org examples](https://www.tensorflow.org/install/docker)

| command | description |
| :---: | :---: |
sudo docker run -it --rm --runtime=nvidia -v $(realpath ~/notebooks):/tf/notebooks -p 8888:8888 tensorflow/tensorflow:latest-gpu-jupyter_braket | mit jupyter, GPU support und mit meinen zusätzlichen (über apt installierten) packages

# bash

| command | description |
| :---: | :---: |
echo Variable |		display content of Variable

<hr>
	
$PATH |			Variable, die alle Pfade enthält, in denen Shell-Programme/Shell-Befehle (ls, echo, df, nautilus, etc.) gesucht werden
which python3 |		
whereis python3	|

<hr>

find /opt/ -iname pattern |						find all files (hier: in dir /opt/ ), for which base of file name (path with leading dirs removed) matches shell pattern pattern (Achtung: pattern muss genau übereinstimmen! Falls Endung unbekannt, mit Sternchen `*` am Ende suchen, dh. `pattern*` statt `pattern` suchen (wie bei `ls` Befehl).
find /opt/ -name pattern |						wie -iname, aber case-sensitive
find /opt/ -iname pattern -type f |					nur files suchen
find /opt/ -iname pattern -type d |					nur dirs suchen
find /opt/ ( -iname pattern1 -o -iname pattern2 ) |	-o für oder
find /opt/ -size +1G |							nur files, die über 1GB groß sind

<hr>

which *Shell-program*	| display path of Shell-program

<hr>

fn + links	|		scrolle nach ganz oben

<hr>

cmd + oben	|	focus letzte input Zeile (zB gut, wenn man zB schnell hochscrollen will)

<hr>

tty	|			zeigt Namen des aktiven terminals

<hr>

ls -ltr /dev/ttys\*	| zeigt Namen aller aktiven terminals 

<hr>

last | 				zeige letzte terminal logins

<hr>

vim &lt;(ls -ltr)	|	zeige Output eines Befehls in vim (ACHTUNG: Leerzeichen hinter “vim” nicht vergessen!)
|oder:
ls -ltr \| vim - |

<hr>

Befehl \| head -3 |	zeige oberste 3 Zeilen des Outputs

<hr>

Befehl \| tail -3 |

<hr>

du -sch ./folder \| sort -rh \| head -5	|	zeige disk usage (=size) of folder (-h für human readable; -s für zeige auch Subdirectories; -c für zeige grand total am Ende) (sort -rh für sortiere nach size, wobei -r für reverse und -h für compare human readable sizes)

<hr>

whoami	|		print the user name associated with the current effective user ID 

<hr>

nautilus .	|	öffne current directory in File Browser

<hr>

\`# ein comment\` |	Kommentar in command line

<hr>

pwd		|		zeige current working directory

<hr>

mkdir -p /file/subfile/subsubfile	|	erstellt file und subfile automatisch, falls sie noch nicht existieren

<hr>

**Tipp:** | IMMER -iv BENUTZEN ! (-i für bestätigen, -v für ausgeführte Aktion zeigen)
mv -iv |
rm -iv |
cp -iv |

<hr>

echo "blabla" >> *filename* | write output to file *filename*
echo "blabla" | tee *filename* | write output to file *filename*

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

<hr>

sudo diskutil mountDisk /dev/disk2s2 | (Partitionsname disk2s2 steht in rechter Spalte bei diskutil list; /dev/disk2 mounted alle Unterpartitionen)
sudo diskutil umountDisk /dev/disk2s2 |
mount_ntfs -o r “/Volumes/Volume node” | (r für read-only; rw für read-write (NICHT MACHEN! Es gibt einen Grund warum das bei Mac per default nicht geht!)

<hr>

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

<hr>

sudo dmidecode -t processor |
sudo dmidecode -t memory |
sudo dmidecode -t bios |

<hr>

cat /proc/cpuinfo |
cat /proc/meminfo |
cat /proc/version |
cat /proc/scsi/scsi |
cat /proc/partitions |

<hr>

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

# cron

The software utility cron also known as cron job is a time-based job scheduler in Unix-like computer operating systems. Users who set up and maintain software environments use cron to schedule jobs to run periodically at fixed times, dates, or intervals.

| command | description |
| :---: | :---: |
crontab -e | opens a file in which jobs can be specified (read this file for more info)

# network

| command | description |
| :---: | :---: |
sudo netstat -lpn \| grep :8889 | zeigt pid des Prozesses auf port 8889 (port kann dann mit `kill \<pid\>` frei gemacht werden)
ss | 

## ssh

| command | description |
| :---: | :---: |
w | list all ssh sessions
ssh bra-ket@10.14.14.60 | installiere vorher openssh-server auf beiden Computern
firefox -no-remote -no-xshm | display firefox on local client (no -X or -Y flag needed in previous ssh command)

**Achtung**: 
- erst in den Server einloggen und **dann** erst in den Computer einloggen, der die Internetverbindung des Servers benutzt !
- **Error**: "X11 connection rejected because of wrong authentication." 
   - ~/.Xauthority löschen und nochmal per ssh einloggen kann helfen bei xauth Problemen (siehe [issue](https://unix.stackexchange.com/a/494742)) ! 
   - (prüfe evtl noch) nach [source](https://www.cyberciti.biz/faq/x11-connection-rejected-because-of-wrong-authentication/):   
      - Make sure X11 SSHD Forwarding Enabled
      - Make sure X11 client forwarding enabled

<hr>

ssh -Y bra-ket@10.14.14.60 | display graphical output on trusted local client (**Caution**: may lead to security issues), [difference -X vs -Y flag](https://askubuntu.com/a/35518)
ssh -X bra-ket@10.14.14.60 | display graphical output on untrusted local client, [difference -X vs -Y flag](https://askubuntu.com/a/35518)
export DISPLAY=localhost:10.0 | set display (use `w` or `xauth list` to list diplays) ("**:0**" ist der server monitor; zB. "**localhost:10.0**" ist der client monitor, wobei localhost:=127.0.0.1 (127.0. 0.1 is the loopback Internet protocol (IP) address also referred to as the localhost. The address is used to establish an IP connection to the same machine or computer being used by the end-user. The same convention is defined for computers that support IPv6 addressing using the connotation of ::1.)

<hr>

caffeinate -u | **for Mac**: prevent the system from sleeping and (-u for) prevent the system from sleeping [source](https://apple.stackexchange.com/questions/53802/waking-display-from-terminal-general-waking/161527)

<hr>

ssh-keygen -R 10.14.14.92 | remove 10.14.14.92 from .ssh/known_hosts (falls aus Versehen geaddet)

## scp

**Achtung:** Spaces müssen im path **DOPPELT** escapet werden ! (s. [hier](https://stackoverflow.com/a/20364170/12282296))

| command | description |
| :---: | :---: |
scp *source* *target* | 
scp -rv Macbook:"~/Desktop/Uni/FS1/Essential\ Astrophysics\ WS1819" ~/Desktop/ | spaces DOPPELT escapen (hier: 1. mit " **UND** 2. mit \) 

## rsync

| command | description |
| :---: | :---: |
rsync -a *source* *destination* | copy directory (**Warning**: -r tag does not copy some stuff, e.g. symlinks)

# GPU

| command | description |
| :---: | :---: |
nvidia-smi -q -d temperature | temperature info including critical temperature values, shutdown temperature etc.
nvidia-smi --query-gpu=name --format=csv | get GPU name
