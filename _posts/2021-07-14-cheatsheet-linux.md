---
title: "Linux Cheatsheet"
read_time: false
excerpt: "Some essential Linux commands"
header:
  teaser: /assets/images/linux_teaser.jpg
  overlay_image: /assets/images/linux_teaser.jpg
  overlay_filter: 0.5 
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - linux
  - cheatsheet
---

# Apps

| command | description |
| :---: | :---: |
htop |				activity monitor (sieht besser aus als "top")
hardinfo	|		hardware info

<hr>

ffmpeg | mp4 to mp3 converter
ffmpeg -i foo.mp4 bar.mp3 | convert foo.mp4 to bar.mp3
ffmpeg -i source.mp4 -ss 00:00:00 -t 00:00:00 -vcodec copy -acodec copy outsplice.mp4 | crop source.mp4 from start time `-ss` to time `-t`

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

<hr>

doxygen | create class diagram for C++ projects (Doxygen  is  a  documentation system for C++, C, Java, Objective-C, IDL (Corba and Microsoft flavors), Fortran, Python, VHDL and to some extent PHP, C#, and D.)
doxywizard | a tool to configure and run doxygen on your source files

<hr>

qt5 | installiere via offline installer `qt-opensource-linux-x64-5.9.5.run`, weil qtcreator nur so automatisch qt konfiguriert (s. bookmarks in CS\/coding\/docker\/)
qtcreator | for GUI development

<hr>

python2.7 | einfach per `apt python2.7` installieren; starten per `python2.7` (nur `python2` geht nicht!)

<hr>

ocrmypdf | `ocrmypdf in.pdf out.pdf`

<hr>

ipad_charge | [github link](https://github.com/mkorenkov/ipad_charge/wiki) automatically start charging ipad when connected to ubuntu

<hr>

sudo apt-get install ruby-full |
sudo apt install rubygems-integration | nicht nötig, wenn ruby-full installiert wurde
sudo gem install bundler |
sudo bundle install | im github-pages repo root folder
bundle exec jekyll serve | warten bis "Server running... press ctrl-c to stop." message und dann ctrl gedrückt halten und auf server address clicken (oder in Browser "http://localhost:4000" aufrufen)

<hr>

(nicht mehr) sudo apt install powerline | maybe install https://github.com/powerline/fonts, if symbols (e.g. branch symbol in git repo) are not displayed correctly	
sudo apt install fish | install fish shell (add `exec fish` in `.bashrc` to start automatically; in `fish_config` theme `fish default` wählen)
curl https://raw.githubusercontent.com/oh-my-fish/oh-my-fish/master/bin/install \| fish | install oh my fish (necessary for bobthefish)
omf install bobthefish | install powerline in fish

<hr>

sudo snap install postman | http client 

<hr>

dive | for docker image dependencies tree (see [github repo](https://github.com/wagoodman/dive))

<hr>

barrier | share keyboard and mouse between multiple computers (client-server model) [**Note:** uncheck "enable SSL" box in barrier "Menu" -> "Change Settings" (on all devices)]

<hr>

backgroundremover | `pip install backgroundremover` and then download `u2net.pth` from `https://drive.google.com/uc?id=1ao1ovG1Qtx4b7EoskHXmi2E9rp5CHLcZ` (or `https://drive.google.com/file/d/1ao1ovG1Qtx4b7EoskHXmi2E9rp5CHLcZ/view`) and save it as `/home/bra-ket/.u2net/u2net.pth`

<hr>

v4l2-ctl | for setting webcam powerline frequency (see below)

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

# System Folder

~/.local/share/Trash/files | `rm FILE` command moves `FILE` to this location

# General commands

| command | description |
| :---: | :---: |
ctrl + r + Suchbegriff	|	reverse search (mehrmals ctrl + r drücken, um zwischen den Suchbegriff enthaltenden commands auszuwählen, danach “->” um zu übernehmen bzw “enter” um auszuführen)

<hr>

sudo -i	|				run single command with root privileges (does not require root password !)
sudo -s		|			run single command with root privileges (does not require root password !) + do not change user and working directory

<hr>

*some_command* & | run process *some_command* in the background
nohup gedit & | start gedit in the background AND do not stop gedit, when shell is stopped. (Dies war ein einfaches Beispiel, aber es macht den eigentlichen Nutzen klar, wenn man z.B. per SSH auf einem fremden Rechner arbeitet und dort einen langwierigen Prozess starten möchte, die ssh-Verbindung aber während des Prozesses nicht permanent aktiv sein soll, weil man etwa den eigenen Rechner ausschalten möchte.)

<hr>

su	|					switches to super user (root user) (requires root password !) (in Ubuntu: root account disabled by default for improved security)

<hr>

ln -s FILE LINK | create symlink
readlink -f LINK | show symlink target \[**ACHTUNG**: das heißt **nicht**, dass das target auch existiert, s.i [LINK](https://serverfault.com/a/76049) !\] 

<hr>

xdg-open file	|			open file using default application
gio open file		|		same as xdg-open, but depends on what desktop the user has installed, whereas xdg-open is desktop agnostic

<hr>

top	|					activity monitor
ps | wie `top`, aber keine real-time updates (dh. nur ein snapshot)
echo $$ | show PID of current shell
kill *PID* | stop process with id *PID*

<hr>

realpath foo.bar | get path to file "foo.bar" (like `pwd` + foo.bar)
readlink -f foo.bar | get path to file "foo.bar" (like `pwd` + foo.bar)

<hr>

ls \| wc -l | count files in a directory
history \| tail -n 30 | show last 30 commands

# Check installed libraries

- `ldconfig -p | grep libnvinfer_plugin.so`

# apt, apt-get, snap, dpkg, pkg-config

## Difference between apt and apt-get + apt-cache:

- `apt` = most commonly used command options from `apt-get` and `apt-cache` see [here](https://itsfoss.com/apt-vs-apt-get-difference/)
- So with `apt`, you get all the necessary tools in one place. You won’t be lost under tons of command options. The main aim of `apt` is to provide an efficient way of handling packages in a way “pleasant for end users”.
- `apt`:
    - shows progress bar while installing or removing a program
    - prompts number of packages that can be upgraded when you update the repository database (i.e. `apt update`)
    - same can be achieved with apt-get (but you need additional options)
- When you use `apt` to install a package, under the hood it uses `dpkg`. When you install a package using `apt`, it first creates a list of all the dependencies and downloads it from the repository.
    - Once the download is finished it calls `dpkg` to install all those files, satisfying all the dependencies.

| command | description |
| :---: | :---: |
sudo apt update			|
sudo apt [-y] upgrade		|	-y oder —yes für automatic yes to prompts	
apt --help |
sudo apt autoremove |   remove not needed packages (NOTE: This command will remove all unused packages (orphaned dependencies). Explicitly installed packages will remain.)
sudo apt-mark auto $PACKAGES | mark $PACKAGES as "automatically installed", if accidentally marked as "manually installed"

## apt

### Install

| command | description |
| :---: | :---: |
sudo apt install ./name.deb | install a .deb file
`sudo apt-get install <package name>=<version>` | install a specific version

### Uninstall

| command | description |
| :---: | :---: |
sudo apt purge ... | Removing packages with `sudo apt purge ...` or `sudo apt --purge remove ...` will remove them and all their global (i.e., systemwide) configuration files. This is usually what people mean when they talk about completely removing a package. **This does not remove packages that were installed as dependencies**, when you installed the package you're now removing. Assuming those packages aren't dependencies of any other packages, and that you haven't marked them as manually installed, you can remove the dependencies with `sudo apt autoremove` or (if you want to delete their systemwide configuration files too) `sudo apt --purge autoremove`.
sudo apt --purge remove ... | see `sudo apt purge ...`

### Check

| command | description |
| :---: | :---: |
`apt-cache policy <package name>` | shows installed package version and also all the available versions in the repository according to the version of Ubuntu in which you are running

### apt-mark

| command | description |
| :---: | :---: |
`sudo apt-mark unhold package_name` | unhold "held" packages (i.e. packages which cannot be upgraded, removed or modified)

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

sudo dpkg -i package_file.deb |	install package_file.deb (alternative: `sudo apt install ./name.deb`)
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

## pkg-config

| command | description |
| :---: | :---: |
| man pkg-config | description of all pkg-config flags
| pkg-config --libs-only-l json-c | was man im CMakeLists.txt in `target_link_libraries` eintragen muss (hier: `-ljson-c` Achtung: das `-l` muss auch im CMakeLists.txt rein!)
| pkg-config --libs-only-L json-c | location of .so library file (hier: in ubuntu 18.04: findet er nicht, ist aber in `/lib/x86_64-linux-gnu`; in ubuntu 20.04: `-L/usr/local/lib`) (see also: [difference .so vs .a libraries](https://stackoverflow.com/a/9810368/12282296)) (muss nicht in CMakeLists.txt rein, s. [Minimalbsp](https://github.com/pharath/home/tree/master/assets/_code_examples/jsonc))
| pkg-config --cflags json-c | include paths of the corresponding library with .h header files (hier: in ubuntu 18.04: `-I/usr/include/json-c`; in ubuntu 20.04: `-I/usr/local/include -I/usr/local/include/json-c`) (muss nicht in CMakeLists.txt rein)

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

# bash

## terminal shortcuts

| command | description |
| :---: | :---: |
ctrl + w | delete the word in front of the cursor
alt + d | delete the word after the cursor
ctrl + a | jump to the beginning of the line
ctrl + e | jump to the end of the line
ctrl + s | freeze/block terminal
ctrl + q | unfreeze/unblock terminal

## terminal commands and bash scripting

| command | description |
| :---: | :---: |
echo Variable |		display content of the variable "Variable"
echo $$ | display PID of current shell
bash | start new bash shell instance in current bash shell (the new shell will have a different PID than the old one, check shell PID via `echo $$`)
exec > some_file | redirect all shell output to some_file
cat /etc/shells | list all shells
chsh | change shell (you will be prompted to enter one of the shells in `cat /etc/shells`)

<hr>

do_something1 && do_something2_that_depended_on_something1 | only run "something2", if "something1" completes successfully
do_something1; do_something2 | run "something2" irrespective of "something1"

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

sed 's/unix/linux/' geekfile.txt | replaces the word 'unix' with 'linux' in the file 'geekfile.txt'. `sed` is mostly used to replace text in a file. Examples: see [here](https://www.geeksforgeeks.org/sed-command-in-linux-unix-with-examples/).

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
echo "blabla" \| tee *filename* | write output to file *filename*

<hr>

history | get a list of the last 1000 commands 
history \| grep command_to_search | search some pattern within the history generated list

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

| command | description |
| :---: | :---: |
sudo diskutil mountDisk /dev/disk2s2 | (Partitionsname disk2s2 steht in rechter Spalte bei diskutil list; /dev/disk2 mounted alle Unterpartitionen)
sudo diskutil umountDisk /dev/disk2s2 |
mount_ntfs -o r “/Volumes/Volume node” | (r für read-only; rw für read-write (NICHT MACHEN! Es gibt einen Grund warum das bei Mac per default nicht geht!)

<hr>

| command | description |
| :---: | :---: |
df		|		zeige alle Laufwerke, ganz rechts steht die Location mit dem Inhalt des Datenträgers (zB /media/bra-ket/UBUNTU 20_0)
sudo fdisk -l	|	wie df, aber mehr Details
lsusb |
lsblk |

# System information

## Hardware

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

| command | description |
| :---: | :---: |
sudo dmidecode -t processor |
sudo dmidecode -t memory |
sudo dmidecode -t bios |

| command | description |
| :---: | :---: |
cat /proc/cpuinfo |
cat /proc/meminfo |
cat /proc/version |
cat /proc/scsi/scsi |
cat /proc/partitions |

## Hard disk

| command | description |
| :---: | :---: |
sudo hdparm -i /dev/sda |

## Disk Usage

| command | description |
| :---: | :---: |
`du -sh *` | 
`du -sh * | sort -h` | 
`du -sh * .[^.]*` | 
`du -sh * .[^.]* | sort -h` | 

## Webcam

| command | description |
| :---: | :---: |
`vlc v4l2:///dev/video0` | check, if device `/dev/video0` is the webcam
`v4l2-ctl --list-devices` | list cameras
`v4l2-ctl -L -d /dev/videoN` | list available settings of camera `/dev/videoN`
`v4l2-ctl --set-ctrl power_line_frequency=0 -d /dev/videoN` | set powerline frequency (50Hz vs 60Hz) of camera `/dev/videoN`

# GPU information

| command | description |
| :---: | :---: |
nvidia-smi -q -d temperature | temperature info including critical temperature values, shutdown temperature etc.
nvidia-smi --query-gpu=name --format=csv | get GPU name

# Retrieval commands curl und wget:

| command | description |
| :---: | :---: |
wget -O output_file -q https://checkip.amazonaws.com -P DESTINATION	|	-O output_file: benutze Minuszeichen “-“ statt output_file wenn output direkt in Terminal erscheinen soll; -q für quiet; -P für Zielordner
wget -A pdf,jpg -m -p -E -k -K -np http://site/path/ | get all pdfs and jpgs from site
wget --accept pdf,jpg --mirror --page-requisites --adjust-extension --convert-links --backup-converted --no-parent http://site/path/ | same as above using long option names
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
scp *source* *target* | immer Anführungszeichen um den *source* Pfad setzen!
scp -rv Macbook:"~/Desktop/Uni/FS1/Essential\ Astrophysics\ WS1819" ~/Desktop/ | spaces DOPPELT escapen (hier: 1. mit " **UND** 2. mit \) 
`scp -r [!.]* source target` | exclude hidden files

## rsync

| command | description |
| :---: | :---: |
rsync -a *source* *destination* | copy directory (**Warning**: -r tag does not copy some stuff, e.g. symlinks)
`rsync -a --exclude="SomeDirForPythonInstall"` | exclude directory "SomeDirForPythonInstall"
`rsync -a --exclude=".*"` | excludes hidden files and directories
`rsync -a --exclude=".*/"` | exclude hidden directories only

### Resume partially scp-transferred files using Rsync

[source](https://ostechnix.com/how-to-resume-partially-downloaded-or-transferred-files-using-rsync/)

| command | description |
| :---: | :---: |
`rsync -P -rsh=ssh ubuntu.iso sk@192.168.225.22:/home/sk/` | resume partially transferred file `ubuntu.iso`
`rsync --partial -rsh=ssh ubuntu.iso sk@192.168.225.22:/home/sk/` | see above
`rsync -avP ubuntu.iso sk@192.168.225.22:/home/sk/` | see above
`rsync -av --partial ubuntu.iso sk@192.168.225.22:/home/sk/` | see above

# tree

tree -H ./ > result.html | save directory tree to file 
firefox ./result.html | view html tree created by `tree` command
