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

# apps:

| command | description |
| :---: | :---: |
htop |				activity monitor (sieht besser aus als “top”)
hardinfo	|		hardware info

| command | description |
| :---: | :---: |
ffmpeg | mp4 to mp3 converter
ffmpeg -i foo.mp4 bar.mp3 | convert foo.mp4 to bar.mp3

| command | description |
| :---: | :---: |
goldendict |		dict for fast lookup (ctrl + c + c)

| command | description |
| :---: | :---: |
pycharm-community |

| command | description |
| :---: | :---: |
docker |
xhost +local:root |	enable GUI for docker
docker login registry.git.rwth-aachen.de |
docker pull |

see also [other commands](##Groups_docker)

| command | description |
| :---: | :---: |
retext markdown_file.md	|			edit markdown_file.md
retext —preview markdown_file.md |		preview markdown_file.md
|**Tipp:**| Shortcuts: s. Menu -> File und Edit
ctrl + e |								preview on/off
ctrl + l |								live preview on/off (die live updates brauchen manchmal bisschen)

| command | description |
| :---: | :---: |
eog	|			picture viewer (shortcuts: https://help.gnome.org/users/eog/stable/index.html.en)

| command | description |
| :---: | :---: |
pinta	|		picture editor (shortcuts: https://www.pinta-project.com/user-guide/shortcuts/)

| command | description |
| :---: | :---: |
gedit |			texteditor
zum Lesen:	| unter F10/Preferences/Font & Colors/ Font ändern zu “TeX Gyre Termes Math Regular”
ctrl + h	|		find and replace (halte im “Find & Replace”-Fenster `alt` gedrückt für schnelle Auswahl der Optionen)
F10	|			menu (u.a. Shortcuts)
F1	|			help, Shortcut overview

| command | description |
| :---: | :---: |
kazam |			screen recorder

| command | description |
| :---: | :---: |
joplin |			Notes
alt + entsprechende Taste im menu |				im Menu stehen alle Shortcuts !
ctrl + l |			change view (editor/markdown viewer/both)
F10	|			show all notebooks sidebar
F11	|			show all Notes sidebar
ctrl + shift + l |		focus note selection
ctrl + shift + b |	focus body
ctrl + shift + n |	focus title

| command | description |
| :---: | :---: |
dconf-editor | zB `gsettings set org.gnome.desktop.interface clock-show-weekday true` geht irgendwie nicht, stattdessen in dconf-editor zu org.gnome.desktop.interface navigieren und clock-show-weekday aktivieren. 

# My aliases

`alias listssids='/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport /usr/local/bin/airport'`

`alias l='ls -CF'` <br />
`alias la='ls -A'` <br />
`alias ll='ls -alF'` <br />
(die restlichen Ubuntu alias gehen nicht wegen Doppelbindestrich Argument —color=auto)

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

## dpkg

| command | description |
| :---: | :---: |
| sudo dpkg -l \| less	| list all installed dpkg packages
-|-
|**Tipp:**|AM BESTEN DIE FOLGENDEN 3 ALLE AUSFÜHREN, DA JEDER EINEN ANDEREN OUTPUT HAT !
sudo dpkg -l package	|		confirm whether package is already installed (wenn nicht installed, dann wird “no packages found matching package” angezeigt) (ACHTUNG: exakten Namen schreiben, zB “lua” findet “lua5.1” nicht !) 
sudo dpkg -l \| grep package	|	confirm whether package is already installed (wenn nicht installed, dann wird nichts angezeigt) (ACHTUNG regexp: zB “lua” findet “lua5.1” ! )
sudo dpkg-query -s package	| prüfe ob package installiert ist (ACHTUNG regexp: exakten Namen schreiben, zB “lua” findet “lua5.1” nicht !) und print weitere Informationen zum package

see also [how-to-show-history-of-installed-packages](https://www.linuxuprising.com/2019/01/how-to-show-history-of-installed.html)

| command | description |
| :---: | :---: |
grep " install " /var/log/dpkg.log |			list recently installed packages (in the current month)
grep " install " /var/log/dpkg.log.1 |		list recently installed packages (in the previous month)
zgrep " install " /var/log/dpkg.log.2.gz |	list recently installed packages (go back 2 months, same for >2 months)
-|-
sudo dpkg -i package_file.deb |	install package_file.deb
sudo apt remove package |		uninstall package_file.deb

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

# Groups, docker
<a name="Groups_docker"></a>

| command | description |
| :---: | :---: |
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
newgrp docker |			log out and log back in so that group membership is re-evaluated (nach group Änderungen)

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

