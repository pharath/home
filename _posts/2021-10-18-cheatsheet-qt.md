---
title: "Python Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - python
  - conda
  - cheatsheet
---

# Install

- am besten mit offline installer (created with Qt Installer Framework); automatische installation via script, das installer automatisch "durchklickt" s. [here](https://flames-of-code.netlify.app/blog/qt-on-docker/) and [here](https://stackoverflow.com/questions/25105269/silent-install-qt-run-installer-on-ubuntu-server); flags für CLI des offline installers [here](https://doc-snapshots.qt.io/qtifw-master/ifw-cli.html)
   - wenn man qt über apt installiert, muss man qtcreator und die qt library manuell verbinden (umständlich)
   - Vorteil: Installation weiterer Komponenten über installer möglich
   - Vorteil: Deinstallation über installer möglich und einfach

# Header files/Qt Base directory

- in /usr/include/x86_64-linux-gnu/qt5
   - zB. `#include <QtWidgets/QWidget>` oder `#include <QtGui/QPicture>`
   - includes einer kompletten library `#include <QtGui>` soll man nicht benutzen(kompletter include erhöht Compile-Zeit stark s. [Discussion](https://forum.qt.io/topic/18279/including-qtgui-or/4): "I prefer to include only what is needed and if possible only in cpp files and in headers use forward declarations.", "and it makes it harder to see what is actually used by your class", "can cause hard to fix compilation issues (unneeded interdepencies between files)")

# qt version management

| command | description |
| :---: | :---: |
qtdiag | u.a. qt version anzeigen
qtchooser -qt=*version* | Selects *version* as the Qt version to be used
qtchooser -print-env | print environment information

# qmake

| command | description |
| :---: | :---: |
qmake *file.pro* | erzeugt ein Makefile. (Befehl in dem Folder ausführen, in dem die build Dateien rein sollen)
make | diesen Befehl nach dem Befehl "qmake *file.pro*" ausführen, um den build Prozess zu starten.
