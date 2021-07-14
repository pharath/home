---
title: "Vim Cheatsheet"
excerpt_separator: "<!--more-->"
categories:
  - Blog
tags:
  - vim
  - cheatsheet
---

# help:

| command | description |
| :---: | :---: |
| :h | 
| :h i_CTRL-R	| lookup meaning of CTRL-R in insert mode (deshalb das i_ prefix; v_ prefix für visual mode usw. s. Tabelle unten drunter) |
|help :u	|		Manual zu ‘:u’ Befehl (im Manual kann man zB via Befehl ’:v’ zu dem ‘:v’ springen; Ganz oben im Manual steht wie man im Manual navigiert)

![Screenshot_table](https://i.ibb.co/jw1X1nH/Screen-Shot-2021-06-10-at-3-51-04-AM-2.png)

| command | description |
| :---: | :---: |
|:h Q\_&lt;tab&gt; |
|:h quickref	| und dann eg /_sc um auf Q_sc tag zu springen
|ctrl + ] (während cursor auf einem tag)	|		springe zu selected tag
|/pattern |	zB /^i springt zur nächsten Zeile, die mit i anfängt
|ctrl + e, d, y, u |

# regexp:
(Achtung: in Windows manchmal bisschen anders s. [https://superuser.com/a/518231](https://superuser.com/a/518231)

| command | description |
| :---: | :---: |
^	|			beginning of line ( praktisch für netrw: zB für jump to nächstem Ordner der mit “i” anfängt: /^i )
\n |	linebreak (man kann damit auch linebreaks suchen und mit einem whitespace (ie einfach 1x Leertaste) ersetzen)

# Von Shell aus:

| command | description |
| :---: | :---: |
|vim .			öffne netrw in current dir

# In netrw:

| command | description |
| :---: | :---: |
|F1	|			help
-|-
|i	|			change listing style
-|-
|d	|			new directory
|%	|			new file
-|-
|s	|			cycle through sort order
|r	|			reverse current sort order
-|-
|:Lexplore	 (in vim)	|	öffnet netrw links und jeder gewählte file wird rechts geöffnet (gut zum Browsen)

# In COMMAND mode

| command | description |
| :---: | :---: |
a |				gehe in INSERT modus
-|-
|shift + z q	|	wie :q!
|shift + z z	|	wie :x ( dasselbe wie :wq, aber :wq überschreibt auch wenn keine modification [nur wichtig, falls modification times matter] )
-|-
1. ctrl + z     |        pause and switch to terminal
2. in terminal: fg |   go back to vim
-|-
ctrl + g | show current file name
1 + ctrl + g | show current file name + path

## Mehrere Zeilen auskommentieren: 
[how-to-comment-and-uncomment-multiple-line-vi-terminal-editor](https://discuss.devopscube.com/t/how-to-comment-and-uncomment-multiple-line-vi-terminal-editor/64)

| command | description |
| :---: | :---: |
1. ctrl + v		|	Block markieren
2. Shift + i	|		enter Insert mode (while in Block mode)
3. “		|		Kommentarsymbol am Anfang der ersten Zeile eingeben (while in Block mode)
4. Esc		|		drücken und 1 sec warten (bis das Kommentarsymbol vor allen Zeilen im Block auftaucht)
-|-
|:terminal|
-|-
|:syntax on|
-|-
|:set number|
-|-
|:filetype indent plugin on |
-|-
|:so $VIMRUNTIME/syntax/python.vim| aktiviere Python Syntax
-|-
|:h |				Hilfe (in Hilfe Cursor auf 	Tag zB \|syntax.txt\| platzieren ctrl + alt + 6 und ctrl + alt + 6 um auf content zu springen)
-|-
|/irgend_ein_wort|	suche irgend_ein_wort vorwärts (springt zum ersten solchen Wort, drücke n für nächstes Wort und N für previous occurrence)
-|-
|?irgend_ein_wort|	suche irgend_ein_wort rückwärts
-|-
|:e /path/to/other/file	|öffne anderen file in vim
-|-
|:!g++ -g -Wall % |	compile current file (% steht für current file)
|:!./a.out	|		(after :!g++ -g -Wall %) execute compiled file a.out
-|-
|:%s/pattern/replace/g	|	find pattern (regexp) and replace with replace (Achtung: Sonderzeichen (eg. Klammern, Punkt, …) muss ein \ vorangestellt werden!)


## Navigation

| command | description |
| :---: | :---: |
|h j k l |			links hoch runter rechts
-|-
|line number + shift + g|	spring zu Zeile line number
|0	|			spring zu Zeilenanfang
|$	|			spring zu Zeilenende
|b	|			spring zu Wortanfang
|e	|			spring zu Wortende
|ctrl + d	|		spring 1/2 window nach unten
|ctrl + u	|		spring 1/2 window nach oben
-|-
|v	|			markieren
|Shift + v	|		Zeile markieren
|x	|			cut
|p		|		paste
-|-
|o	|			insert new line below
-|-
|d w|				delete (=cut) to the start of next word
|d i w		|		delete (=cut) current word
|5 d w		|	delete (=cut) next 5 words
|d d	|			delete (=cut) current line
|d %	|			delete (=cut) betw matching brackets {}, [], ()
|d $	|			delete (=cut) to end of line
-|-
|y w	|			yank to the start of next word
|y i w		|		yank current word
|y y	|			yank current line
|y %		|		yank to the matching character (useful to copy text betw matching brackets {}, [], () )
-|-
|&gt;&gt;		|		indent (in Insert mode: ctrl + t)
|&lt;&lt;		|		unindent (in Insert mode: ctrl + d)
|10&lt;&lt;	|		unindent 10 lines
|&lt;%		|		unindent betw matching brackets {}, [], ()
-|-
|u oder :u	|		undo last change
|ctrl + r	|		redo
-|-			
|shift + g | 		Jump to end of file
|g + g |			Jump to first line of file
-|-
|vi -o /path/to/file1 /path/to/file2	|	öffne 2 files in split screen
-|-
|ctrl + w, s	|	öffne neuen split window horizontal
|ctrl + w, v	|	öffne neuen split window vertical (oder besser: `:Lexplore`)
|ctrl + w, &lt;h j k l&gt;|	change active viewport
|ctrl + r		|	rotate viewport (zum Anordnen der viewports)
|ctrl + R|
|ctrl + w, q	|	schließe aktiven split window
|ctrl + w, =	|	resize viewports to be of equal size
|ctrl + w, &lt;	|	decrease active viewport size (für 8 Einheiten Verkleinerung: ctrl + w, 8, &lt;)
|ctrl + 	|		zoom in (anschließend ctrl + w, =)
|ctrl -		|	zoom out (anschließend ctrl + w, =)

# In INSERT mode

| command | description |
| :---: | :---: |
Esc | gehe in command mode

## Avoid the escape key: 
[vim.fandom.com](https://vim.fandom.com/wiki/Avoid_the_escape_key#Avoiding_the_Esc_key)

| command | description |
| :---: | :---: |
ctrl + c | gehe in command mode **<span style="color:red">Achtung:</span>** abbreviations funktionieren hier nicht im Ggs. zu dem "Esc" command mode
alt + normal mode command | führt "normal mode command" im INSERT mode aus
ctrl + o + (cmd)	|	switch to command mode for one command (gut für zB ctrl + o + $ oder andere Jump-commands) **<span style="color:red">Note:</span>** cmd nur bei Mac

## Schreiben

| command | description |
| :---: | :---: |
ctrl + x und danach ctrl + o	|	Omnicompletion (navigiere in Dropdown hoch/runter mit ctrl + p/ctrl + n)
ctrl + p	|		completion with previous matching pattern
ctrl + n 	|		completion with next matching pattern
-|-
ctrl + t		|	indent
ctrl + d		|	unindent
-|-
ctrl + r dann % | insert current file name

# General facts:

## Reasons for swap files (.swp)
Swap files store changes you've made to the buffer. If Vim or your computer crashes, they allow you to recover those changes.
Swap files also provide a way to avoid multiple instances of Vim from editing the same file. This can be useful on multi-user systems or just to know if you have another Vim already editing a file.
[s. stackexcange question](https://vi.stackexchange.com/questions/177/what-is-the-purpose-of-swap-files)
