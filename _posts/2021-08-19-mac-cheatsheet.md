---
title: "Mac Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - mac
  - cheatsheet
---

# Wifi settings

```bash
cd /System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/
./airport en1 prefs # set wifi connection behaviour (eg. DisconnectOnLogout=NO)
```

# ssh tricks

| command | description |
| :---: | :---: |
caffeinate -u | prevent the system from sleeping and (-u for) prevent the system from sleeping [source](https://apple.stackexchange.com/questions/53802/waking-display-from-terminal-general-waking/161527)
