---
title: "Doxygen Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - doxygen
  - cheatsheet
---

# General Experiences/Issues

## Markdown files are merged

**Note:** This happened when I loaded an old `Doxyfile` in order to generate the documentation. After that Ichanged the destination dir for the doc and the working dir from which doxygen runs (maybe, the issue arised because the src dir and working dir were different at this point). Furthermore, `EXTRACT_PRIVATE` in Expert/Build was activated.

**Solution:** If doxygen merges all markdown files into a single file in the output (i.e. only one entry for all markdown files together in the left doxygen documentation pane instead of one entry for each markdown file), then start doxywizard without the `Doxyfile` and configure doxygen from scratch.

# Expert

## Build

- `EXTRACT_PRIVATE` must be checked, if private member functions shall be included in the output.
