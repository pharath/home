---
title: "Bash Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - bash 
  - cheatsheet
---

# bash scripting

## Shebang

In computing, a shebang is the character sequence consisting of the characters number sign and exclamation mark (#!) at the beginning of a script.

When a text file with a shebang is used as if it is an executable in a Unix-like operating system, the program loader mechanism parses the rest of the file's initial line as an interpreter directive. The loader executes the specified interpreter program, passing to it as an argument the path that was initially used when attempting to run the script, so that the program may use the file as input data.[8] For example, if a script is named with the path path/to/script, and it starts with the following line, #!/bin/sh, then the program loader is instructed to run the program /bin/sh, passing path/to/script as the first argument. In Linux, this behavior is the result of both kernel and user-space code.[9] 
The shebang line is usually ignored by the interpreter, because the "#" character is a comment marker in many scripting languages; some language interpreters that do not use the hash mark to begin comments still may ignore the shebang line in recognition of its purpose.[10] 

## Was bedeutet $# in einem bash script? (s. [askubuntu post](https://askubuntu.com/questions/939620/what-does-mean-in-bash))

$#
number of arguments (wie argc in C)

## Vergleichsoperatoren

if [[ $# -gt 0 ]]; then
else
fi

“-gt” für “greater than” (ie der “>” operator) in der condition
“-eq” für “equal to” (ie der “=“ operator)
