---
title: "Git Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
tags:
  - git
  - cheatsheet
---

| command | description |
| :---: | :---: |
git clone --recurse-submodules repo <Ziel directory> | 
git reflog |	view history of checkout operations
git log	|	view history of commits
git revert commit hash	| commit “commit hash” rückgängig machen
git tag -l	|	list all tags
git checkout tags/<tag name>	|		checkout a specific tag
git fetch	|
git checkout solution/2_foundations	|
git submodule init |
git submodule update |

<hr>

When files modified wrt how you checked them out, you will not be able to “git checkout” another branch. 
Therefore, either discard via

git reset —hard	|  reset all tracked files to original states
git clean -f -d	|	delete all untracked/new files and dirs

or  keep via

git status |	see which files are modified
git add -A |	add all new files
git add file1 file2	|	add specific files (hier: file1, file2)
git status |		should show added files in green (green = files are added)
git commit -m “message for saving my solution to exercise 2” |	commit added files to your local “exercise/2_foundations”

<hr>

git diff origin/exercise/4_ros_node_cpp..origin/solution/4_ros_node_cpp |
