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

# Basics

| command | description |
| :---: | :---: |
git clone --recurse-submodules repo \<Ziel directory\> | 
git reflog |	view history of checkout operations
git log	|	view history of commits
git revert \<commit-hash\>	| commit \<commit-hash\> rückgängig machen
git tag -l	|	list all tags
git checkout tags/\<tag name\>	|		checkout a specific tag
git fetch	|
git checkout solution/2\_foundations	|
git submodule init |
git submodule update |
git config --get remote.origin.url | get only the URL of the current remote
git remote show [remote-name] command | get more details about a particular remote
git remote show origin | get more details about the current remote

# Checkout another branch after modifying files in current branch

When files modified wrt how you checked them out, you will not be able to `git checkout` another branch. 
Therefore, either discard via

```bash
git reset —hard	   # reset all tracked files to original states
git clean -f -d	   # delete all untracked/new files and dirs
```

or keep via

```bash
git status        # see which files are modified
git add -A        # add all new files
git add file1 file2	# add specific files (hier: file1, file2)
git status 		# should show added files in green (green = files are added)
git commit -m “message for saving my solution to exercise 2” #	commit added files to your local “exercise/2_foundations”
```

# Compare code files

```bash
git diff origin/exercise/4_ros_node_cpp..origin/solution/4_ros_node_cpp
```

# credential helper

| command | description |
| :---: | :---: |
git config credential.helper store | store next entered pw in ~/.git-credentials (visible for anyone !) (for current repo only)
git config --global credential.helper store | store next entered pw in ~/.git-credentials (visible for anyone !) (globally, ie. for all repos)

## Reset credential helper

Unset credential helper

`git config --global --unset credential.helper`

and set pw again as usual or just use

`git config credential.helper store`

and you will be prompted to enter pw again.

# Create a new repository on the command line

```bash
echo "# documentation" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/pharath/documentation.git
git push -u origin main
```

# Push an existing repository from the command line

```bash
git remote add origin https://github.com/pharath/documentation.git
git branch -M main
git push -u origin main
```

