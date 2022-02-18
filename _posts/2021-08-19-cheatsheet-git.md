---
title: "Git Cheatsheet"
read_time: false
excerpt: "Some essential git commands"
header:
  teaser: /assets/images/Vim.jpg
  overlay_image: /assets/images/Vim.jpg
  overlay_filter: 0.5 
toc: true
toc_sticky: true
categories:
  - Cheatsheet
tags:
  - git
  - cheatsheet
---

# Terminology

**remote** = remote-repository (e.g. in `git push *remote* *branch*`)

# Basics

| command | description |
| :---: | :---: |
git clone --recurse-submodules repo \<Ziel directory\> | 
git branch -a | show/list all branches (local and remote)
git branch -r | show/list all branches (only remote)
git show-branch -a | show/list all branches **and commits** (local and remote)
git show-branch -r | show/list all branches **and commits** (only remote)
git checkout \<existing\_branch\> | switch to an existing branch (or: git switch *branch*)
git checkout -b \<new\_branch\> | switch to a non-existing branch (or: git switch -c *branch*)
git reflog |	view history of checkout operations
git log	|	view history of commits
git revert \<commit-hash\>	| commit \<commit-hash\> rückgängig machen
git tag -l	|	list all tags
git -c 'versionsort.suffix=-' ls-remote --tags --sort='v:refname' \<repository\> | list all tags of \<repository\>
git checkout tags/\<tag name\>	|		checkout a specific tag
git fetch	|
git checkout solution/2\_foundations	|
git submodule init |
git submodule update |
git config --get remote.origin.url | get only the URL of the current remote
git remote show [remote-name] command | get more details about a particular remote
git remote show origin | get more details about the current remote
git remote update origin --prune | To update the local list of remote branches

# Reset/undo changes

| command | description |
| :---: | :---: |
git reflog | get SHA-1 list of previous states
git reset --hard *SHA-1* | reset to a previous state (**Warning**: All changes will be lost.)

# Checkout another branch after modifying files in current branch

When files modified wrt how you checked them out, you will not be able to `git checkout` another branch. 
Therefore, either discard via

```bash
git reset --hard   # reset all tracked files to original states
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

# git diff

## Compare code files

```bash
git diff origin/exercise/4_ros_node_cpp..origin/solution/4_ros_node_cpp
```

## Compare commits

```bash
# Syntax: git diff COMMIT~ COMMIT (vergleicht COMMIT mit dessen ancestor)
git diff 2326473e602be4b90b46f6b6afc7315ff1d09a17~ 2326473e602be4b90b46f6b6afc7315ff1d09a17
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

# submodules

## populate a repository's submodules

1. `git submodule init`
2. `git submodule update`

# Troubleshooting/Errors

```bash
 ! [remote rejected] origin/parking -> origin/parking (deny updating a hidden ref)
error: failed to push some refs to 'https://git.rwth-aachen.de/team_galaxis/carolo-cup-2021.git'
```

**Solution**: The branch name is wrong. Use `git push origin parking` instead of `git push origin origin/parking`.
