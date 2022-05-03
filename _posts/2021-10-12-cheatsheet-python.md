---
title: "Python Cheatsheet"
read_time: false
excerpt_separator: "<!--more-->"
categories:
  - Cheatsheet
  - Python
tags:
  - python
  - conda
  - cheatsheet
toc: true
toc_label: "Contents"

---

# Installing multiple Python Versions

- [source](https://askubuntu.com/a/682875):
    - `sudo add-apt-repository ppa:deadsnakes/ppa`
    - `sudo apt-get update`
    - `sudo apt-get install python3.5`
    - `sudo apt-get install python3.5-dev`
        - It will not overwrite your existing `python3.4` which is still symlinked as `python3`.
        - Instead, to run `python3.5`, run the command `python3.5` (or `python3.X` for any other version of python).

## venv using multiple Python Versions

- install `sudo apt install python3.7-venv` or `sudo apt install python3.x-venv` for **each python version**!

# Uninstalling multiple Python Versions

- `sudo apt purge python3.6 libpython3.6-minimal libpython3.6-stdlib python3.6-minimal`

# python packages and modules

- [source](https://realpython.com/absolute-vs-relative-python-imports/)
- `.py` files are **modules**
- folders (containing modules) are **packages**
    - importing a package essentially imports the package's `__init__.py` file as a module

# pyenv

- For Python version management, e.g. 
	- if you want to use multiple python versions on the same machine
	- if a project requires an older python version
- `pipenv install` (see below) will automatically install a python version using `pyenv`, if the project requires it 

## Prerequisites

```bash
sudo apt-get update; sudo apt-get install make build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm \
libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev
```

## Install

```bash
curl https://pyenv.run | bash
```

# pip

- use `pipenv` (see subsection below) instead of `venv` and `pip`
- pip caches downloaded packages, so that you can resume downloads and installations by running `pip install` again, if e.g. 
    - the installation is aborted because there was no space left on the device

| command | description |
| :---: | :---: |
pip install pip-autoremove | utility to remove a package plus unused dependencies
pip show *package* | show location of *package*
pip-autoremove *package* | remove a package plus unused dependencies (install `pip install pip-autoremove` first)

# venv (python3)

- **Warning:** Do not move or copy the `env` folder to other locations! Always re-create environments by using `python3 -m venv env` and re-installing all packages from a `requirements.txt` file. 

| command | description |
| :---: | :---: |
python3 -m venv env | create the environment `env`
source env/bin/activate | activate the environment `env`
deactivate | deactivate the environment that is currently activated
rm -r env/ | delete the environment `env`

# virtualenv (python2)

| command | description |
| :---: | :---: |
virtualenv -p /home/username/opt/python-2.7.15/bin/python venv | create a virtualenv with name "venv"

- the rest is similar to venv

# requirements.txt files

| command | description |
| :---: | :---: |
pip install -r requirements.txt | install all packages from a `requirements.txt` file
pip freeze > requirements.txt | write all packages in the current environment to a `requirements.txt` file

# pipenv

- **Warning**: not much development here, i.e. maybe official support ends soon? Use `poetry` instead !

| command | description |
| :---: | :---: |
pip install --user pipenv | Python's officially recommended packaging tool
pipenv install | install right python version, environment and all dependencies
pipenv lock --clear | if `ERROR: No matching distribution found for markupsafe==1.0 ERROR: Couldn't install package: MarkupSafe Package installation failed...` (and after this command run `pipenv install` again)
pipenv | shows help 
pipenv shell | activates env (similar to `source /env/bin/activate` for `pip`)
pipenv --rm | remove the virtualenv created under /home/bra-ket/.local/share/virtualenvs
pipenv run python blockchain.py | Spawns a command installed into the virtualenv.
pipenv graph | shows installed dependencies

# poetry

| command | description |
| :---: | :---: |
curl -sSL https://install.python-poetry.org \| python3 - | install poetry (check the installation with `poetry` or `poetry --version`)

# conda

Vorinstallierte modules: [List](https://docs.anaconda.com/anaconda/packages/py3.8_linux-64/)

| command | description |
| :---: | :---: |
conda config --set auto_activate_base False | conda base environment nicht mit shell starten
conda env list |
conda info --envs |
conda create --name myenv |
conda env create -f environment.yml | 
conda activate myenv |
conda install --file requirements.txt |
conda deactivate |
conda remove --name myenv --all |

# repl

## Start flags

| command | description |
| :---: | :---: |
python3 -v | zeige Details der ausgeführten Befehle (eg. automatische imports, andere getriggerte Befehle, Konstruktor calls, etc.)

# jupyter

## Convert notebooks to other formats

| command | description |
| :---: | :---: |
jupyter nbconvert --to html notebook.ipynb | pass --execute flag, if cells should be run before converting
jupyter nbconvert --to pdf notebook.ipynb |

# package source code location

| command | description |
| :---: | :---: |
pip show torch | show the location of package "torch" (there you can find the source code of the package)
