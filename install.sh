#!/bin/bash

[ -d ~/.DL-autorun ] && rm -rf ~/.DL-autorun

cd ..
if [ -d DL-autorun ]; then
    cp -r DL-autorun ~/.DL-autorun
    cd
else
    cd
    git clone https://github.com/Venquieu/DL-autorun.git .DL-autorun || {
        echo "git clone failed!"
        exit 1
    }
fi

alias_info="alias monitor='python ~/.DL-autorun/runner.py'"
if [ -f .bashrc ]; then
    echo >> .bashrc
    echo "# DL_autorun setting" >> .bashrc
    echo  $alias_info >> .bashrc
fi
if [ -f .zshrc ]; then
    echo >> .zshrc
    echo "# DL_autorun setting" >> .zshrc
    echo  $alias_info >> .zshrc
fi

echo "Install finished!"
echo "Run monitor --help for usage"
