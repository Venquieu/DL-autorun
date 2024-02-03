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

if [ -f /usr/bin/python ]; then
    alias_info="alias monitor='/usr/bin/python ~/.DL-autorun/runner.py'"
elif [ -f /usr/bin/python3 ]; then
    alias_info="alias monitor='/usr/bin/python3 ~/.DL-autorun/runner.py'"
else
    python --version && alias_info="alias monitor='python ~/.DL-autorun/runner.py'"
    python3 --version && alias_info="alias monitor='python3 ~/.DL-autorun/runner.py'"
fi

[ -n "$alias_info" ] || {
    echo "not found python, have you installed it?"
    exit 1
}

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
echo "Please close and restart your shell to make it work, then run monitor --help for usage"
