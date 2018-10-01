#!/bin/bash
echo "Installing NITE 1.5.2.23 ..."

function check_lib(){
    sudo ldconfig -v 2>/dev/null | grep $1 2>/dev/null
    echo $?
}

pushd "${HOME}/libs"
if [ check_lib NITE ]; then
    wget "http://www.openni.ru/wp-content/uploads/2013/10/NITE-Bin-Linux-x64-v1.5.2.23.tar.zip" -O nite.zip
    mkdir -p nite
    unzip nite.zip -d nite
    cd nite
    tar xvjf "NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2"
    cd "NITE-Bin-Dev-Linux-x64-v1.5.2.23"
    sudo ./install.sh
fi
popd
