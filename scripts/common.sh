#!/bin/bash
# Created by Nelson Durrant, Jan 2026
# 
# Common script functions

function printInfo {
    echo -e "\033[0m\033[36m[ 🐟 ] $1\033[0m"
}

function printWarning {
    echo -e "\033[0m\033[33m[ 🐡 ] $1\033[0m"
}

function printError {
    echo -e "\033[0m\033[31m[ 🐙 ] $1\033[0m"
}
