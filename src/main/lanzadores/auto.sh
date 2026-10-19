#!/bin/bash
# Lanza roscore (si hace falta) + main.launch + navigation.launch
source "$(dirname "$(readlink -f "$0")")/comun.sh"
lanzar_modo navigation.launch
