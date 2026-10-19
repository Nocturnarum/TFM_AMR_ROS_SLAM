#!/bin/bash
# Lanza roscore (si hace falta) + main.launch + manual.launch
source "$(dirname "$(readlink -f "$0")")/comun.sh"
lanzar_modo manual.launch
