#!/bin/bash
# Lanza roscore (si hace falta) + main.launch + adaptativo.launch
source "$(dirname "$(readlink -f "$0")")/comun.sh"
lanzar_modo adaptativo.launch
