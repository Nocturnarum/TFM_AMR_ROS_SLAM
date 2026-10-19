#!/bin/bash
# Lanza roscore (si hace falta) + main.launch + gmapping.launch
source "$(dirname "$(readlink -f "$0")")/comun.sh"
lanzar_modo gmapping.launch
