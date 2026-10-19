#!/bin/bash
# Nota: se busca "bin/roslaunch main" para no confundir con el bash -c de las pestañas
# Funciones comunes de los lanzadores (ROS Melodic, ~/catkin_ws). Sin preguntas: paran/arrancan lo necesario.
ENTORNO="source /opt/ros/melodic/setup.bash; source $HOME/catkin_ws/devel/setup.bash"
eval "$ENTORNO"

roscore_ok() { timeout 3 rostopic list >/dev/null 2>&1; }

# Arranca roscore en su propia ventana si no responde y espera hasta 30 s
asegurar_roscore() {
  roscore_ok && return 0
  gnome-terminal --title="roscore" -- bash -c "$ENTORNO; roscore; exec bash"
  for i in $(seq 1 30); do roscore_ok && return 0; sleep 1; done
  echo "roscore no responde tras 30 s"; return 1
}

# ¿Está corriendo 'roslaunch main <archivo>'?
corriendo() { pgrep -f "bin/roslaunch main $1" >/dev/null; }

# Para todos los 'roslaunch main ...' (main, modo, grabación) sin tocar roscore
parar_launches() {
  pgrep -f "bin/roslaunch main" >/dev/null || return 0
  pkill -INT -f "bin/roslaunch main"
  for i in $(seq 1 15); do pgrep -f "bin/roslaunch main" >/dev/null || return 0; sleep 1; done
  pkill -TERM -f "bin/roslaunch main"; sleep 2
}

# Abre una ventana con pestañas para main.launch (si falta) y el modo $1
abrir_main_y_modo() {
  local args=()
  corriendo main.launch || args+=(--tab --title="main" -e "bash -c '$ENTORNO; roslaunch main main.launch; exec bash'")
  corriendo "$1" || args+=(--tab --title="$1" -e "bash -c '$ENTORNO; sleep 5; roslaunch main $1; exec bash'")
  [ ${#args[@]} -gt 0 ] && gnome-terminal "${args[@]}"
  return 0
}

# Modo de doble clic: para lo anterior, arranca roscore (si falta), main y el modo
lanzar_modo() {
  parar_launches
  asegurar_roscore || { gnome-terminal -- bash -c "echo 'ERROR: roscore no arranca'; exec bash"; exit 1; }
  abrir_main_y_modo "$1"
}

# Ensayo: si main y el modo ya corren, se dejan; si no, se para lo que haya y se arranca lo que falte
preparar_modo() {
  if corriendo main.launch && corriendo "$1" && ! pgrep -f "bin/roslaunch main record_trial" >/dev/null \
     && [ "$(pgrep -fc 'bin/roslaunch main')" -eq 2 ]; then return 0; fi
  parar_launches
  asegurar_roscore || return 1
  abrir_main_y_modo "$1"
  echo "Esperando a que arranquen main y $1 (15 s)..."; sleep 15
}
