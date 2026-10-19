#!/bin/bash
# Graba y procesa un ensayo: record_trial.launch + metricas_ensayo (sin modificarlos)
# Si se lanza con doble clic (sin terminal) se reabre en gnome-terminal
[ -t 0 ] || exec gnome-terminal --title="Ensayo" -- bash "$(readlink -f "$0")"
source "$(dirname "$(readlink -f "$0")")/comun.sh"
DIR=~/ensayos; mkdir -p "$DIR"
fin() { read -p "Pulsa Enter para cerrar"; exit "${1:-0}"; }

# 1. Escenario, modo y repetición
echo "Escenario: 1=S1 amplia, 2=S2 estrecha, 3=S3 obstáculo, 4=S4 reconfigurada"
until [[ "$ESC" =~ ^[1-4]$ ]]; do read -p "Escenario (1-4): " ESC; done
until [[ "$MODO" =~ ^(tele|fijo|adap)$ ]]; do read -p "Modo (tele/fijo/adap): " MODO; done
until [[ "$REP" =~ ^[1-5]$ ]]; do read -p "Repetición (1-5): " REP; done
case $MODO in tele) LAUNCH=manual.launch;; fijo) LAUNCH=navigation.launch;; adap) LAUNCH=adaptativo.launch;; esac

# 2. Nombre (rosbag -o añade _AAAA-MM-DD-hh-mm-ss.bag)
NOMBRE=$(printf "S%d_%s_%02d" "$ESC" "$MODO" "$REP")
if ls "$DIR/${NOMBRE}"_20*.bag >/dev/null 2>&1; then
  echo "AVISO: ya existe un bag de $NOMBRE:"; ls "$DIR/${NOMBRE}"_20*.bag
  read -p "¿Repetirlo y REEMPLAZAR sus datos? (s/N): " r; [ "$r" = s ] || fin 1
  REEMPLAZAR=1
fi

# Quita del CSV $1 las filas cuya columna bag empiece por "$NOMBRE_20" (deja .bak y la cabecera)
quitar_filas() {
  [ -f "$1" ] || return 0
  cp "$1" "$1.bak"
  awk -F, -v pre="${NOMBRE}_20" 'NR==1 || index($1, pre)!=1' "$1.bak" > "$1.tmp" && mv "$1.tmp" "$1"
  echo "  $(basename "$1"): $(( $(wc -l < "$1.bak") - $(wc -l < "$1") )) fila(s) quitada(s) (copia en $(basename "$1").bak)"
}

# Reemplazo: mueve los bags antiguos a descartados/ (no se borran) y limpia los CSV
reemplazar() {
  mkdir -p "$DIR/descartados"
  echo "Moviendo bags antiguos de $NOMBRE a $DIR/descartados/"
  mv -v "$DIR/${NOMBRE}"_20*.bag "$DIR/descartados/"
  for c in resultados_amcl.csv resultados_odom.csv hoja_campo.csv; do quitar_filas "$DIR/$c"; done
}

# Si se repite: mover bags viejos y limpiar CSV antes de grabar el nuevo
[ -n "$REEMPLAZAR" ] && reemplazar

# 3. Arrancar/parar lo necesario y grabar
preparar_modo "$LAUNCH" || { echo "ERROR: no se pudo preparar ROS"; fin 1; }
echo "Grabando $NOMBRE en $DIR ..."
roslaunch main record_trial.launch nombre:="$NOMBRE" >"/tmp/record_$NOMBRE.log" 2>&1 &
read -p ">>> Pulsa Enter para PARAR la grabación <<<"
pkill -INT -f "bin/roslaunch main record_trial.launch nombre:=$NOMBRE"; wait
for i in $(seq 1 20); do ls "$DIR/${NOMBRE}"_20*.bag.active >/dev/null 2>&1 || break; sleep 1; done
BAG=$(ls -t "$DIR/${NOMBRE}"_20*.bag 2>/dev/null | head -1)
[ -n "$BAG" ] || { echo "ERROR: no se encontró el bag (ver /tmp/record_$NOMBRE.log)"; fin 1; }
echo "Bag: $BAG"

# 4. Hoja de campo
read -p "Distancia medida a la marca (m): " D
read -p "dtheta medido (rad): " DT
read -p "Batería (LED, texto): " BAT
read -p "Observaciones: " OBS
HOJA=$DIR/hoja_campo.csv
[ -f "$HOJA" ] || echo "bag,d_medida_m,dtheta_medida_rad,bateria,obs,fecha" > "$HOJA"
limpia() { echo "$1" | tr -d ',\n'; }   # sin comas para no romper el CSV
echo "$(basename "$BAG"),$(limpia "$D"),$(limpia "$DT"),$(limpia "$BAT"),$(limpia "$OBS"),$(date '+%Y-%m-%d %H:%M:%S')" >> "$HOJA"

# 5. Métricas (mismas opciones que procesar_ensayos.sh)
cd "$DIR"
rosrun main metricas_ensayo "$BAG" --tol 0.15 0.26 --csv "$DIR/resultados_amcl.csv"
rosrun main metricas_ensayo "$BAG" --tol 0.15 0.26 --pose /odom --csv "$DIR/resultados_odom.csv"
echo "Listo: $HOJA, $DIR/resultados_amcl.csv, $DIR/resultados_odom.csv"
fin 0
