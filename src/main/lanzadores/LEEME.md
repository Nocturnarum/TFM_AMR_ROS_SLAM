# Lanzadores de escritorio (Jetson, Ubuntu 18.04, ROS Melodic, usuario bryan)

Con doble clic arrancan o paran solos lo necesario. Los modos no preguntan nada; Ensayo solo pide los datos del ensayo.

| Lanzador | Script | Qué hace |
|---|---|---|
| Compilar | compilar.sh | `cd ~/catkin_ws && catkin_make`. La terminal se cierra sola si va bien y queda abierta si hay error |
| Mapear | mapear.sh | Para los `roslaunch main …` en marcha y arranca roscore (si falta), `main.launch` y `gmapping.launch` |
| Manual | manual.sh | Igual, con `manual.launch` |
| Auto | auto.sh | Igual, con `navigation.launch` |
| Auto-adaptativo | auto_adaptativo.sh | Igual, con `adaptativo.launch` (archivo nuevo de Bryan; aún NO está en catkin_ws) |
| Ensayo | ensayo.sh | Pregunta escenario, modo y repetición, prepara ROS, graba, pide la hoja de campo y calcula las métricas (ver abajo) |
| Parar todo | parar_todo.sh | `rosnode kill -a; killall roslaunch rosmaster roscore` |

`comun.sh` lo comparten todos:
- Hace `source` de `/opt/ros/melodic/setup.bash` y `~/catkin_ws/devel/setup.bash`.
- Comprueba roscore con `rostopic list`. Si falta, lo abre en su propia ventana y espera hasta 30 s. Si ya corre, no lo toca.
- Para con SIGINT, y si no responde con SIGTERM, todos los `roslaunch main …` en marcha: main, el modo anterior y una grabación.
- Abre una ventana con pestañas `main` y modo; el modo arranca 5 s después de main.
- Cada pestaña termina en `exec bash`, así que queda abierta si algo falla.

## Ensayo (ensayo.sh)
Usa lo que existe en catkin_ws, sin modificarlo: `main/launch/record_trial.launch` y `main/src/metricas_ensayo.cpp` (`rosrun main metricas_ensayo`). Hace lo mismo que `bot_ws/scripts/procesar_ensayos.sh`, pero con un solo bag.

1. Pregunta escenario (1=S1 amplia, 2=S2 estrecha, 3=S3 obstáculo, 4=S4 reconfigurada), modo (`tele`/`fijo`/`adap`) y repetición (1-5).
2. Nombre `S{n}_{modo}_{rep:02d}`. Si ya hay un `~/ensayos/<nombre>_20*.bag`, avisa y pregunta si repetirlo. Si se responde `s`, se REEMPLAZAN los datos antes de grabar:
   - Los bags antiguos se mueven, sin borrarlos, a `~/ensayos/descartados/`. `procesar_ensayos.sh` solo lee `~/ensayos/*.bag`, así que no los vuelve a procesar.
   - En `resultados_amcl.csv`, `resultados_odom.csv` y `hoja_campo.csv` se guarda antes una copia `.bak` (solo la del último reemplazo).
   - Luego se quitan con `awk` las filas cuya columna `bag` empieza por `<nombre>_20`; la cabecera se mantiene. Con el prefijo `_20` no se confunden, por ejemplo, `S1_tele_01` y `S1_tele_010`.
   - Se usa awk porque está siempre en Ubuntu 18.04; no depende de la versión de python.
3. Prepara ROS: tele = `manual.launch`, fijo = `navigation.launch`, adap = `adaptativo.launch`.
   - Si roscore, main y ese modo ya corren (y nada más), los deja.
   - Si no, para lo que haya y arranca lo que falte (espera 15 s).
   - Después lanza `roslaunch main record_trial.launch nombre:=<nombre>`, que hace `rosbag record -o ~/ensayos/<nombre>` de /odom, /amcl_pose, /tf, /scan, /cmd_vel*, /mode_manual, /move_base/*, /perfil_navegacion…
   - Enter → para la grabación con SIGINT y espera a que se cierre el `.bag.active`.
4. Pide la distancia a la marca (m), dtheta (rad), la batería (texto del LED) y observaciones, y añade una fila a `~/ensayos/hoja_campo.csv` (`bag,d_medida_m,dtheta_medida_rad,bateria,obs,fecha`). Las comas del texto se eliminan.
5. Métricas:
   - `metricas_ensayo <bag> --tol 0.15 0.26 --csv ~/ensayos/resultados_amcl.csv` (pose por defecto `/amcl_pose`)
   - `metricas_ensayo <bag> --tol 0.15 0.26 --pose /odom --csv ~/ensayos/resultados_odom.csv`
   - Los CSV se van ampliando con una fila por tramo. `procesar_ensayos.sh` los borra y los regenera con todos los bags.

Opciones verificadas en `metricas_ensayo.cpp`: `--pose`, `--tol XY YAW`, `--csv` y `--real X Y YAW`; todas existen. No existen el paquete `tfm_tools` ni `manual_trial.launch`. Lo equivalente está en el paquete `main` (`record_trial.launch`, `metricas_ensayo`), y para teleoperado se usa `manual.launch`.

Orden de los 60 ensayos: `lista_ensayos.md`.

## Instalación (sin sudo)
Los lanzadores viven en `~/catkin_ws/src/main/lanzadores/`. Los .desktop apuntan a `/home/bryan/catkin_ws/src/main/lanzadores/X.sh` y los .sh encuentran `comun.sh` en su propia carpeta.
```bash
# 1. Descomprimir (el zip tiene la raíz lanzadores/)
unzip -o lanzadores.zip -d ~/catkin_ws/src/main/
# 2. Scripts ejecutables
chmod +x ~/catkin_ws/src/main/lanzadores/*.sh
# 3. Accesos directos al escritorio (~/Desktop o ~/Escritorio)
DESK=$(xdg-user-dir DESKTOP)
cp ~/catkin_ws/src/main/lanzadores/*.desktop "$DESK"/
chmod +x "$DESK"/*.desktop
```
Si Ubuntu muestra "Lanzador no confiable", clic derecho en cada icono → **Permitir lanzar**.

La carpeta no afecta a catkin: no tiene `package.xml` ni `CMakeLists.txt`, y el `CMakeLists.txt` de `main` no la referencia, así que `catkin_make` la ignora.

## Notas
- Nombres verificados en `catkin_ws/src/main/launch`: `gmapping.launch`, `manual.launch`, `navigation.launch`, `record_trial.launch`. `adaptativo.launch` no existe todavía.
- **Auto ya es adaptativo:** `navigation.launch` ya lanza `perfil_adaptativo`, así que "Auto"/"fijo" también cambia de perfil. Para un "fijo" real habría que quitarlo de `navigation.launch` (decisión de Bryan).
- **Pestañas viejas:** al cambiar de modo, las pestañas del modo anterior quedan abiertas con un bash (el roslaunch ya parado); se pueden cerrar a mano.
- **Detección de procesos:** se buscan los procesos `bin/roslaunch main …` (no el `bash -c` de las pestañas).
- **Ensayo sin terminal:** si se abre con doble clic, ensayo.sh se reabre solo en gnome-terminal.
- **Opción `-e` de gnome-terminal:** está obsoleta pero funciona en 18.04 (gnome-terminal 3.28); puede salir un aviso.
