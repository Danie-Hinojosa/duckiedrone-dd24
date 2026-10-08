# Duckiedrone DD24-B

Workspace de ROS 2 (Jazzy) con lo que usamos para armar, configurar y volar nuestro
Duckiedrone DD24-B: el modelo del dron para RViz, un control de altura propio, el arreglo
que tuvimos que hacerle al software de Duckietown, el firmware y los parámetros del
controlador de vuelo, y los registros de cada vuelo.

El dron corre la imagen `ente` de Duckietown con PX4 1.15.4 en una Diatone Mamba F405 MK2 y
MAVROS en la Raspberry Pi.

## Qué hay en cada carpeta

```
src/duckiedrone_dd24/   paquete de ROS 2: URDF, mallas, launch, RViz y nodos
tools/dd-tof-fix/       servicio que se instala en el dron (ver abajo)
tools/flight/           run_hover.sh, para correr la prueba de hover desde la laptop
tools/dron_ros2_env.sh  prepara una terminal para hablar con el ROS 2 del dron
firmware/px4/           bootloader, firmware y parámetros de PX4 que flasheamos
firmware/betaflight/    Betaflight 4.3.2 y la configuración de Duckietown (solo para flashear los ESC)
logs/flights/           un CSV por vuelo de prueba en el dron real
logs/sim/               un CSV por vuelo de prueba en la simulación
```

`local/` no se sube al repositorio: ahí quedan la imagen de la microSD, QGroundControl y
Betaflight Configurator, que pesan mucho y se pueden volver a descargar.

## Compilar

```bash
cd ~/duckiedrone_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Para conectarse al dron real hacen falta además:

```bash
sudo apt install ros-jazzy-rmw-zenoh-cpp ros-jazzy-mavros-msgs
```

## Ver el modelo sin el dron

```bash
ros2 launch duckiedrone_dd24 display.launch.py
```

El URDF (`src/duckiedrone_dd24/urdf/dd24.urdf`) usa las mallas STL que publica Duckietown en
`dt-duckiebot-interface`. Cada motor va al centro de un aro protector, en (±91, ±91) mm, que
sale de ajustar un círculo a los aros de las placas; las hélices miden ~63 mm de radio y los
aros 65 mm por dentro. La disposición del resto (patas bajo cada motor, Raspberry Pi arriba,
batería abajo, cámara al frente) sale de fotos del dron. Medimos en el dron la separación
entre placas (35 mm) y el largo de las patas (47 mm, de la placa inferior al piso); con eso
el ToF del modelo queda a 4.35 cm del piso, lo mismo que marca el sensor real en piso firme.
La masa y las alturas de la electrónica siguen estimadas.

Para revisar el modelo sin abrir RViz: `python3 tools/urdf_preview.py vista.png` genera una
vista superior y una lateral.

Los ejes son los de ROS: x hacia la cámara, y a la izquierda, z arriba. Los motores siguen la
numeración de PX4:

| Motor | Posición | Giro |
|---|---|---|
| 1 | adelante, derecha | antihorario |
| 2 | atrás, izquierda | antihorario |
| 3 | adelante, izquierda | horario |
| 4 | atrás, derecha | horario |

## Ver el dron real en RViz

El dron usa `rmw_zenoh_cpp` en el dominio 42. La laptop se conecta directo al router de
Zenoh del dron, así que los dos tienen que estar en una red donde se vean entre sí (algunas
redes de invitados aíslan a los dispositivos; un hotspot o un router propio funcionan).

```bash
source tools/dron_ros2_env.sh      # en cada terminal que vaya a hablar con el dron
ros2 launch duckiedrone_dd24 drone_rviz.launch.py
```

`drone_state_bridge` convierte la pose del EKF de PX4 en la transformada `map -> base_link`,
publica el ToF inferior en `/dd24/tof_bottom` con su frame y hace girar las hélices del modelo
según la salida a cada motor. Sin odometría visual, la posición en x/y del EKF no es confiable;
la altura y la orientación sí.

## Simulación (en la laptop)

La simulación corre en la laptop, no en la Raspberry Pi: PX4 SITL 1.15.4 (la misma versión
del dron) con Gazebo Harmonic y MAVROS. El modelo `sim/models/dd24` tiene la geometría del URDF,
1.76 kg y un lidar de un rayo hacia abajo que hace de ToF. Su lectura se publica en el mismo
tópico que el driver real (`/duckiedrone01/bottom_tof_driver_node/range`), así que los nodos de
control corren igual en la simulación y en el dron.

Una sola vez:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-mavros ros-jazzy-mavros-extras
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
tools/sim/build_px4_sitl.sh        # descarga y compila PX4 en local/ (~10 min)
```

Cada vez:

```bash
ros2 launch duckiedrone_dd24 sim.launch.py             # gui:=false sin ventana, rviz:=true con RViz
ros2 run duckiedrone_dd24 hover_test --height 0.4 --hold 5   # en otra terminal
```

Resultados en la simulación (logs en `logs/sim/`): con 1 m durante 20 s la altura media fue
1.006 m (1.000 m en los últimos 10 s) y el empuje medio 0.525, muy cerca del 0.52-0.55 que
usa el dron real. Al despegar se pasa unos 20 cm porque en la simulación el dron despega con
más empuje del que necesita para sostenerse (0.59 contra 0.525) y el hover estimado sale alto;
la integral lo corrige en unos segundos.

Diferencias con el dron real: en la simulación PX4 usa barómetro y GPS simulados para estimar
la posición (en el dron no hay GPS ni odometría visual), y el ToF no le llega al EKF de PX4
porque el puente de Gazebo de PX4 1.15 no lee sensores de distancia. Las constantes de los
motores son estimadas para que el hover quede cerca del 55 % de la velocidad máxima.

## Prueba de hover

`hover_test` sube a una altura, se mantiene y aterriza, con un PID de altura sobre el ToF en
modo OFFBOARD (PX4 estabiliza la orientación). Mide solo el empuje de despegue, compensa
escalones del piso y corta los motores si el dron se voltea. Corre en el dron, dentro del
contenedor de MAVROS:

```bash
tools/flight/run_hover.sh --height 1.0 --hold 20
```

El CSV del vuelo se copia a `logs/flights/`. Los parámetros por defecto están en
`src/duckiedrone_dd24/config/hover.yaml`, que también usa `hover.launch.py`.

Antes de volar: hélices revisadas, batería cargada (4S, 16.8 V), piso despejado y mate, y el
panel de Duckietown abierto con el botón KILL a la mano. No hay control horizontal todavía,
así que el dron deriva.

## El arreglo `dd-tof-fix`

En la imagen `duckietown/dt-ros2-interface:ente-arm64v8` del 23 de septiembre de 2026, los
parámetros de los plugins de MAVROS que trae `px4_config.yaml` no se aplican. Por eso el ToF
nunca llega a PX4, PX4 se queda sin altura, y si salta el failsafe de batería termina
cortando los motores en el aire. También queda `setpoint_raw/thrust_scaling = nan`, con lo que
MAVROS descarta el empuje en OFFBOARD.

`tools/dd-tof-fix` es un servicio de systemd para la Pi que vuelve a aplicar esos parámetros y
reenvía el ToF cada vez que el contenedor de MAVROS arranca. Se instala con:

```bash
tools/dd-tof-fix/install.sh
```

y se quita (en el dron) con:

```bash
sudo systemctl disable --now dd-tof-fix && sudo rm -rf /opt/dd-tof-fix /etc/systemd/system/dd-tof-fix.service
```

## Parámetros de PX4 que cambiamos

Partimos de `firmware/px4/duckiedrone-px4-v4.params` de Duckietown y cambiamos:

| Parámetro | Antes | Ahora | Por qué |
|---|---|---|---|
| `MC_BAT_SCALE_EN` | 0 | 1 | escala el empuje con el voltaje de la batería |
| `BAT1_V_LOAD_DROP` | 0.1 | 0.5 | sin sensor de corriente, compensa la caída de voltaje al estimar la carga |

La copia completa de los 779 parámetros está en `firmware/px4/px4_params_2026-10-08.tsv`.

## Pendiente

- Reducir el sobrepaso al despegar en la simulación (estimación de hover).
- Control horizontal (odometría visual o flujo óptico con la cámara).
- Medir el dron para corregir masa y posiciones estimadas del URDF.
- Reportar el error de `dt-ros2-interface` a Duckietown.

## Créditos y licencias

Las mallas STL de `src/duckiedrone_dd24/meshes/` (placas, patas, soportes y hélices del DD24)
son de Duckietown y vienen de su repositorio
[dt-duckiebot-interface](https://github.com/duckietown/dt-duckiebot-interface/tree/ente/packages/STL).
Están sujetas a la licencia de Duckietown
([LICENSE.pdf](https://github.com/duckietown/dt-duckiebot-interface/blob/ente/LICENSE.pdf)),
no a la de este repositorio. Lo mismo aplica a `firmware/px4/duckiedrone-px4-v4.params` y
`firmware/betaflight/MAMBAF405MK2V2.conf`, que son configuraciones publicadas por Duckietown.

El firmware de `firmware/px4/` es PX4 (licencia BSD 3-Clause) compilado por Duckietown
(`dd24-mamba-f405-mk2-v1.15.4-1`); el bootloader `omnibusf4sd_bl` es el que publica PX4.
Betaflight (`firmware/betaflight/betaflight_4.3.2_STM32F405.hex`) tiene licencia GPL-3.0.

El resto del código (paquete `duckiedrone_dd24`, `tools/`) es nuestro.
