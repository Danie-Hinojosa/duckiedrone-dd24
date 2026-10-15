# Duckiedrone DD24-B

ROS 2 (Jazzy) workspace with what we use to build, configure and fly our Duckiedrone DD24-B:
the drone model for RViz, our own altitude controller, the fix we had to apply to the
Duckietown software, the flight controller firmware and parameters, and the log of each
flight.

The drone runs the Duckietown `ente` image with PX4 1.15.4 on a Diatone Mamba F405 MK2, and
MAVROS on the Raspberry Pi.

## What is in each folder

```
src/duckiedrone_dd24/   ROS 2 package: URDF, meshes, launch files, RViz and nodes
tools/dd-tof-fix/       service installed on the drone (see below)
docs/                   write-ups: offboard-arming-fix.md (Duckietown PID exercise does not arm),
                        dts-vs-our-approach.md (Duckietown dts workflow vs. ours)
tools/flight/           run_hover.sh, runs the hover test from the laptop
tools/drone_ros2_env.sh sets up a terminal to talk to the drone's ROS 2
firmware/px4/           PX4 bootloader, firmware and parameters that we flashed
firmware/betaflight/    Betaflight 4.3.2 and the Duckietown configuration (only for flashing the ESCs)
logs/flights/           one CSV per test flight on the real drone
logs/sim/               one CSV per test flight in simulation
```

`local/` is not pushed to the repository. It holds the microSD image, QGroundControl and
Betaflight Configurator, which are large and can be downloaded again.

## Build

```bash
cd ~/duckiedrone_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

To connect to the real drone you also need:

```bash
sudo apt install ros-jazzy-rmw-zenoh-cpp ros-jazzy-mavros-msgs
```

## View the model without the drone

```bash
ros2 launch duckiedrone_dd24 display.launch.py
```

The URDF (`src/duckiedrone_dd24/urdf/dd24.urdf`) uses the STL meshes that Duckietown publishes
in `dt-duckiebot-interface`. Each motor sits at the center of a prop guard ring, at (±91, ±91) mm,
which comes from fitting a circle to the rings on the plates; the propellers have a radius of
~63 mm and the rings are 65 mm on the inside. The layout of everything else (legs under each
motor, Raspberry Pi on top, battery underneath, camera at the front) comes from photos of the
drone. We measured the plate spacing (35 mm) and the leg length (47 mm, from the bottom plate to
the floor) on the drone; with those, the model's ToF ends up 4.35 cm above the floor, the same
value the real sensor reads on a hard floor. The mass and the heights of the electronics are
still estimates.

To check the model without opening RViz: `python3 tools/urdf_preview.py view.png` produces a
top view and a side view.

The axes follow the ROS convention: x toward the camera, y to the left, z up. The motors follow
the PX4 numbering:

| Motor | Position | Rotation |
|---|---|---|
| 1 | front, right | counterclockwise |
| 2 | rear, left | counterclockwise |
| 3 | front, left | clockwise |
| 4 | rear, right | clockwise |

## View the real drone in RViz

The drone uses `rmw_zenoh_cpp` on domain 42. The laptop connects directly to the drone's Zenoh
router, so both have to be on a network where they can reach each other (some guest networks
isolate devices; a hotspot or your own router works).

```bash
source tools/drone_ros2_env.sh      # in every terminal that will talk to the drone
ros2 launch duckiedrone_dd24 drone_rviz.launch.py
```

`drone_state_bridge` turns the PX4 EKF pose into the `map -> base_link` transform, publishes
the bottom ToF on `/dd24/tof_bottom` with its frame, and spins the model's propellers according
to the output for each motor. Without visual odometry the EKF x/y position is not reliable;
altitude and orientation are.

## Simulation (on the laptop)

The simulation runs on the laptop, not on the Raspberry Pi: PX4 SITL 1.15.4 (the same version
as the drone) with Gazebo Harmonic and MAVROS. The `sim/models/dd24` model has the URDF geometry,
1.76 kg and a single-ray downward lidar that stands in for the ToF. Its reading is published on
the same topic as the real driver (`/duckiedrone01/bottom_tof_driver_node/range`), so the
control nodes run the same way in simulation and on the drone.

Once:

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-mavros ros-jazzy-mavros-extras
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
tools/sim/build_px4_sitl.sh        # downloads and builds PX4 into local/ (~10 min)
```

Every time:

```bash
ros2 launch duckiedrone_dd24 sim.launch.py             # gui:=false for no window, rviz:=true for RViz
ros2 run duckiedrone_dd24 hover_test --height 0.4 --hold 5   # in another terminal
```

Simulation results (logs in `logs/sim/`): at 1 m for 20 s the mean altitude was 1.006 m
(1.000 m over the last 10 s) and the mean thrust 0.525, very close to the 0.52-0.55 the real
drone uses. On takeoff it overshoots by about 20 cm, because in simulation the drone takes off
with more thrust than it needs to hover (0.59 vs 0.525) and the estimated hover thrust comes out
high; the integral term corrects it within a few seconds.

Differences from the real drone: in simulation PX4 uses a simulated barometer and GPS to
estimate position (the drone has no GPS or visual odometry), and the ToF does not reach the PX4
EKF because the PX4 1.15 Gazebo bridge does not read distance sensors. The motor constants are
estimated so that hover sits near 55 % of maximum speed.

## AprilTags for position hold

`tools/apriltags/` has print-ready PDFs (`apriltags_tag36h11_A4_160mm.pdf` and the A3 version
with 250 mm tags) and the script that generates them. The first page explains how to print them
and how to lay them out on the floor: a grid with 40 cm spacing around the takeoff point.
With the drone's camera (160° fisheye at 480 px), a 16 cm tag at 1.3 m covers about 20 px,
which is the limit for reliable detection; if you can print on A3, do.

## Hover test

`hover_test` climbs to a target altitude, holds it and lands, using an altitude PID on the ToF
in OFFBOARD mode (PX4 stabilizes the attitude). It measures the takeoff thrust on its own,
compensates for steps in the floor and cuts the motors if the drone flips over. It runs on the
drone, inside the MAVROS container:

```bash
tools/flight/run_hover.sh --height 1.0 --hold 20
```

The flight CSV is copied to `logs/flights/`. The default parameters are in
`src/duckiedrone_dd24/config/hover.yaml`, which `hover.launch.py` also uses.

Before flying: propellers checked, battery charged (4S, 16.8 V), floor clear and matte, and the
Duckietown dashboard open with the KILL button within reach. Without `--hold-position` there is
no horizontal control and the drone drifts.

## Position hold

With `--hold-position`, `hover_test` also controls x/y using the AprilTags. `tag_localizer`
publishes the pose on `/dd24/tag_pose`, a per-axis Kalman filter fuses it with the IMU
acceleration, and a PD computes the required acceleration, which is converted into roll/pitch
for PX4 (10° max). On top of the PD there is an adaptive L1 augmentation (`--hpos-mode l1`, the
default) that estimates the acceleration pushing the drone to one side (offset center of
gravity, badly calibrated level) and subtracts it. The other mode, `pd`, uses an integral term
instead.

```bash
ros2 launch duckiedrone_dd24 sim.launch.py gui:=false
ros2 run duckiedrone_dd24 hover_test --height 1.0 --hold 20 --hold-position
```

The hold point is the first position the tags give once the drone is in the air: from the
ground the camera sees them at too shallow an angle to detect them. If no tag is in view yet at
40 cm, the drone comes back down, and it does the same if it loses them for more than 3 s while
at the target altitude.

In simulation (1 m, 20 s, logs in `logs/sim/`) it was tested with a 3° pitch bias and a -2° roll
bias (`--test-pitch-bias 3 --test-roll-bias -2`), which has the same effect as the tilt that
pushed the real drone forward:

| Test | Max. displacement | RMS at altitude | Landed at |
|---|---|---|---|
| No control, with bias | over 1 m (lost the tags) | | |
| PD + integral, with bias | 30 cm | 11 cm | 3 cm |
| PD + L1, with bias | 11 cm | 3 cm | 1 cm |
| PD + L1, no bias | 9 cm | 3 cm | 2 cm |

The PD alone is slow to compensate for the bias because its integral is slow; L1 estimates it
in about a second. On the real drone the camera image still has to be sorted out before this
can be tested.

## The `dd-tof-fix` workaround

In the `duckietown/dt-ros2-interface:ente-arm64v8` image from September 23, 2026, the MAVROS
plugin parameters in `px4_config.yaml` are not applied. Because of that the ToF never reaches
PX4, PX4 has no altitude, and if the battery failsafe triggers it ends up cutting the motors in
the air. It also leaves `setpoint_raw/thrust_scaling = nan`, so MAVROS discards the thrust in
OFFBOARD.

`tools/dd-tof-fix` is a systemd service for the Pi that reapplies those parameters and relays
the ToF every time the MAVROS container starts. Install it with:

```bash
tools/dd-tof-fix/install.sh
```

and remove it (on the drone) with:

```bash
sudo systemctl disable --now dd-tof-fix && sudo rm -rf /opt/dd-tof-fix /etc/systemd/system/dd-tof-fix.service
```

## PX4 parameters we changed

We started from Duckietown's `firmware/px4/duckiedrone-px4-v4.params` and changed:

| Parameter | Before | After | Why |
|---|---|---|---|
| `MC_BAT_SCALE_EN` | 0 | 1 | scales thrust with battery voltage |
| `BAT1_V_LOAD_DROP` | 0.1 | 0.5 | with no current sensor, compensates for voltage sag when estimating charge |

The full copy of all 779 parameters is in `firmware/px4/px4_params_2026-10-08.tsv`.

## To do

- Reduce the takeoff overshoot in simulation (hover thrust estimate).
- Horizontal control (visual odometry or optical flow with the camera).
- Measure the drone to correct the estimated mass and positions in the URDF.
- Report the `dt-ros2-interface` bug to Duckietown.

## Credits and licenses

The STL meshes in `src/duckiedrone_dd24/meshes/` (DD24 plates, legs, mounts and propellers)
belong to Duckietown and come from their
[dt-duckiebot-interface](https://github.com/duckietown/dt-duckiebot-interface/tree/ente/packages/STL)
repository. They are covered by the Duckietown license
([LICENSE.pdf](https://github.com/duckietown/dt-duckiebot-interface/blob/ente/LICENSE.pdf)),
not by this repository's license. The same applies to `firmware/px4/duckiedrone-px4-v4.params`
and `firmware/betaflight/MAMBAF405MK2V2.conf`, which are configurations published by Duckietown.

The firmware in `firmware/px4/` is PX4 (BSD 3-Clause license) built by Duckietown
(`dd24-mamba-f405-mk2-v1.15.4-1`); the `omnibusf4sd_bl` bootloader is the one PX4 publishes.
Betaflight (`firmware/betaflight/betaflight_4.3.2_STM32F405.hex`) is licensed under GPL-3.0.

The AprilTag images in `tools/apriltags/img/` (tag36h11 family) are the official ones from
[AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs) (BSD-2 license).

The rest of the code (the `duckiedrone_dd24` package, `tools/`) is ours.
