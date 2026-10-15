# Duckietown `dts` workflow vs. how we fly the drone

There are two ways we have run altitude control on the DD24. One is the official Duckietown
exercise ([`lx-dd-altitude-pid-control`](https://github.com/duckietown/lx-dd-altitude-pid-control)),
built and run with `dts`. The other is our own `hover_test.py`. Both run the controller on the
drone, next to MAVROS, so neither depends on Wi-Fi during the flight. They differ in how the
code gets to the drone and in how much the node does besides the PID.

This comparison is based on version `0e30f00` of the exercise (September 22, 2026) and the
`ente` images available in October 2026.

| | Duckietown `dts` (`lx-dd-altitude-pid-control`) | Ours (`hover_test.py`) |
|---|---|---|
| How the code reaches the drone | `dts code build` builds a Docker image on the drone and `dts code workbench` runs it in its own container | `tools/flight/run_hover.sh` copies the script into the `ros2-mavros` container and runs it there |
| Time to test a change | Rebuild the image. The first build took 6 min 15 s on our Pi; later builds use the cache | Seconds: copy and run |
| Control topic | `setpoint_attitude` (`PoseStamped` + `Thrust`), broken by the image bug (see [offboard-arming-fix.md](offboard-arming-fix.md)) | `setpoint_raw/attitude` (`AttitudeTarget`) at 50 Hz |
| Arming | As soon as the node starts, and it retries every 5 s | Once, after the preflight checks pass |
| Takeoff and landing | None. The PID is switched on with a service; switching it off drops to 0.1 thrust | Finds the hover thrust, ramps up, holds, ramps down and disarms when the ToF reads the floor |
| Checks before arming | Only that MAVROS is connected | Connection, IMU, ToF, drone on the floor, battery, `thrust_scaling` |
| In-flight aborts | Turns the PID off if the ToF is lost for more than 1 s | Lost ToF, too high, tilt, leaving OFFBOARD, flip detection, Ctrl-C |
| Logging | Only what it prints to the terminal | A CSV row per control cycle |
| Simulation | Duckietown virtual drone (`dts duckiebot virtual`) | PX4 SITL + Gazebo on the laptop |
| Tuning gains | Live, with `ros2 param set` | Command-line arguments at launch |

## Duckietown `dts`

Advantages:

- It is the official workflow. Anyone at Duckietown will recognise every command, and the
  exercise notebooks walk through the PID theory step by step.
- Code, dependencies and ROS version are packaged in one image. If it works on one drone it
  should work the same on another with the same base image.
- The code lives in its own container and does not touch `ros2-mavros` or any other
  Duckietown container.
- Changing gains while it flies, without restarting the node, is very convenient for tuning.
- `thrust_cap` (0.55 by default) puts a ceiling on the thrust while people are learning.

Disadvantages:

- It does not work today without fixes, because of the `use_quaternion` bug. Even with the
  topic fixed, without `dd-tof-fix` the thrust on `setpoint_raw` arrives as `nan` and PX4 has no
  altitude.
- Every code change means rebuilding the image on the Pi. The first build downloaded Ubuntu
  packages, so the drone needs internet access.
- The node only covers holding altitude. It does not ramp the takeoff or land. Switching the
  PID off with the same service drops the thrust to 0.1, so if the drone is in the air it
  falls. At the default 0.25 m that is not a big deal; higher up it is.
- It arms as soon as it starts, and it arms again every 5 s if the drone is disarmed while
  the node is still running. Disarming to "stop" without killing the node does not work: it
  re-arms by itself.
- It does not log the flight, so there is no way to look back at what happened.

## Our approach

Advantages:

- It already works on the real drone: 1 m for 20 s, mean height 1.002 m
  (`logs/flights/`).
- Iterating is fast: edit the script and it is on the drone a few seconds later.
- The script runs the whole flight from start to end, with checks and aborts that each came
  from a real problem: the flip on landing, integrator windup on the floor, the ToF below
  4 cm, the battery failsafe.
- The CSV logs let us find bugs after each flight and compare against the simulation.
- We have our own simulation with the same PX4 (1.15.4), where we tested horizontal control
  before trying it on the drone.

Disadvantages:

- It depends on `dd-tof-fix`, which is our own workaround. If Duckietown changes the image
  (fixes the bug, renames topics), the workaround may stop working or get in the way, and has
  to be checked.
- The script is copied to `/tmp` in a container that is not ours. If `ros2-mavros` restarts,
  it is gone and has to be copied again (`run_hover.sh` does that). It also uses whatever
  Python that container has. We have not checked whether it has numpy, which horizontal
  control needs.
- It is not the Duckietown workflow. If we ask them for help, we first have to explain
  everything we changed.
- It is a test script that runs once and exits, not a node that keeps running. Things like
  teleoperation or visual odometry will need it split into separate nodes.
- Everything is in one large file. It works, but it is harder for someone else to follow.

## Problems that can hit both

- Image updates. A `dts duckiebot update` can change MAVROS. With `dts` you notice when you
  build; with our approach, `dd-tof-fix` has to be checked after every update.
- Ctrl-C and disarming. With `dts code workbench`, stopping it with Ctrl-C removed the
  container before the node's shutdown disarm ran, and the drone stayed armed in STABILIZED
  for about 16 s. Our script does disarm on Ctrl-C, but if the SSH session drops or the
  container dies, PX4's own failsafe is what acts (`COM_OF_LOSS_T` = 1 s). Either way, keep
  the dashboard KILL open.
- Battery. PX4 estimates the charge from voltage only. With Duckietown's
  `BAT1_V_LOAD_DROP = 0.1` the battery failsafe triggered a few seconds into our flights; we
  use 0.5.
- Gains do not carry over between drones without retuning. The hover thrust depends on each
  drone's mass and battery.

## What we would do

For what we are doing (our own controller, quick tests, horizontal control), we will keep
our approach while developing. Once something is stable, it is worth packaging it as a `dts`
project: it would run in its own container without copying files into `ros2-mavros`, and
other teams could run it with commands they already know.

For a course that follows the exercise, the exercise is still the right tool, with the patch
and `dd-tof-fix` from [offboard-arming-fix.md](offboard-arming-fix.md), and keeping in mind
that the node does not land on its own.
