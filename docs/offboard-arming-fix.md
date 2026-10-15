# "Arming failed. Check pre-arm checks." with `lx-dd-altitude-pid-control`

This page is for anyone running the Duckietown altitude PID exercise
([`duckietown/lx-dd-altitude-pid-control`](https://github.com/duckietown/lx-dd-altitude-pid-control))
on a real DD24 with the `ente` software, where the node switches PX4 to OFFBOARD and then
cannot arm. I reproduced the problem on our own drone, found the cause, and confirmed the fix
with the propellers removed. The steps below should work on any DD24 with the same image.

## What you see

After `dts code build -R <robot>` and `dts code workbench -R <robot>`, the node starts and
connects to MAVROS without errors, and then it gets stuck:

```
[altitude_pid_node]: Preflight complete, MAVROS connected — transitioning to OFFBOARD_IDLE.
[altitude_pid_node]: OFFBOARD mode requested successfully.
[altitude_pid_node]: Arming failed. Check pre-arm checks.
[altitude_pid_node]: Arming failed. Check pre-arm checks.
...
```

The "Arming failed" line repeats every 5 s. `/mavros/state` shows `mode: OFFBOARD` and
`armed: false`. Running `dts duckiebot update` does not change anything. The image on the
Duckietown registry was still the same one when I checked on October 15, 2026.

## Why it happens

The exercise node sends the attitude as a `geometry_msgs/PoseStamped` on
`/mavros/setpoint_attitude/attitude` and the thrust on `/mavros/setpoint_attitude/thrust`.
The MAVROS `setpoint_attitude` plugin only listens to `PoseStamped` when its parameter
`use_quaternion` is `true`. When it is `false`, it waits for a `TwistStamped` on
`/mavros/setpoint_attitude/cmd_vel` instead.

Duckietown's `px4_config.yaml` sets `use_quaternion: true` for this reason, but in the image
`duckietown/dt-ros2-interface:ente-arm64v8` built on September 23, 2026 the parameters under
`/**/<plugin>` in that file never reach the MAVROS plugin nodes. So `use_quaternion` stays
`false`, nobody subscribes to `/mavros/setpoint_attitude/attitude`, and MAVROS never sends a
single setpoint to PX4. You can check it yourself:

```bash
ros2 topic info /mavros/setpoint_attitude/attitude       # "Unknown topic": nobody listens
ros2 param get /mavros/setpoint_attitude use_quaternion  # Boolean value is: False
```

PX4 lets you select OFFBOARD while the drone is disarmed, so the mode request "works". When
the node then asks to arm, PX4 checks whether OFFBOARD has a live setpoint stream, finds none,
and refuses.

PX4 says this in its events, but MAVROS only prints them as numbers in the `ros2-mavros`
log. These are the ones that matter here:

| Event ID | PX4 event name | Meaning |
|---|---|---|
| 3087815 | `check_modes_offboard_signal` | No OFFBOARD setpoints are arriving |
| 133277 | `commander_arm_denied_resolve_failures` | Arming denied because of the check above |

To see them on the drone:

```bash
docker logs -f --since 1m ros2-mavros 2>&1 | grep "FCU: EVENT"
```

The IDs are the PX4 event name hashed with 32-bit FNV-1a and masked to 24 bits, so you can
decode any other one from the PX4 v1.15 source. You will also see `check_modes_local_pos`,
`check_modes_global_pos`, `check_modes_mission` and `check_modes_home_position`. Those only
say that the position-based modes are unavailable without GPS or a position estimate. They do
not block arming in OFFBOARD.

The same bug breaks two more things that you will hit right after this one:

- `/mavros/setpoint_raw/thrust_scaling` stays `nan`, and MAVROS drops the thrust of every
  `setpoint_raw/attitude` message.
- The `distance_sensor` plugin has no configuration, so the bottom ToF is never forwarded to
  PX4. The DD24 parameters set `SYS_HAS_BARO = 0` and use the ToF as the height reference
  (`EKF2_HGT_REF = 2`), so PX4 has no altitude at all. If a failsafe triggers in
  the air (low battery, for example), PX4 cannot run Land or Descend without altitude and
  falls back to flight termination, which cuts the motors. This happened to us on one of our
  first flights.

## The fix

There are two parts, and you need both.

### Part 1: install `dd-tof-fix` on the drone

`tools/dd-tof-fix` in this repository is a small systemd service for the Raspberry Pi. Every
10 s it checks the `ros2-mavros` container. When the container is new or was restarted, it:

1. applies the `/**/<plugin>` parameters from Duckietown's own `px4_config.yaml` to the
   MAVROS plugin nodes, which fixes `thrust_scaling` and the `distance_sensor` configuration;
2. starts a relay from `/<robot>/bottom_tof_driver_node/range` to `/mavros/bottom_tof`, so the
   ToF reaches PX4.

It does not modify any Duckietown image or file, and it takes the robot name from the
drone's hostname.

It cannot fix `use_quaternion`. Changing that parameter at runtime makes MAVROS recreate the
subscription, and with `rmw_zenoh` that fails with "Invalid history policy". That is why
Part 2 is needed.

From your laptop:

```bash
git clone https://github.com/Danie-Hinojosa/duckiedrone-dd24.git
cd duckiedrone-dd24
tools/dd-tof-fix/install.sh duckie@<robot>.local
```

It asks for the drone password for `scp` and `sudo`. At the end it prints the last lines of
the service log, which should include `workaround applied`. If it says
`mavros not ready yet; retrying`, wait 20 s and check again with
`journalctl -u dd-tof-fix -n 5` on the drone.

To check it worked, on the drone:

```bash
docker exec -it ros2-mavros bash -l
source /opt/ros/jazzy/setup.bash
timeout 10 ros2 param get /mavros/setpoint_raw thrust_scaling   # Double value is: 1.0
timeout 10 ros2 topic echo --once /mavros/bottom_tof            # a range of a few mm on the floor
```

The service is enabled, so it comes back on its own after every reboot or container restart
(it takes about 15 s after MAVROS starts). When Duckietown fixes the image you can remove it,
on the drone, with:

```bash
sudo systemctl disable --now dd-tof-fix && sudo rm -rf /opt/dd-tof-fix /etc/systemd/system/dd-tof-fix.service
```

### Part 2: send the attitude through `setpoint_raw`

`/mavros/setpoint_raw/attitude` takes a `mavros_msgs/AttitudeTarget` with the orientation
and the thrust in one message, and it does not depend on `use_quaternion`. The patch in
[`docs/patches/lx-dd-altitude-pid-control-setpoint-raw.patch`](patches/lx-dd-altitude-pid-control-setpoint-raw.patch)
adds a publisher for it to the exercise node. In every control cycle the node publishes the
same level attitude and the same thrust it already computes, so the rest of the exercise
(the PID, the `/enable_altitude_control` service, the joystick override) works the same. The
old `PoseStamped` and `Thrust` messages are still published; they are harmless.

```bash
cd ~/lx-dd-altitude-pid-control
git apply /path/to/duckiedrone-dd24/docs/patches/lx-dd-altitude-pid-control-setpoint-raw.patch
git diff --stat   # pid_controller_node.py, 1 file changed
```

The patch was made against commit `0e30f00` of the exercise (September 22, 2026). If
`git apply` refuses it, your copy is a different version and you can add the lines by hand.
They are three small changes in `packages/pid_controller/pid_controller/pid_controller_node.py`:

```python
# 1. import
from mavros_msgs.msg import State, Thrust, ManualControl, AttitudeTarget

# 2. in __init__, next to the other publishers
self.pub_raw = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

# 3. in _timer_cb, right after self.pub_thrust.publish(thrust_msg)
raw = AttitudeTarget()
raw.header.stamp = thrust_msg.header.stamp
raw.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE
                 | AttitudeTarget.IGNORE_YAW_RATE)
raw.orientation = attitude_msg.pose.orientation
raw.thrust = float(thrust_value)
self.pub_raw.publish(raw)
```

The code is copied into the image at build time, so rebuild after patching:

```bash
dts code build -R <robot>
dts code workbench -R <robot>
```

## First test: no propellers

Take the propellers off for the first run. With the fix you should see:

```
[altitude_pid_node]: OFFBOARD mode requested successfully.
[altitude_pid_node]: Vehicle armed successfully.
[altitude_pid_node]: Vehicle just armed — PID state reset.
```

The motors spin slowly, because the node sends an idle thrust of 0.1 while it waits. The PID
only takes over after `ros2 service call /enable_altitude_control std_srvs/srv/Trigger`.

On our drone the unpatched node failed to arm every time, and the patched node armed on the
first attempt. That was with the same image, the same PX4 parameters and the same gains.

## Be careful when you stop it

When I stopped `dts code workbench` with Ctrl-C, the exercise container was removed before
the node's own shutdown disarm ran. PX4 lost the OFFBOARD stream, switched to STABILIZED, and
the drone stayed armed for about 16 s until I disarmed it by hand. Without propellers nothing
happened, but with propellers on that is dangerous.

After every run, check `/mavros/state`. If it still says `armed: true`, press KILL in the
dashboard or force a disarm from inside `ros2-mavros`:

```bash
ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
  '{command: 400, param1: 0.0, param2: 21196.0}'
```

That is `MAV_CMD_COMPONENT_ARM_DISARM` with the PX4 force code, so it disarms even if PX4
thinks the drone is flying. Keep the dashboard open with KILL within reach for every flight.

## What I have not tested

I tested each part on our drone, which already had `dd-tof-fix` installed. I have not run the
full sequence on a drone that never had it (install the fix from scratch, then fly the
patched exercise). If something in that order fails, the `FCU: EVENT` numbers in the
`ros2-mavros` log are the first thing to look at.
