#!/usr/bin/env python3
"""Flight test: take off to --height meters, hold for --hold seconds, then land.

Custom altitude control (PID on the bottom ToF) in PX4 OFFBOARD mode: a level
attitude + thrust is published on /mavros/setpoint_raw/attitude and PX4
stabilizes the attitude with its own PIDs. There is no horizontal control (no
visual odometry), so the drone can drift in x/y.

Run it INSIDE the drone's ros2-mavros container (see run_hover.sh).

Safety:
  - Ctrl-C / SIGHUP / SIGTERM -> controlled descent and disarm.
  - Aborts (descends) if: no ToF data for > 0.3 s, height > height + 0.6 m,
    tilt > 35°, it leaves OFFBOARD or gets disarmed.
  - If this script dies, PX4 detects the OFFBOARD loss (COM_OF_LOSS_T) and
    runs its failsafe.
  - The dashboard KILL keeps working at all times.
"""
import argparse
import csv
import sys
import math
import signal
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import AttitudeTarget, RCOut, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import BatteryState, Imu, Range


RATE_HZ = 50.0
STEP_M = 0.03       # jump between ToF readings (~30 Hz) treated as a step in the floor
                    # (in normal hover 95 % of the changes between readings are < 1.4 cm)
STEP_MAX_OFFSET = 0.30  # maximum accumulated compensation [m]
CRASH_TILT_DEG = 45.0   # crash/flip tilt: immediate forced disarm in any phase
LAND_TILT_DEG = 25.0    # near the floor (descend/disarm), tilt that means it is tipping over
ACCEL_REF = 0.5     # [m/s^2] reference deceleration when approaching the target height
TOF_TOPIC = "/duckiedrone01/bottom_tof_driver_node/range"


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(w=cr * cp * cy + sr * sp * sy, x=sr * cp * cy - cr * sp * sy,
                      y=cr * sp * cy + sr * cp * sy, z=cr * cp * sy - sr * sp * cy)


def rpy_from_quaternion(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


class HoverTest:
    def __init__(self, args):
        self.a = args
        self.n = rclpy.create_node("dd_hover_test")
        self.state = None
        self.imu = None
        self.rng = None
        self.rng_t = 0.0
        self.vz = 0.0       # vertical velocity from the PX4 EKF (logged only: unreliable with the ToF < 4 cm)
        self.vz_tof = 0.0   # vertical velocity derived from the ToF (filtered): this is what the controller uses
        self._rng_prev = None    # (raw reading, compensated reading, time)
        self.terrain_offset = 0.0  # compensates floor steps (objects under the drone)
        self.step_comp = False     # steps are only compensated in the air (the loop enables it)
        self.terrain_steps = 0
        self.batt = None
        self.motors = []  # motor outputs (MAIN 1-4) as reported by PX4, via /mavros/rc/out
        self.n.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.n.create_subscription(Imu, "/mavros/imu/data", self._on_imu, qos_profile_sensor_data)
        self.n.create_subscription(Range, TOF_TOPIC, self._on_rng, qos_profile_sensor_data)
        self.n.create_subscription(TwistStamped, "/mavros/local_position/velocity_local",
                                   self._on_vel, qos_profile_sensor_data)
        self.n.create_subscription(BatteryState, "/mavros/battery", self._on_batt, qos_profile_sensor_data)
        self.n.create_subscription(RCOut, "/mavros/rc/out", self._on_rcout, qos_profile_sensor_data)
        # Horizontal control with AprilTags (optional)
        self.hc = None
        if args.hold_position:
            # imported here so altitude control still works if numpy is missing
            try:  # as a package (ros2 run/launch) or as a loose file (run_hover.sh copies both to /tmp)
                from .horizontal_control import HorizontalController
            except ImportError:
                from horizontal_control import HorizontalController
            self.hc = HorizontalController(mode=args.hpos_mode, kp=args.hpos_kp, kd=args.hpos_kd,
                                           ki=args.hpos_ki, max_tilt_deg=args.max_tilt)
            self.n.create_subscription(PoseStamped, args.tag_topic, self._on_tag, 10)
        self.pub = self.n.create_publisher(AttitudeTarget, "/mavros/setpoint_raw/attitude", 10)
        self.arm_cli = self.n.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_cli = self.n.create_client(SetMode, "/mavros/set_mode")
        self.cmd_cli = self.n.create_client(CommandLong, "/mavros/cmd/command")
        self.stop_requested = False
        self.integral = 0.0
        self.thrust = 0.0
        self.log = None

    # --- callbacks -------------------------------------------------------------------------
    def _on_state(self, m): self.state = m
    def _on_imu(self, m): self.imu = m

    def _on_tag(self, m):
        if self.imu is None:
            return
        o = m.pose.orientation
        yaw_tags = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        _, _, yaw_px4 = rpy_from_quaternion(self.imu.orientation)
        self.hc.on_tag(time.monotonic(), m.pose.position.x, m.pose.position.y, yaw_tags, yaw_px4)

    def accel_world(self):
        """Acceleration without gravity in the PX4 ENU frame, from the IMU."""
        q, a = self.imu.orientation, self.imu.linear_acceleration
        w, x, y, z = q.w, q.x, q.y, q.z
        R = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
             [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
             [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
        v = (a.x, a.y, a.z)
        return [sum(R[i][j] * v[j] for j in range(3)) for i in range(2)]
    def _on_vel(self, m): self.vz = m.twist.linear.z
    def _on_batt(self, m): self.batt = m
    def _on_rcout(self, m): self.motors = list(m.channels[:4])

    def _on_rng(self, m):
        # Below min_range (4 cm) the VL53 still reads consistently on the floor
        # (e.g. carpet: ~1-2 cm), so it is accepted; only readings above the maximum are dropped.
        if 0.0 <= m.range <= m.max_range:
            now = time.monotonic()
            r = m.range
            if self._rng_prev is not None:
                step = r - self._rng_prev[0]
                # A large jump between two readings with no real acceleration (|a| ~ g) is not the
                # drone moving: it passed over an object or an edge. Compensate instead of reacting.
                if self.step_comp and abs(step) > STEP_M and self.imu is not None:
                    acc = self.imu.linear_acceleration
                    if (abs(math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2) - 9.81) < 2.0
                            and abs(self.terrain_offset - step) <= STEP_MAX_OFFSET):
                        self.terrain_offset -= step
                        self.terrain_steps += 1
                        print(f"Compensated floor step of {step * 100:+.0f} cm "
                              f"(offset {self.terrain_offset * 100:+.0f} cm)")
                dt_r = now - self._rng_prev[2]
                if dt_r > 1e-3:
                    raw = (r + self.terrain_offset - self._rng_prev[1]) / dt_r
                    self.vz_tof += 0.3 * (max(-3.0, min(3.0, raw)) - self.vz_tof)
            self._rng_prev = (r, r + self.terrain_offset, now)
            self.rng = r
            self.rng_t = now

    # --- helpers ---------------------------------------------------------------------------
    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.n, timeout_sec=0.01)

    def call(self, cli, req, timeout=5.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.n, timeout_sec=0.01)
        return fut.result() if fut.done() else None

    def height(self):
        """Height above the takeoff floor: ToF corrected for tilt and floor steps."""
        roll, pitch, _ = rpy_from_quaternion(self.imu.orientation)
        c = math.cos(roll) * math.cos(pitch)
        return (self.rng + self.terrain_offset) * c, roll, pitch

    def height_raw(self):
        """Distance to whatever is directly below (to detect the floor when landing)."""
        roll, pitch, _ = rpy_from_quaternion(self.imu.orientation)
        return self.rng * math.cos(roll) * math.cos(pitch)

    def publish(self, thrust, yaw, roll=0.0, pitch=0.0):
        m = AttitudeTarget()
        m.header.stamp = self.n.get_clock().now().to_msg()
        m.type_mask = (AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE
                       | AttitudeTarget.IGNORE_YAW_RATE)
        m.orientation = quaternion_from_rpy(roll, pitch, yaw)
        m.thrust = float(thrust)
        self.pub.publish(m)

    def pid(self, h_ref, h, dt, airborne, tilt_comp, vz_ref=0.0, integrate=True):
        """vz_ref: desired vertical velocity (climb/descent ramp) for the D term.
        integrate: only integrate while holding height (avoids windup during the climb -> overshoot)."""
        a = self.a
        e = h_ref - h
        if airborne and integrate:
            self.integral = max(-a.i_max, min(a.i_max, self.integral + e * dt))
        u = a.hover + a.kp * e + a.ki * self.integral + a.kd * (vz_ref - self.vz_tof)
        u /= tilt_comp
        u = max(a.thrust_min, min(a.thrust_max, u))
        # limit the thrust change per cycle to avoid jerks
        step = a.thrust_slew / RATE_HZ
        self.thrust = max(self.thrust - step, min(self.thrust + step, u))
        return self.thrust, e

    # --- preflight -------------------------------------------------------------------------
    def preflight(self):
        self.spin_for(2.0)
        problems = []
        if not (self.state and self.state.connected):
            problems.append("MAVROS is not connected to the FC")
        if self.state and self.state.armed:
            problems.append("the drone is already armed")
        if self.imu is None:
            problems.append("no IMU data")
        if self.rng is None or time.monotonic() - self.rng_t > 0.5:
            problems.append(f"no ToF data ({TOF_TOPIC})")
        elif self.rng > 0.2:
            problems.append(f"ToF reads {self.rng:.2f} m: the drone must be on the floor")
        if self.batt is not None and self.batt.percentage == self.batt.percentage:  # no NaN
            pct = self.batt.percentage * 100 if self.batt.percentage <= 1.0 else self.batt.percentage
            print(f"Battery: {self.batt.voltage:.2f} V, {pct:.0f}%")
            if pct < self.a.min_battery:
                problems.append(f"battery {pct:.0f}% < {self.a.min_battery}%")
        # thrust_scaling must be 1.0 (set by the dd-tof-fix service); if it is NaN mavros drops the thrust
        cli = self.n.create_client(GetParameters, "/mavros/setpoint_raw/get_parameters")
        res = self.call(cli, GetParameters.Request(names=["thrust_scaling"]))
        ts = res.values[0].double_value if res and res.values else float("nan")
        if not ts == ts or abs(ts - 1.0) > 1e-6:
            problems.append(f"setpoint_raw/thrust_scaling = {ts} (must be 1.0; is dd-tof-fix running?)")
        self.ground_h = self.rng if self.rng is not None else 0.0
        for p in problems:
            print("PREFLIGHT FAILED:", p)
        return not problems

    # --- main ------------------------------------------------------------------------------
    def run(self):
        a = self.a
        if not self.preflight():
            return 1
        roll0, pitch0, yaw0 = rpy_from_quaternion(self.imu.orientation)
        hover_txt = "auto" if a.hover is None else f"{a.hover:.2f}"
        print(f"Preflight OK. Target {a.height:.2f} m for {a.hold:.0f} s "
              f"(hover={hover_txt}, kp={a.kp}, ki={a.ki}, kd={a.kd})")
        if a.dry_run:
            print("--dry-run: not arming. Showing readings for 5 s...")
            end = time.monotonic() + 5
            while time.monotonic() < end:
                self.spin_for(0.5)
                h, r, p = self.height()
                print(f"  h={h:.3f} m  roll={math.degrees(r):5.1f}°  pitch={math.degrees(p):5.1f}°  vz={self.vz:+.2f}")
            return 0

        self.log_file = open(a.log, "w", newline="", buffering=1)  # line-buffered: the tail is not lost
        self.log = csv.writer(self.log_file)
        self.log.writerow(["t", "phase", "h_ref", "h", "vz", "err", "integral", "thrust", "roll", "pitch", "batt_v", "vz_tof", "terrain_off",
                           "m1_FR", "m2_RL", "m3_FL", "m4_RR"]
                           + (["px", "py", "vx", "vy", "ax_cmd", "ay_cmd", "sigma_x", "sigma_y", "roll_cmd", "pitch_cmd"]
                              if self.hc is not None else []))

        # PX4 requires a setpoint stream before it accepts OFFBOARD
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            self.publish(0.0, yaw0, roll0, pitch0)
            self.spin_for(1.0 / RATE_HZ)

        phase, t_phase, h_ref = "arming", time.monotonic(), 0.0
        mode_req = arm_req = None
        dt = 1.0 / RATE_HZ
        next_tick = time.monotonic()
        t0 = time.monotonic()
        landed_since = None
        h_start = None
        t_ref0 = None
        t_reached = None
        t_lost = None
        t_at_max = None
        force_sent = False
        h_base = 0.0      # height where the takeoff ramp starts
        lift_ticks = 0

        while rclpy.ok():
            next_tick += dt
            now = time.monotonic()
            st = self.state
            h, roll, pitch = self.height()
            h_raw = self.height_raw()
            self.step_comp = phase == "hold" and h_raw > 0.15
            hinfo = {"ok": False}
            roll_c = pitch_c = 0.0
            if self.hc is not None:
                self.hc.predict(self.accel_world(), dt)
            tilt = max(abs(roll), abs(pitch))
            airborne = h > 0.12

            # --- flip/crash: cut the motors now (PX4 would keep spinning up to level it) ---
            crash = tilt > math.radians(CRASH_TILT_DEG) or (
                phase in ("descend", "disarm") and h_raw < 0.4 and tilt > math.radians(LAND_TILT_DEG))
            if crash and st.armed and phase in ("find_hover", "takeoff", "hold", "descend", "disarm") \
                    and not force_sent:
                print(f"FLIP/CRASH: tilt {math.degrees(tilt):.0f}° (h={h_raw:.2f} m) "
                      "-> immediate forced disarm")
                self.cmd_cli.call_async(CommandLong.Request(command=400, param1=0.0, param2=21196.0))
                force_sent = True
                phase, t_phase = "disarm", now

            # --- abort conditions -> descend ---
            if phase in ("find_hover", "takeoff", "hold"):
                reason = None
                if self.stop_requested:
                    reason = "user interrupt"
                elif now - self.rng_t > 0.3:
                    reason = "no ToF data"
                elif h > a.height + 0.6:
                    reason = f"height too high ({h:.2f} m)"
                elif tilt > math.radians(35):
                    reason = f"tilt too high ({math.degrees(tilt):.0f}°)"
                elif phase == "find_hover" and max(abs(roll - roll0), abs(pitch - pitch0)) > math.radians(a.find_max_tilt):
                    reason = (f"tilted before takeoff (Δroll={math.degrees(roll - roll0):+.1f}°, "
                              f"Δpitch={math.degrees(pitch - pitch0):+.1f}°; pitch+ = nose down): "
                              "one side has less thrust; check props/motors/center of gravity")
                elif not st.armed or st.mode != "OFFBOARD":
                    reason = f"PX4 left OFFBOARD/armed (mode={st.mode}, armed={st.armed})"
                if not reason and self.hc is not None and self.hc.ref is None and phase != "find_hover" \
                        and h_raw > 0.4:
                    # On the floor the camera sees the tags at a steep angle and rarely detects them; the
                    # hold point is taken from the first detection in the air. If there is none at 40 cm, land.
                    reason = f"no AprilTags on {a.tag_topic} (is tag_localizer running? are the tags in front of the camera?)"
                if not reason and phase == "hold" and h_raw < self.ground_h + 0.03:
                    print(f"ABORT: dropped to the floor (h={h_raw:.3f} m) -> disarming")
                    phase, t_phase = "disarm", now
                if reason and phase == "find_hover":
                    print("ABORT:", reason, "-> disarming (still on the floor)")
                    phase, t_phase = "disarm", now
                elif reason:
                    print("ABORT:", reason, "-> descending")
                    phase, t_phase = "descend", now
            if phase == "arming" and self.stop_requested:
                print("Interrupted before takeoff")
                break

            # --- state machine ---
            if phase == "arming":
                thrust = 0.0
                if st.mode != "OFFBOARD" and (mode_req is None or mode_req.done()):
                    mode_req = self.mode_cli.call_async(SetMode.Request(custom_mode="OFFBOARD"))
                elif st.mode == "OFFBOARD" and not st.armed and (arm_req is None or arm_req.done()):
                    arm_req = self.arm_cli.call_async(CommandBool.Request(value=True))
                if st.mode == "OFFBOARD" and st.armed:
                    self.thrust = a.thrust_min
                    if a.hover is None:
                        print("OFFBOARD + armed. Searching for takeoff thrust...")
                        phase, t_phase = "find_hover", now
                    else:
                        print("OFFBOARD + armed. Taking off.")
                        phase, t_phase, h_base = "takeoff", now, 0.0
                        h_ref = 0.0
                elif now - t_phase > 5.0:
                    print(f"Could not arm/enter OFFBOARD (mode={st.mode}, armed={st.armed}). "
                          "Check the PX4 warnings.")
                    break
            elif phase == "find_hover":
                # slow thrust ramp until the ToF detects liftoff
                thrust = self.thrust = min(a.find_max, a.thrust_min + a.find_rate * (now - t_phase))
                err = 0.0
                lift_ticks = lift_ticks + 1 if h > self.ground_h + 0.06 else 0
                if lift_ticks >= 3:
                    a.hover = max(a.thrust_min, thrust - 0.02)
                    a.thrust_max = min(0.9, a.hover + 0.2)
                    print(f"Lifted off at thrust {thrust:.3f} -> hover={a.hover:.3f}, "
                          f"thrust_max={a.thrust_max:.3f}. Climbing.")
                    phase, t_phase, h_base = "takeoff", now, h
                    h_ref = h
                elif thrust >= a.find_max:
                    # half a second of grace at the cap: it may be lifting off right there
                    t_at_max = t_at_max or now
                    if now - t_at_max > 0.5:
                        print(f"No liftoff at thrust {a.find_max:.2f} -> disarming. "
                              "Check battery/props or raise --find-max.")
                        phase, t_phase = "disarm", now
            elif phase == "takeoff":
                # reference velocity that brakes at ACCEL_REF before the target height
                vz_ref = min(a.climb_rate, math.sqrt(2 * ACCEL_REF * max(0.0, a.height - h_ref)))
                h_ref = min(a.height, h_ref + vz_ref * dt)
                if a.height - h_ref < 0.003:
                    h_ref, vz_ref = a.height, 0.0
                # No integral while the reference is climbing (avoids overshoot). It turns on at the
                # target height to correct a hover estimate that is too high or too low.
                reached = h_ref >= a.height
                t_reached = (t_reached or now) if reached else None
                thrust, err = self.pid(h_ref, h, dt, airborne, math.cos(roll) * math.cos(pitch),
                                       vz_ref=vz_ref, integrate=reached)
                if reached and (abs(h - a.height) < 0.1 or now - t_reached > 2.0):
                    print(f"Height reached ({h:.2f} m). Holding for {a.hold:.0f} s.")
                    phase, t_phase = "hold", now
                elif now - t_phase > (a.height - h_base) / a.climb_rate + 9.0:
                    print("Did not reach the height in time (hover too low?) -> descending")
                    phase, t_phase = "descend", now
            elif phase == "hold":
                h_ref = a.height
                thrust, err = self.pid(h_ref, h, dt, airborne, math.cos(roll) * math.cos(pitch))
                if now - t_phase >= a.hold:
                    print("Time is up. Landing.")
                    phase, t_phase = "descend", now
            elif phase == "descend":
                if h_start is None:  # the descent starts from wherever it is
                    h_start = max(h_ref, h)
                h_ref = max(0.0, h_start - a.descent_rate * (now - t_phase))
                if h_ref > 0.0:
                    thrust, err = self.pid(h_ref, h, dt, airborne, math.cos(roll) * math.cos(pitch),
                                           vz_ref=-a.descent_rate)
                    t_ref0 = None
                else:
                    # reference already on the floor: ramp the thrust down, no PID
                    t_ref0 = t_ref0 or now
                    thrust = self.thrust = max(a.thrust_min, self.thrust - 0.5 * dt)
                    err = -h
                # on the floor according to the ToF (not the EKF, which diverges with the ToF < 4 cm)
                grounded = h_raw < self.ground_h + 0.03
                landed_since = (landed_since or now) if grounded else None
                if landed_since and now - landed_since > 0.5:
                    print("On the floor (ToF). Disarming.")
                    phase, t_phase = "disarm", now
                elif t_ref0 and now - t_ref0 > 2.0 and h < 0.15:
                    print("Reference has been on the floor for 2 s. Disarming.")
                    phase, t_phase = "disarm", now
            elif phase == "disarm":
                thrust = 0.0
                if st.armed and (arm_req is None or arm_req.done()):
                    arm_req = self.arm_cli.call_async(CommandBool.Request(value=False))
                if st.armed and now - t_phase > 1.5 and not force_sent:
                    # PX4 may refuse to disarm if it thinks it has not landed: force it (we are on the floor)
                    print("PX4 did not disarm; forcing disarm.")
                    self.cmd_cli.call_async(CommandLong.Request(command=400, param1=0.0, param2=21196.0))
                    force_sent = True
                if not st.armed:
                    print("Disarmed. Done.")
                    break
                if now - t_phase > 5.0:
                    print("PX4 did not accept the disarm; use Space/KILL in the dashboard.")
                    break

            if phase in ("arming", "disarm"):
                err = 0.0
            # Hold point: what the tags see while still on the floor (before it moves)
            if self.hc is not None and self.hc.ref is None and phase != "disarm" and self.hc.tags_ok(now):
                (px, py), _ = self.hc.state()
                rx = px if math.isnan(a.hold_x) else a.hold_x
                ry = py if math.isnan(a.hold_y) else a.hold_y
                self.hc.set_reference(rx, ry)
                print(f"Holding position at x={rx:+.2f} y={ry:+.2f} m (tags frame, mode {a.hpos_mode})")
            if phase in ("arming", "find_hover", "disarm"):
                # On the floor: request the same resting tilt. If 0° is requested while the drone rests
                # tilted, PX4 cannot correct it and its integral winds up -> one side lifts.
                self.publish(thrust, yaw0, roll0, pitch0)
            else:
                if self.hc is not None and phase in ("takeoff", "hold", "descend") and h_raw > 0.08:
                    roll_c, pitch_c, hinfo = self.hc.control(now, dt, integrate=(phase == "hold" or (phase == "takeoff" and h_raw > 0.3)))
                    if phase == "hold" and not self.hc.tags_ok(now):
                        t_lost = t_lost or now
                        if now - t_lost > 3.0:
                            print("ABORT: lost the AprilTags for more than 3 s -> descending")
                            phase, t_phase = "descend", now
                    else:
                        t_lost = None
                # test bias (simulates bad leveling/center of gravity; 0 in real flight)
                self.publish(thrust, yaw0, roll_c + math.radians(a.test_roll_bias),
                             pitch_c + math.radians(a.test_pitch_bias))
            self.log.writerow([f"{now - t0:.3f}", phase, f"{h_ref:.3f}", f"{h:.3f}", f"{self.vz:.3f}",
                               f"{err:.3f}", f"{self.integral:.3f}", f"{thrust:.3f}",
                               f"{math.degrees(roll):.1f}", f"{math.degrees(pitch):.1f}",
                               f"{self.batt.voltage:.2f}" if self.batt else "", f"{self.vz_tof:.3f}", f"{self.terrain_offset:.3f}"]
                              + (self.motors + [""] * 4)[:4]
                              + ([f"{hinfo.get(k, float('nan')):.3f}" for k in ("px", "py", "vx", "vy", "ax", "ay", "sx", "sy")]
                                 + [f"{math.degrees(roll_c):.2f}", f"{math.degrees(pitch_c):.2f}"]
                                 if self.hc is not None else []))
            while time.monotonic() < next_tick:
                rclpy.spin_once(self.n, timeout_sec=max(0.0, next_tick - time.monotonic()))
        self.log_file.close()
        if self.terrain_steps:
            print(f"Floor steps compensated during the flight: {self.terrain_steps}")
        print("Log:", a.log)
        return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--height", type=float, default=1.0, help="target height [m] (max 1.5)")
    p.add_argument("--hold", type=float, default=20.0, help="time in the air [s]")
    p.add_argument("--hover", type=float, default=None,
                   help="normalized hover thrust 0-1; measured at takeoff by default")
    p.add_argument("--find-rate", type=float, default=0.12,
                   help="[1/s] thrust ramp while searching for hover")
    p.add_argument("--find-max-tilt", type=float, default=4.0,
                   help="[°] maximum tilt change from rest while searching for hover")
    p.add_argument("--find-max", type=float, default=0.7,
                   help="maximum thrust while searching for hover before giving up")
    p.add_argument("--kp", type=float, default=0.25)
    p.add_argument("--ki", type=float, default=0.15)
    p.add_argument("--kd", type=float, default=0.10)
    p.add_argument("--i-max", type=float, default=2.0, help="integrator limit [m·s]")
    p.add_argument("--thrust-min", type=float, default=0.05)
    p.add_argument("--thrust-max", type=float, default=None, help="default: hover + 0.2")
    p.add_argument("--thrust-slew", type=float, default=1.0, help="max thrust change per second")
    p.add_argument("--climb-rate", type=float, default=0.3, help="[m/s]")
    p.add_argument("--descent-rate", type=float, default=0.25, help="[m/s]")
    p.add_argument("--min-battery", type=float, default=50.0, help="[%%] minimum to take off")
    p.add_argument("--log", default="/tmp/hover_test.csv")
    p.add_argument("--dry-run", action="store_true", help="only check sensors, do not arm")
    # horizontal control (position hold with AprilTags)
    p.add_argument("--hold-position", action="store_true", help="hold position using the AprilTags")
    p.add_argument("--hpos-mode", default="l1", choices=["pd", "l1"], help="pd: baseline PID; l1: PD + adaptive L1")
    p.add_argument("--hpos-kp", type=float, default=1.5, help="[1/s^2] position -> acceleration")
    p.add_argument("--hpos-kd", type=float, default=2.0, help="[1/s] velocity -> acceleration")
    p.add_argument("--hpos-ki", type=float, default=0.5, help="integral (pd mode only)")
    p.add_argument("--max-tilt", type=float, default=10.0, help="[°] maximum tilt of the horizontal controller")
    p.add_argument("--hold-x", type=float, default=float("nan"), help="[m] hold point (nan: takeoff spot)")
    p.add_argument("--hold-y", type=float, default=float("nan"))
    p.add_argument("--tag-topic", default="/dd24/tag_pose")
    p.add_argument("--test-pitch-bias", type=float, default=0.0, help="[°] simulation only: pitch bias to test the controller")
    p.add_argument("--test-roll-bias", type=float, default=0.0, help="[°] simulation only: roll bias")
    # ros2 run / ros2 launch add --ros-args ...: strip them before argparse
    a = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    t = HoverTest(a)
    # Every option is also a ROS parameter (e.g. from config/hover.yaml in a launch file).
    # The ROS parameter wins over the argparse default. -1.0 means "no value".
    for k, v in vars(a).items():
        if isinstance(v, bool):
            pv = t.n.declare_parameter(k, v).value
        elif isinstance(v, str):
            pv = t.n.declare_parameter(k, v).value
        else:
            pv = t.n.declare_parameter(k, -1.0 if v is None else float(v)).value
            pv = None if pv == -1.0 else pv
        setattr(a, k, pv)
    if not 0.2 <= a.height <= 1.5:
        t.n.get_logger().error("height must be between 0.2 and 1.5 m")
        return 2
    if a.thrust_max is None:
        a.thrust_max = min(0.9, (a.hover if a.hover is not None else a.find_max) + 0.2)

    def stop(*_):
        t.stop_requested = True
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, stop)
    try:
        return t.run()
    finally:
        if getattr(t, "log_file", None) and not t.log_file.closed:
            t.log_file.close()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
