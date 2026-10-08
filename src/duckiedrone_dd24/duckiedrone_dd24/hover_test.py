#!/usr/bin/env python3
"""Prueba de vuelo: despega a --height metros, se mantiene --hold segundos y aterriza.

Control de altura propio (PID sobre el ToF inferior) en modo OFFBOARD de PX4:
se publica una orientación nivelada + empuje en /mavros/setpoint_raw/attitude y
PX4 estabiliza la actitud con sus propios PID. No hay control horizontal (no hay
odometría visual), así que el dron puede derivar en x/y.

Se corre DENTRO del contenedor ros2-mavros del dron (ver run_hover.sh).

Seguridad:
  - Ctrl-C / SIGHUP / SIGTERM -> descenso controlado y desarme.
  - Aborta (desciende) si: ToF sin datos > 0.3 s, altura > height + 0.6 m,
    inclinación > 35°, sale de OFFBOARD o se desarma.
  - Si este script muere, PX4 detecta pérdida de OFFBOARD (COM_OF_LOSS_T) y
    aplica su failsafe.
  - El KILL del dashboard sigue funcionando en todo momento.
"""
import argparse
import csv
import sys
import math
import signal
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped, Quaternion
from mavros_msgs.msg import AttitudeTarget, RCOut, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import BatteryState, Imu, Range

RATE_HZ = 50.0
STEP_M = 0.03       # salto entre lecturas del ToF (~30 Hz) que se considera escalón del piso
                    # (en hover normal el 95 % de los cambios entre lecturas es < 1.4 cm)
STEP_MAX_OFFSET = 0.30  # compensación máxima acumulada [m]
CRASH_TILT_DEG = 45.0   # inclinación de choque/volteo: desarme forzado inmediato en cualquier fase
LAND_TILT_DEG = 25.0    # cerca del piso (descenso/desarme), inclinación que indica que se está volteando
ACCEL_REF = 0.5     # [m/s^2] desaceleración de la referencia al llegar a la altura objetivo
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
        self.vz = 0.0       # velocidad vertical del EKF de PX4 (solo se registra: no es fiable con el ToF < 4 cm)
        self.vz_tof = 0.0   # velocidad vertical derivada del ToF (filtrada): la que usa el control
        self._rng_prev = None    # (lectura cruda, lectura compensada, tiempo)
        self.terrain_offset = 0.0  # compensa escalones del piso (objetos debajo del dron)
        self.step_comp = False     # solo se compensan escalones en el aire (lo activa el lazo)
        self.terrain_steps = 0
        self.batt = None
        self.motors = []  # salidas a motores (MAIN 1-4) según PX4, vía /mavros/rc/out
        self.n.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.n.create_subscription(Imu, "/mavros/imu/data", self._on_imu, qos_profile_sensor_data)
        self.n.create_subscription(Range, TOF_TOPIC, self._on_rng, qos_profile_sensor_data)
        self.n.create_subscription(TwistStamped, "/mavros/local_position/velocity_local",
                                   self._on_vel, qos_profile_sensor_data)
        self.n.create_subscription(BatteryState, "/mavros/battery", self._on_batt, qos_profile_sensor_data)
        self.n.create_subscription(RCOut, "/mavros/rc/out", self._on_rcout, qos_profile_sensor_data)
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
    def _on_vel(self, m): self.vz = m.twist.linear.z
    def _on_batt(self, m): self.batt = m
    def _on_rcout(self, m): self.motors = list(m.channels[:4])

    def _on_rng(self, m):
        # Debajo de min_range (4 cm) el VL53 sigue midiendo de forma consistente en el piso
        # (p. ej. alfombra: ~1-2 cm), así que se acepta; solo se descarta lo que pasa del máximo.
        if 0.0 <= m.range <= m.max_range:
            now = time.monotonic()
            r = m.range
            if self._rng_prev is not None:
                step = r - self._rng_prev[0]
                # Un salto grande entre dos lecturas sin aceleración real (|a| ~ g) no es el dron
                # moviéndose: pasó sobre un objeto o una orilla. Se compensa en vez de reaccionar.
                if self.step_comp and abs(step) > STEP_M and self.imu is not None:
                    acc = self.imu.linear_acceleration
                    if (abs(math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2) - 9.81) < 2.0
                            and abs(self.terrain_offset - step) <= STEP_MAX_OFFSET):
                        self.terrain_offset -= step
                        self.terrain_steps += 1
                        print(f"Escalón del piso de {step * 100:+.0f} cm compensado "
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
        """Altura sobre el piso de despegue: ToF corregido por inclinación y por escalones."""
        roll, pitch, _ = rpy_from_quaternion(self.imu.orientation)
        c = math.cos(roll) * math.cos(pitch)
        return (self.rng + self.terrain_offset) * c, roll, pitch

    def height_raw(self):
        """Distancia a lo que haya justo debajo (para detectar el piso al aterrizar)."""
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
        """vz_ref: velocidad vertical deseada (rampa de subida/bajada) para la parte D.
        integrate: solo se integra al mantener altura (evita windup durante la subida -> sobrepaso)."""
        a = self.a
        e = h_ref - h
        if airborne and integrate:
            self.integral = max(-a.i_max, min(a.i_max, self.integral + e * dt))
        u = a.hover + a.kp * e + a.ki * self.integral + a.kd * (vz_ref - self.vz_tof)
        u /= tilt_comp
        u = max(a.thrust_min, min(a.thrust_max, u))
        # limita el cambio de empuje por ciclo para evitar tirones
        step = a.thrust_slew / RATE_HZ
        self.thrust = max(self.thrust - step, min(self.thrust + step, u))
        return self.thrust, e

    # --- preflight -------------------------------------------------------------------------
    def preflight(self):
        self.spin_for(2.0)
        problems = []
        if not (self.state and self.state.connected):
            problems.append("MAVROS no está conectado al FC")
        if self.state and self.state.armed:
            problems.append("el dron ya está armado")
        if self.imu is None:
            problems.append("sin datos de IMU")
        if self.rng is None or time.monotonic() - self.rng_t > 0.5:
            problems.append(f"sin datos del ToF ({TOF_TOPIC})")
        elif self.rng > 0.2:
            problems.append(f"ToF marca {self.rng:.2f} m: el dron debe estar en el piso")
        if self.batt is not None and self.batt.percentage == self.batt.percentage:  # no NaN
            pct = self.batt.percentage * 100 if self.batt.percentage <= 1.0 else self.batt.percentage
            print(f"Batería: {self.batt.voltage:.2f} V, {pct:.0f}%")
            if pct < self.a.min_battery:
                problems.append(f"batería {pct:.0f}% < {self.a.min_battery}%")
        # thrust_scaling debe ser 1.0 (lo pone el servicio dd-tof-fix); si es NaN mavros descarta el empuje
        cli = self.n.create_client(GetParameters, "/mavros/setpoint_raw/get_parameters")
        res = self.call(cli, GetParameters.Request(names=["thrust_scaling"]))
        ts = res.values[0].double_value if res and res.values else float("nan")
        if not ts == ts or abs(ts - 1.0) > 1e-6:
            problems.append(f"setpoint_raw/thrust_scaling = {ts} (debe ser 1.0; ¿corre dd-tof-fix?)")
        self.ground_h = self.rng if self.rng is not None else 0.0
        for p in problems:
            print("PREFLIGHT FALLA:", p)
        return not problems

    # --- main ------------------------------------------------------------------------------
    def run(self):
        a = self.a
        if not self.preflight():
            return 1
        roll0, pitch0, yaw0 = rpy_from_quaternion(self.imu.orientation)
        hover_txt = "auto" if a.hover is None else f"{a.hover:.2f}"
        print(f"Preflight OK. Objetivo {a.height:.2f} m durante {a.hold:.0f} s "
              f"(hover={hover_txt}, kp={a.kp}, ki={a.ki}, kd={a.kd})")
        if a.dry_run:
            print("--dry-run: no se arma. Mostrando lecturas 5 s...")
            end = time.monotonic() + 5
            while time.monotonic() < end:
                self.spin_for(0.5)
                h, r, p = self.height()
                print(f"  h={h:.3f} m  roll={math.degrees(r):5.1f}°  pitch={math.degrees(p):5.1f}°  vz={self.vz:+.2f}")
            return 0

        self.log_file = open(a.log, "w", newline="", buffering=1)  # line-buffered: no se pierde el final
        self.log = csv.writer(self.log_file)
        self.log.writerow(["t", "phase", "h_ref", "h", "vz", "err", "integral", "thrust", "roll", "pitch", "batt_v", "vz_tof", "terrain_off",
                           "m1_FR", "m2_RL", "m3_FL", "m4_RR"])

        # PX4 exige un flujo de setpoints antes de aceptar OFFBOARD
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
        force_sent = False
        h_base = 0.0      # altura desde la que arranca la rampa de despegue
        lift_ticks = 0

        while rclpy.ok():
            next_tick += dt
            now = time.monotonic()
            st = self.state
            h, roll, pitch = self.height()
            h_raw = self.height_raw()
            self.step_comp = phase == "hold" and h_raw > 0.15
            tilt = max(abs(roll), abs(pitch))
            airborne = h > 0.12

            # --- volteo/choque: cortar motores ya (PX4 seguiría acelerando para enderezarlo) ---
            crash = tilt > math.radians(CRASH_TILT_DEG) or (
                phase in ("descend", "disarm") and h_raw < 0.4 and tilt > math.radians(LAND_TILT_DEG))
            if crash and st.armed and phase in ("find_hover", "takeoff", "hold", "descend", "disarm") \
                    and not force_sent:
                print(f"VOLTEO/CHOQUE: inclinación {math.degrees(tilt):.0f}° (h={h_raw:.2f} m) "
                      "-> desarme forzado inmediato")
                self.cmd_cli.call_async(CommandLong.Request(command=400, param1=0.0, param2=21196.0))
                force_sent = True
                phase, t_phase = "disarm", now

            # --- condiciones de aborto -> descenso ---
            if phase in ("find_hover", "takeoff", "hold"):
                reason = None
                if self.stop_requested:
                    reason = "interrupción del usuario"
                elif now - self.rng_t > 0.3:
                    reason = "ToF sin datos"
                elif h > a.height + 0.6:
                    reason = f"altura excesiva ({h:.2f} m)"
                elif tilt > math.radians(35):
                    reason = f"inclinación excesiva ({math.degrees(tilt):.0f}°)"
                elif phase == "find_hover" and max(abs(roll - roll0), abs(pitch - pitch0)) > math.radians(a.find_max_tilt):
                    reason = (f"se inclinó antes de despegar (Δroll={math.degrees(roll - roll0):+.1f}°, "
                              f"Δpitch={math.degrees(pitch - pitch0):+.1f}°; pitch+ = nariz abajo): "
                              "un lado empuja menos; revisa hélices/motores/centro de gravedad")
                elif not st.armed or st.mode != "OFFBOARD":
                    reason = f"PX4 salió de OFFBOARD/armado (mode={st.mode}, armed={st.armed})"
                if not reason and phase == "hold" and h_raw < self.ground_h + 0.03:
                    print(f"ABORTO: cayó al piso (h={h_raw:.3f} m) -> desarmando")
                    phase, t_phase = "disarm", now
                if reason and phase == "find_hover":
                    print("ABORTO:", reason, "-> desarmando (aún en el piso)")
                    phase, t_phase = "disarm", now
                elif reason:
                    print("ABORTO:", reason, "-> descendiendo")
                    phase, t_phase = "descend", now
            if phase == "arming" and self.stop_requested:
                print("Interrumpido antes de despegar")
                break

            # --- máquina de estados ---
            if phase == "arming":
                thrust = 0.0
                if st.mode != "OFFBOARD" and (mode_req is None or mode_req.done()):
                    mode_req = self.mode_cli.call_async(SetMode.Request(custom_mode="OFFBOARD"))
                elif st.mode == "OFFBOARD" and not st.armed and (arm_req is None or arm_req.done()):
                    arm_req = self.arm_cli.call_async(CommandBool.Request(value=True))
                if st.mode == "OFFBOARD" and st.armed:
                    self.thrust = a.thrust_min
                    if a.hover is None:
                        print("OFFBOARD + armado. Buscando empuje de despegue...")
                        phase, t_phase = "find_hover", now
                    else:
                        print("OFFBOARD + armado. Despegando.")
                        phase, t_phase, h_base = "takeoff", now, 0.0
                        h_ref = 0.0
                elif now - t_phase > 5.0:
                    print(f"No se pudo armar/entrar a OFFBOARD (mode={st.mode}, armed={st.armed}). "
                          "Revisa los avisos de PX4.")
                    break
            elif phase == "find_hover":
                # rampa lenta de empuje hasta que el ToF detecta que despegó
                thrust = self.thrust = min(a.find_max, a.thrust_min + a.find_rate * (now - t_phase))
                err = 0.0
                lift_ticks = lift_ticks + 1 if h > self.ground_h + 0.06 else 0
                if lift_ticks >= 3:
                    a.hover = max(a.thrust_min, thrust - 0.02)
                    a.thrust_max = min(0.9, a.hover + 0.2)
                    print(f"Despegó con empuje {thrust:.3f} -> hover={a.hover:.3f}, "
                          f"thrust_max={a.thrust_max:.3f}. Subiendo.")
                    phase, t_phase, h_base = "takeoff", now, h
                    h_ref = h
                elif thrust >= a.find_max:
                    print(f"No despegó con empuje {a.find_max:.2f} -> desarmando. "
                          "Revisa batería/hélices o sube --find-max.")
                    phase, t_phase = "disarm", now
            elif phase == "takeoff":
                # velocidad de referencia que frena con ACCEL_REF antes de la altura objetivo
                vz_ref = min(a.climb_rate, math.sqrt(2 * ACCEL_REF * max(0.0, a.height - h_ref)))
                h_ref = min(a.height, h_ref + vz_ref * dt)
                if a.height - h_ref < 0.003:
                    h_ref, vz_ref = a.height, 0.0
                # Mientras la referencia sube, sin integral (evita el sobrepaso). Ya en la altura
                # objetivo se activa, para corregir un hover estimado alto o bajo.
                reached = h_ref >= a.height
                t_reached = (t_reached or now) if reached else None
                thrust, err = self.pid(h_ref, h, dt, airborne, math.cos(roll) * math.cos(pitch),
                                       vz_ref=vz_ref, integrate=reached)
                if reached and (abs(h - a.height) < 0.1 or now - t_reached > 2.0):
                    print(f"Altura alcanzada ({h:.2f} m). Manteniendo {a.hold:.0f} s.")
                    phase, t_phase = "hold", now
                elif now - t_phase > (a.height - h_base) / a.climb_rate + 9.0:
                    print("No alcanzó la altura a tiempo (¿hover muy bajo?) -> descendiendo")
                    phase, t_phase = "descend", now
            elif phase == "hold":
                h_ref = a.height
                thrust, err = self.pid(h_ref, h, dt, airborne, math.cos(roll) * math.cos(pitch))
                if now - t_phase >= a.hold:
                    print("Tiempo cumplido. Aterrizando.")
                    phase, t_phase = "descend", now
            elif phase == "descend":
                if h_start is None:  # el descenso arranca desde donde esté
                    h_start = max(h_ref, h)
                h_ref = max(0.0, h_start - a.descent_rate * (now - t_phase))
                if h_ref > 0.0:
                    thrust, err = self.pid(h_ref, h, dt, airborne, math.cos(roll) * math.cos(pitch),
                                           vz_ref=-a.descent_rate)
                    t_ref0 = None
                else:
                    # referencia ya en el piso: bajar el empuje en rampa, sin PID
                    t_ref0 = t_ref0 or now
                    thrust = self.thrust = max(a.thrust_min, self.thrust - 0.5 * dt)
                    err = -h
                # en el piso según el ToF (no el EKF, que diverge con el ToF < 4 cm)
                grounded = h_raw < self.ground_h + 0.03
                landed_since = (landed_since or now) if grounded else None
                if landed_since and now - landed_since > 0.5:
                    print("En el piso (ToF). Desarmando.")
                    phase, t_phase = "disarm", now
                elif t_ref0 and now - t_ref0 > 2.0 and h < 0.15:
                    print("Referencia en el piso hace 2 s. Desarmando.")
                    phase, t_phase = "disarm", now
            elif phase == "disarm":
                thrust = 0.0
                if st.armed and (arm_req is None or arm_req.done()):
                    arm_req = self.arm_cli.call_async(CommandBool.Request(value=False))
                if st.armed and now - t_phase > 1.5 and not force_sent:
                    # PX4 puede negar el desarme si cree que no ha aterrizado: forzar (ya estamos en el piso)
                    print("PX4 no desarmó; forzando desarme.")
                    self.cmd_cli.call_async(CommandLong.Request(command=400, param1=0.0, param2=21196.0))
                    force_sent = True
                if not st.armed:
                    print("Desarmado. Fin.")
                    break
                if now - t_phase > 5.0:
                    print("PX4 no aceptó el desarme; usa Space/KILL en el dashboard.")
                    break

            if phase in ("arming", "disarm"):
                err = 0.0
            if phase in ("arming", "find_hover", "disarm"):
                # En el piso: pedir la misma inclinación de reposo. Si se pide 0° y el dron descansa
                # inclinado, PX4 no puede corregir y su integral se acumula -> se levanta un lado.
                self.publish(thrust, yaw0, roll0, pitch0)
            else:
                self.publish(thrust, yaw0)
            self.log.writerow([f"{now - t0:.3f}", phase, f"{h_ref:.3f}", f"{h:.3f}", f"{self.vz:.3f}",
                               f"{err:.3f}", f"{self.integral:.3f}", f"{thrust:.3f}",
                               f"{math.degrees(roll):.1f}", f"{math.degrees(pitch):.1f}",
                               f"{self.batt.voltage:.2f}" if self.batt else "", f"{self.vz_tof:.3f}", f"{self.terrain_offset:.3f}"]
                              + (self.motors + [""] * 4)[:4])
            while time.monotonic() < next_tick:
                rclpy.spin_once(self.n, timeout_sec=max(0.0, next_tick - time.monotonic()))
        self.log_file.close()
        if self.terrain_steps:
            print(f"Escalones del piso compensados en el vuelo: {self.terrain_steps}")
        print("Log:", a.log)
        return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--height", type=float, default=1.0, help="altura objetivo [m] (máx 1.5)")
    p.add_argument("--hold", type=float, default=20.0, help="tiempo en el aire [s]")
    p.add_argument("--hover", type=float, default=None,
                   help="empuje de hover normalizado 0-1; por defecto se mide al despegar")
    p.add_argument("--find-rate", type=float, default=0.12,
                   help="[1/s] rampa de empuje al buscar el hover")
    p.add_argument("--find-max-tilt", type=float, default=4.0,
                   help="[°] cambio máximo de inclinación respecto al reposo mientras busca el hover")
    p.add_argument("--find-max", type=float, default=0.6,
                   help="empuje máximo al buscar el hover antes de rendirse")
    p.add_argument("--kp", type=float, default=0.25)
    p.add_argument("--ki", type=float, default=0.15)
    p.add_argument("--kd", type=float, default=0.10)
    p.add_argument("--i-max", type=float, default=2.0, help="límite del integrador [m·s]")
    p.add_argument("--thrust-min", type=float, default=0.05)
    p.add_argument("--thrust-max", type=float, default=None, help="por defecto hover + 0.2")
    p.add_argument("--thrust-slew", type=float, default=1.0, help="máx. cambio de empuje por segundo")
    p.add_argument("--climb-rate", type=float, default=0.3, help="[m/s]")
    p.add_argument("--descent-rate", type=float, default=0.25, help="[m/s]")
    p.add_argument("--min-battery", type=float, default=50.0, help="[%%] mínimo para despegar")
    p.add_argument("--log", default="/tmp/hover_test.csv")
    p.add_argument("--dry-run", action="store_true", help="solo revisa sensores, no arma")
    # ros2 run / ros2 launch agregan --ros-args ...: se quitan antes de argparse
    a = p.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    t = HoverTest(a)
    # Cada opción también es un parámetro de ROS (p. ej. desde config/hover.yaml en un launch).
    # El parámetro de ROS gana sobre el valor por defecto de argparse. -1.0 significa "sin valor".
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
        t.n.get_logger().error("height debe estar entre 0.2 y 1.5 m")
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
