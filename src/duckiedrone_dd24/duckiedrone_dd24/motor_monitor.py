#!/usr/bin/env python3
"""Muestra en vivo inclinación (IMU) y la salida de PX4 a cada motor (/mavros/rc/out).

Prueba SIN HÉLICES: arma en STABILIZED con algo de throttle e inclina el dron con la mano.
  - Nariz abajo  (pitch +) -> deben SUBIR los motores de ADELANTE: m1_FR y m3_FL
  - Lado derecho abajo (roll +) -> deben SUBIR los motores DERECHOS: m1_FR y m4_RR
"""
import math
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from mavros_msgs.msg import RCOut


def main():
    rclpy.init()
    n = rclpy.create_node("dd_motor_monitor")
    st = {"imu": None, "m": []}
    n.create_subscription(Imu, "/mavros/imu/data", lambda m: st.__setitem__("imu", m),
                          qos_profile_sensor_data)
    n.create_subscription(RCOut, "/mavros/rc/out", lambda m: st.__setitem__("m", list(m.channels[:4])),
                          qos_profile_sensor_data)
    print("roll+ = derecha abajo, pitch+ = nariz abajo.  Ctrl-C para salir")
    print(f"{'roll':>7} {'pitch':>7} | {'m1_FR':>6} {'m3_FL':>6} | {'m2_RL':>6} {'m4_RR':>6} | "
          "adelante-atrás  derecha-izq")
    try:
        while rclpy.ok():
            end = time.monotonic() + 0.25
            while time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=0.02)
            q, m = st["imu"], st["m"]
            if q is None or len(m) < 4:
                continue
            o = q.orientation
            roll = math.degrees(math.atan2(2 * (o.w * o.x + o.y * o.z), 1 - 2 * (o.x * o.x + o.y * o.y)))
            # FLU (ROS): pitch+ = nariz abajo
            pitch = math.degrees(math.asin(max(-1, min(1, 2 * (o.w * o.y - o.z * o.x)))))
            m1, m2, m3, m4 = m
            print(f"{roll:+7.1f} {pitch:+7.1f} | {m1:6d} {m3:6d} | {m2:6d} {m4:6d} | "
                  f"{(m1 + m3) - (m2 + m4):+8d}  {(m1 + m4) - (m2 + m3):+10d}")
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
