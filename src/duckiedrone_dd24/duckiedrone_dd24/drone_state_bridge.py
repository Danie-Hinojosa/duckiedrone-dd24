#!/usr/bin/env python3
"""Puente entre el dron (MAVROS / drivers de Duckietown) y RViz.

- TF map -> base_link a partir de /mavros/local_position/pose (EKF de PX4).
  Sin odometría visual, x/y del EKF no son confiables; z (con el ToF) y la orientación sí.
- /joint_states para las 4 hélices del URDF, girando según la salida a cada motor (/mavros/rc/out).
- Republica el ToF inferior en /dd24/tof_bottom con frame_id = tof_bottom_link.

Con demo:=true no necesita el dron: solo publica /joint_states en cero (para ver el modelo).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState, Range
from tf2_ros import TransformBroadcaster

ROTOR_JOINTS = ["rotor_1_joint", "rotor_2_joint", "rotor_3_joint", "rotor_4_joint"]
ROTOR_SIGN = [1.0, 1.0, -1.0, -1.0]   # 1 y 2 antihorario (CCW), 3 y 4 horario (CW), vistos desde arriba
DSHOT_IDLE, DSHOT_MAX = 109.0, 1999.0  # valores de /mavros/rc/out en reposo y a tope
VIS_MAX_RAD_S = 25.0                   # velocidad visual máxima (la real es demasiado rápida para verse)


class DroneStateBridge(Node):
    def __init__(self):
        super().__init__("drone_state_bridge")
        self.demo = self.declare_parameter("demo", False).value
        vehicle = self.declare_parameter("vehicle", "duckiedrone01").value
        self.frame_map = self.declare_parameter("map_frame", "map").value
        self.frame_base = self.declare_parameter("base_frame", "base_link").value

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.angles = [0.0] * 4
        self.speeds = [0.0] * 4
        self.last_t = self.get_clock().now()

        if not self.demo:
            from mavros_msgs.msg import RCOut  # solo hace falta con el dron (ros-jazzy-mavros-msgs)
            self.tf = TransformBroadcaster(self)
            self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.on_pose,
                                     qos_profile_sensor_data)
            self.create_subscription(RCOut, "/mavros/rc/out", self.on_rcout, qos_profile_sensor_data)
            self.tof_pub = self.create_publisher(Range, "/dd24/tof_bottom", qos_profile_sensor_data)
            self.create_subscription(Range, f"/{vehicle}/bottom_tof_driver_node/range", self.on_tof,
                                     qos_profile_sensor_data)
        self.create_timer(1.0 / 30.0, self.publish_joints)
        self.get_logger().info("modo demo (sin dron)" if self.demo else f"conectado a {vehicle}")

    def on_pose(self, m: PoseStamped):
        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id = self.frame_map
        t.child_frame_id = self.frame_base
        t.transform.translation.x = m.pose.position.x
        t.transform.translation.y = m.pose.position.y
        t.transform.translation.z = m.pose.position.z
        t.transform.rotation = m.pose.orientation
        self.tf.sendTransform(t)

    def on_rcout(self, m):
        for i, raw in enumerate(list(m.channels[:4])):
            frac = 0.0 if raw < DSHOT_IDLE else (raw - DSHOT_IDLE) / (DSHOT_MAX - DSHOT_IDLE)
            self.speeds[i] = ROTOR_SIGN[i] * VIS_MAX_RAD_S * min(1.0, max(0.0, frac) + (0.15 if raw >= DSHOT_IDLE else 0.0))

    def on_tof(self, m: Range):
        m.header.frame_id = "tof_bottom_link"
        self.tof_pub.publish(m)

    def publish_joints(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        for i in range(4):
            self.angles[i] = math.remainder(self.angles[i] + self.speeds[i] * dt, 2 * math.pi)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ROTOR_JOINTS
        js.position = list(self.angles)
        self.joint_pub.publish(js)


def main():
    rclpy.init()
    node = DroneStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
