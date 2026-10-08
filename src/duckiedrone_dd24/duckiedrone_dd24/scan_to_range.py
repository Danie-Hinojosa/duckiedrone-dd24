#!/usr/bin/env python3
"""Simulación: convierte el lidar de un rayo de Gazebo (LaserScan) en el sensor_msgs/Range que
publica el driver del ToF en el dron real, en el mismo tópico. Así los nodos de control
(hover_test y los que hagamos) corren igual en la simulación y en el dron.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class ScanToRange(Node):
    def __init__(self):
        super().__init__("tof_bottom_sim")
        vehicle = self.declare_parameter("vehicle", "duckiedrone01").value
        self.frame = self.declare_parameter("frame_id", "tof_bottom_link").value
        self.fov = self.declare_parameter("field_of_view", 0.471).value  # VL53 real: 27°
        self.pub = self.create_publisher(Range, f"/{vehicle}/bottom_tof_driver_node/range",
                                         qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/dd24/tof_bottom/scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, scan: LaserScan):
        if not scan.ranges:
            return
        r = scan.ranges[0]
        m = Range()
        m.header.stamp = scan.header.stamp
        m.header.frame_id = self.frame
        m.radiation_type = Range.INFRARED
        m.field_of_view = self.fov
        m.min_range = scan.range_min
        m.max_range = scan.range_max
        # gz devuelve inf fuera de rango; el driver real reporta el máximo
        m.range = scan.range_max if math.isinf(r) or math.isnan(r) else float(r)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ScanToRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
