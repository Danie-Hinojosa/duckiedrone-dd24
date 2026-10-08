import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
rclpy.init(); n = rclpy.create_node("tof_relay_tmp")
p = n.create_publisher(Range, "/mavros/bottom_tof", qos_profile_sensor_data)
n.create_subscription(Range, "/duckiedrone01/bottom_tof_driver_node/range", p.publish, qos_profile_sensor_data)
rclpy.spin(n)
