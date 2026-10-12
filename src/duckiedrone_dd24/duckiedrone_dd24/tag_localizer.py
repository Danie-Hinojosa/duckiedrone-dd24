#!/usr/bin/env python3
"""Posición del dron a partir de AprilTags (tag36h11) pegados en el piso.

Detecta los tags con OpenCV en la imagen de la cámara (ojo de pez), corrige la distorsión con
un modelo equidistante, resuelve la pose de cada tag respecto a la cámara y, con el mapa de los
tags y la posición de la cámara en el dron, calcula la pose de base_link en el marco "tags"
(origen en el punto de despegue, x hacia el frente del acomodo, z hacia arriba).

Publica:
  /dd24/tag_pose (geometry_msgs/PoseStamped, frame_id = tags)
  /dd24/tag_debug (sensor_msgs/Image, opcional): imagen con los tags detectados
"""
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

# Mapa por defecto: la cuadrícula del PDF (x al frente, y a la izquierda, cada 0.4 m)
DEFAULT_MAP = {0: (0.8, 0.4), 1: (0.8, 0.0), 2: (0.8, -0.4), 3: (0.4, 0.4), 4: (0.4, 0.0), 5: (0.4, -0.4),
               6: (0.0, 0.4), 7: (0.0, -0.4), 8: (-0.4, 0.4), 9: (-0.4, 0.0), 10: (-0.4, -0.4)}


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def quat_from_R(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


class TagLocalizer(Node):
    def __init__(self):
        super().__init__("tag_localizer")
        p = self.declare_parameter
        self.tag_size = p("tag_size", 0.16).value                  # lado del cuadro negro [m]
        self.image_topic = p("image_topic", "/dd24/camera/image").value
        self.compressed = p("compressed", False).value               # True en el dron (JPEG)
        # Modelo de la cámara: ojo de pez equidistante (r = f * theta)
        self.f = p("focal_px", 229.2).value
        self.cx = p("cx", -1.0).value                                # -1: centro de la imagen
        self.cy = p("cy", -1.0).value
        self.D = np.array(p("fisheye_d", [0.0, 0.0, 0.0, 0.0]).value, dtype=float).reshape(4, 1)
        # Cámara en base_link (camera_link del URDF; x a lo largo del eje óptico)
        cam_xyz = p("camera_xyz", [0.1277, 0.0, 0.0082]).value
        cam_rpy = p("camera_rpy", [0.0, 0.8779, 0.0]).value
        # Giro de la textura/hoja del tag respecto al marco "tags" (se calibra en la simulación)
        self.tag_yaw = p("tag_yaw_offset", -math.pi / 2).value
        ids = p("tag_ids", list(DEFAULT_MAP.keys())).value
        xs = p("tag_x", [DEFAULT_MAP[i][0] for i in DEFAULT_MAP]).value
        ys = p("tag_y", [DEFAULT_MAP[i][1] for i in DEFAULT_MAP]).value
        self.publish_debug = p("publish_debug", False).value
        self.max_dist = p("max_tag_distance", 3.0).value

        self.tag_map = {int(i): transform(rot_rpy(0, 0, self.tag_yaw), [x, y, 0.0]) for i, x, y in zip(ids, xs, ys)}
        # camera_link -> marco óptico de OpenCV (x derecha, y abajo, z al frente)
        T_base_link = transform(rot_rpy(*cam_rpy), cam_xyz)
        T_link_opt = transform(rot_rpy(-math.pi / 2, 0, -math.pi / 2), [0, 0, 0])
        self.T_opt_base = np.linalg.inv(T_base_link @ T_link_opt)

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        s = self.tag_size / 2
        self.obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)

        self.pub = self.create_publisher(PoseStamped, "/dd24/tag_pose", 10)
        self.dbg = self.create_publisher(Image, "/dd24/tag_debug", 1) if self.publish_debug else None
        msg_type = CompressedImage if self.compressed else Image
        self.create_subscription(msg_type, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.n_frames = 0
        self.get_logger().info(f"{len(self.tag_map)} tags en el mapa, lado {self.tag_size} m, imagen {self.image_topic}")

    def to_gray(self, msg):
        if self.compressed:
            return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_GRAYSCALE)
        a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, -1)
        return cv2.cvtColor(a, cv2.COLOR_RGB2GRAY) if a.shape[2] == 3 else a[:, :, 0]

    def on_image(self, msg):
        gray = self.to_gray(msg)
        if gray is None:
            return
        h, w = gray.shape
        cx = w / 2 if self.cx < 0 else self.cx
        cy = h / 2 if self.cy < 0 else self.cy
        K = np.array([[self.f, 0, cx], [0, self.f, cy], [0, 0, 1]], dtype=np.float64)
        corners, ids, _ = self.detector.detectMarkers(gray)
        poses = []
        if ids is not None:
            for c, tid in zip(corners, ids.flatten()):
                if int(tid) not in self.tag_map:
                    continue
                # esquinas sin distorsión, en coordenadas normalizadas
                und = cv2.fisheye.undistortPoints(c.reshape(-1, 1, 2).astype(np.float64), K, self.D)
                ok, rvec, tvec = cv2.solvePnP(self.obj, und.reshape(-1, 2), np.eye(3), None,
                                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok or np.linalg.norm(tvec) > self.max_dist:
                    continue
                T_opt_tag = transform(cv2.Rodrigues(rvec)[0], tvec.flatten())
                T_tags_base = self.tag_map[int(tid)] @ np.linalg.inv(T_opt_tag) @ self.T_opt_base
                poses.append((T_tags_base, float(np.linalg.norm(tvec)), int(tid)))
        if self.dbg is not None:
            self.publish_debug_image(gray, corners, ids, msg.header)
        if not poses:
            return
        # promedio ponderado por cercanía (los tags cercanos se miden mejor)
        wts = np.array([1.0 / max(d, 0.2) ** 2 for _, d, _ in poses])
        pos = sum(wt * T[:3, 3] for wt, (T, _, _) in zip(wts, poses)) / wts.sum()
        yaws = [math.atan2(T[1, 0], T[0, 0]) for T, _, _ in poses]
        yaw = math.atan2(sum(wt * math.sin(a) for wt, a in zip(wts, yaws)), sum(wt * math.cos(a) for wt, a in zip(wts, yaws)))
        best = poses[int(np.argmax(wts))][0]
        # roll/pitch del tag más cercano (la IMU los mide mejor; se publican como referencia)
        R = rot_rpy(math.atan2(best[2, 1], best[2, 2]), math.asin(max(-1, min(1, -best[2, 0]))), yaw)
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "tags"
        out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float, pos)
        q = quat_from_R(R)
        out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w = q
        self.pub.publish(out)
        self.n_frames += 1
        if self.n_frames % 30 == 1:
            self.get_logger().info(f"tags {[t for _, _, t in poses]}: x={pos[0]:+.3f} y={pos[1]:+.3f} "
                                   f"z={pos[2]:+.3f} yaw={math.degrees(yaw):+.1f}°")

    def publish_debug_image(self, gray, corners, ids, header):
        img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
        m = Image()
        m.header = header
        m.height, m.width = img.shape[:2]
        m.encoding = "bgr8"
        m.step = m.width * 3
        m.data = img.tobytes()
        self.dbg.publish(m)


def main():
    rclpy.init()
    node = TagLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
