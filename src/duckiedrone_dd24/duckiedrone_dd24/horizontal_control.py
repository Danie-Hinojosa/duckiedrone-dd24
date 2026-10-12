"""Control horizontal del DD24: estimador (Kalman con AprilTags + IMU) y controladores.

Todo trabaja en el marco "tags" (x al frente del acomodo de AprilTags, y a la izquierda) y
entrega roll/pitch para el AttitudeTarget de PX4 (el mismo canal que el control de altura).

Modos:
  pd  PID de posición -> aceleración deseada -> inclinación.
  l1  El mismo PD sin integral, más una aumentación L1 adaptativa en el lazo de velocidad que
      estima una perturbación de aceleración (empuje desviado por centro de gravedad corrido,
      nivel desajustado, arrastre, efecto suelo) y la compensa a través de un filtro pasa-bajas.
"""
import math

import numpy as np

G = 9.81


class AxisKalman:
    """Posición y velocidad en un eje: predicción con la aceleración de la IMU, corrección con los tags."""

    def __init__(self, sigma_acc=0.6, sigma_pos=0.03, gate=0.30):
        self.x = np.zeros(2)
        self.P = np.diag([1.0, 1.0])
        self.qa = sigma_acc ** 2
        self.r = sigma_pos ** 2
        self.gate = gate          # [m] innovación máxima aceptada (descarta saltos de los tags)
        self.rejected = 0
        self.ready = False

    def reset(self, p):
        self.x = np.array([p, 0.0])
        self.P = np.diag([self.r, 0.25])
        self.rejected = 0
        self.ready = True

    def predict(self, a, dt):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        B = np.array([0.5 * dt * dt, dt])
        self.x = F @ self.x + B * a
        Q = self.qa * np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt * dt]])
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        y = z - self.x[0]
        if abs(y) > self.gate and self.rejected < 5:
            self.rejected += 1      # varios rechazos seguidos = el dron sí se movió: se acepta
            return False
        self.rejected = 0
        S = self.P[0, 0] + self.r
        K = self.P[:, 0] / S
        self.x = self.x + K * y
        self.P = self.P - np.outer(K, self.P[0, :])
        return True


class L1Axis:
    """Aumentación L1 en el lazo de velocidad de un eje (predictor + adaptación + filtro)."""

    def __init__(self, a_m=-4.0, gamma=60.0, omega_c=1.5, sigma_max=2.0):
        self.a_m, self.gamma, self.wc, self.smax = a_m, gamma, omega_c, sigma_max
        self.v_hat = 0.0
        self.sigma = 0.0
        self.sigma_f = 0.0

    def reset(self, v):
        self.v_hat, self.sigma, self.sigma_f = v, 0.0, 0.0

    def step(self, v, a_applied, dt):
        e = self.v_hat - v
        self.v_hat += dt * (a_applied + self.sigma + self.a_m * e)
        self.sigma = max(-self.smax, min(self.smax, self.sigma - dt * self.gamma * e))
        self.sigma_f += dt * self.wc * (self.sigma - self.sigma_f)
        return self.sigma_f


class HorizontalController:
    def __init__(self, mode="l1", kp=1.5, kd=2.0, ki=0.5, i_frac=0.7, max_tilt_deg=10.0,
                 l1_gamma=60.0, l1_omega_c=1.5, tag_timeout=0.5):
        assert mode in ("pd", "l1")
        self.mode = mode
        self.kp, self.kd, self.ki = kp, kd, (0.0 if mode == "l1" else ki)
        self.a_max = G * math.tan(math.radians(max_tilt_deg))
        self.i_max = i_frac * self.a_max   # la integral (en m/s^2) puede usar hasta este tanto del límite
        self.kf = [AxisKalman(), AxisKalman()]
        self.l1 = [L1Axis(gamma=l1_gamma, omega_c=l1_omega_c), L1Axis(gamma=l1_gamma, omega_c=l1_omega_c)]
        self.integral = [0.0, 0.0]
        self.a_prev = [0.0, 0.0]
        self.ref = None
        self.heading = None          # yaw del dron en el marco tags (de los AprilTags)
        self.yaw_offset = None       # yaw_tags - yaw_px4 (para pasar la IMU al marco tags)
        self.t_tag = -1e9
        self.tag_timeout = tag_timeout
        self.sigma = [0.0, 0.0]

    # ---- mediciones ----
    def on_tag(self, now, x, y, yaw_tags, yaw_px4):
        off = math.atan2(math.sin(yaw_tags - yaw_px4), math.cos(yaw_tags - yaw_px4))
        if self.yaw_offset is None:
            self.yaw_offset = off
        else:   # promedio lento (el yaw de PX4 sin magnetómetro deriva poco a poco)
            d = math.atan2(math.sin(off - self.yaw_offset), math.cos(off - self.yaw_offset))
            self.yaw_offset += 0.05 * d
        self.heading = yaw_tags
        for kf, z in zip(self.kf, (x, y)):
            if not kf.ready:
                kf.reset(z)
            else:
                kf.update(z)
        self.t_tag = now

    def predict(self, a_world_px4, dt):
        """a_world_px4: aceleración (sin gravedad) en el marco ENU de PX4."""
        if self.yaw_offset is None:
            return
        c, s = math.cos(self.yaw_offset), math.sin(self.yaw_offset)
        ax = c * a_world_px4[0] - s * a_world_px4[1]
        ay = s * a_world_px4[0] + c * a_world_px4[1]
        for kf, a in zip(self.kf, (ax, ay)):
            if kf.ready:
                kf.predict(a, dt)

    # ---- estado ----
    def tags_ok(self, now):
        return now - self.t_tag < self.tag_timeout and all(k.ready for k in self.kf)

    def state(self):
        return [k.x[0] for k in self.kf], [k.x[1] for k in self.kf]

    def set_reference(self, x, y):
        self.ref = (x, y)

    def reset_integrators(self):
        self.integral = [0.0, 0.0]
        self.sigma = [0.0, 0.0]
        for l1, kf in zip(self.l1, self.kf):
            l1.reset(kf.x[1])
        self.a_prev = [0.0, 0.0]

    # ---- control ----
    def control(self, now, dt, integrate=True):
        """Devuelve (roll, pitch) deseados [rad] en el marco de rumbo del dron, y datos para el log."""
        if self.ref is None or not self.tags_ok(now) or self.heading is None:
            self.reset_integrators()
            return 0.0, 0.0, {"ok": False}
        (px, py), (vx, vy) = self.state()
        a = []
        for i, (p, v, r) in enumerate(((px, vx, self.ref[0]), (py, vy, self.ref[1]))):
            e = r - p
            if integrate and self.ki > 0:
                self.integral[i] = max(-self.i_max, min(self.i_max, self.integral[i] + self.ki * e * dt))
            ai = self.kp * e - self.kd * v + self.integral[i]
            if self.mode == "l1":
                if integrate:      # sin integrar (bajando) se conserva la última estimación
                    self.sigma[i] = self.l1[i].step(v, self.a_prev[i], dt)
                ai -= self.sigma[i]
            a.append(ai)
        # saturación conservando la dirección
        n = math.hypot(a[0], a[1])
        if n > self.a_max:
            a = [ai * self.a_max / n for ai in a]
        self.a_prev = list(a)
        # marco tags -> marco de rumbo del dron
        c, s = math.cos(self.heading), math.sin(self.heading)
        ax_b = c * a[0] + s * a[1]
        ay_b = -s * a[0] + c * a[1]
        pitch = math.atan2(ax_b, G)      # pitch+ (nariz abajo) acelera hacia +x
        roll = -math.atan2(ay_b, G)      # roll+ (derecha abajo) acelera hacia -y
        return roll, pitch, {"ok": True, "px": px, "py": py, "vx": vx, "vy": vy,
                             "ax": a[0], "ay": a[1], "sx": self.sigma[0], "sy": self.sigma[1]}
