"""Extended Kalman filter for one Alvik.

The filter keeps a best guess of the robot's state and how unsure it is
(a covariance matrix). Each sensor nudges the guess in proportion to how much
it is trusted; a sensor that is off or blocked simply gives no nudge, and the
uncertainty grows until the next reading arrives.

State (floor frame, cm and radians):
    x, y     position
    th       heading (0 = +x, counter-clockwise positive)
    v        forward speed (cm/s)
    w        turn rate (rad/s)
    b        gyro bias (rad/s): what the gyro reads when the robot is still

Measurements:
    wheel_v   forward speed from the wheel encoders (RAW v_cms)
    wheel_w   turn rate from the wheel encoders (RAW w_dps); slips in turns
    gyro      turn rate + bias (RAW gz); does not slip, but drifts
    zupt      "zero velocity update": wheels say the robot is still, so v = w = 0
    cam_xy    camera position fix (cm), possibly late
    cam_th    camera heading, when the detector gives one
    cam_light camera fix of the robot's top LIGHT (blink codes), which sits a
              fixed offset (forward, left) from the centre: h = position +
              R(th) offset, so it also says a little about heading
    heading   a heading fix with its own noise (heading from motion, the
              calibration rolls / start nudges)
"""
import math

import numpy as np

D2R = math.pi / 180.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Noise:
    """How much each sensor is trusted (1 standard deviation). These are
    starting values; fit them from E1 data (still periods and tape errors)."""

    def __init__(self, **kw):
        self.v_base = 0.3          # cm/s, wheel speed noise
        self.v_rel = 0.03          # fraction of speed (wheel size error, small slips)
        self.w_base = 1.0 * D2R    # rad/s, wheel turn-rate noise
        self.w_rel = 0.08          # fraction of turn rate (wheels slip most when turning)
        self.g_base = 0.3 * D2R    # rad/s, gyro noise
        self.g_rel = 0.01          # gyro scale error
        self.zupt = 0.02           # cm/s and rad/s: how sure "still" is
        self.cam_xy = 0.8          # cm, camera position noise (replace with E4's result)
        self.cam_th = 8.0 * D2R    # rad, camera heading noise
        self.blink_xy = 0.5        # cm, blink light position noise (blink.py: ~0.15 px = 0.04 cm; the rest
                                   # is calibration, LED height and offset error; replace with a measurement)
        self.q_v = 40.0            # (cm/s)/sqrt(s): how fast speed can change
        self.q_w = 4.0             # (rad/s)/sqrt(s): how fast turn rate can change
        self.q_b = 0.01 * D2R      # (rad/s)/sqrt(s): gyro bias drift
        self.q_pos = 0.05          # cm/sqrt(s): bumps not explained by the model
        for k, v in kw.items():
            if not hasattr(self, k):
                raise KeyError(f"unknown noise setting {k}")
            setattr(self, k, v)


class EKF:
    def __init__(self, x, y, th, t, noise=None, sigma_xy=0.3, sigma_th=1.0 * D2R):
        self.n = noise or Noise()
        self.s = np.array([x, y, th, 0.0, 0.0, 0.0])
        self.P = np.diag([sigma_xy ** 2, sigma_xy ** 2, sigma_th ** 2, 1.0, (5 * D2R) ** 2, (1.0 * D2R) ** 2])
        self.t = t
        self.last_cam = t   # when the camera last corrected position

    # ---- copy / restore (used to rewind for late camera readings) ----
    def snapshot(self):
        return (self.s.copy(), self.P.copy(), self.t, self.last_cam)

    def restore(self, snap):
        self.s, self.P, self.t, self.last_cam = snap[0].copy(), snap[1].copy(), snap[2], snap[3]

    # ---- motion ----
    def predict(self, t):
        dt = t - self.t
        if dt <= 0:
            return
        x, y, th, v, w, b = self.s
        c, s = math.cos(th), math.sin(th)
        self.s[0] = x + v * c * dt
        self.s[1] = y + v * s * dt
        self.s[2] = wrap(th + w * dt)
        F = np.eye(6)
        F[0, 2] = -v * s * dt
        F[0, 3] = c * dt
        F[1, 2] = v * c * dt
        F[1, 3] = s * dt
        F[2, 4] = dt
        n = self.n
        Q = np.diag([n.q_pos ** 2 * dt, n.q_pos ** 2 * dt, 0.0, n.q_v ** 2 * dt, n.q_w ** 2 * dt, n.q_b ** 2 * dt])
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def predicted(self, t):
        """State and covariance at time t, without changing the filter."""
        snap = self.snapshot()
        self.predict(t)
        out = (self.s.copy(), self.P.copy())
        self.restore(snap)
        return out

    # ---- measurement update ----
    def _update(self, z, h, H, R, angle_rows=()):
        y = np.atleast_1d(z - h)
        for i in angle_rows:
            y[i] = wrap(y[i])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.s = self.s + K @ y
        self.s[2] = wrap(self.s[2])
        I_KH = np.eye(6) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T   # Joseph form: stays symmetric and positive
        return float(y @ np.linalg.solve(S, y))          # squared Mahalanobis distance of the reading

    def wheel_v(self, v):
        r = self.n.v_base + self.n.v_rel * abs(v)
        H = np.zeros((1, 6))
        H[0, 3] = 1
        self._update(np.array([v]), np.array([self.s[3]]), H, np.array([[r * r]]))

    def wheel_w(self, w):
        r = self.n.w_base + self.n.w_rel * abs(w)
        H = np.zeros((1, 6))
        H[0, 4] = 1
        self._update(np.array([w]), np.array([self.s[4]]), H, np.array([[r * r]]))

    def gyro(self, g):
        r = self.n.g_base + self.n.g_rel * abs(g)
        H = np.zeros((1, 6))
        H[0, 4] = 1
        H[0, 5] = 1
        self._update(np.array([g]), np.array([self.s[4] + self.s[5]]), H, np.array([[r * r]]))

    def zupt(self):
        H = np.zeros((2, 6))
        H[0, 3] = 1
        H[1, 4] = 1
        r = self.n.zupt
        self._update(np.zeros(2), self.s[3:5].copy(), H, np.diag([r * r, (r * D2R) ** 2]))

    def cam_d2(self, xy, t=None):
        """Squared Mahalanobis distance of a camera fix from the predicted position."""
        s, P = (self.s, self.P) if t is None else self.predicted(t)
        S = P[:2, :2] + np.eye(2) * self.n.cam_xy ** 2
        d = np.asarray(xy) - s[:2]
        return float(d @ np.linalg.solve(S, d))

    def cam_xy(self, xy):
        H = np.zeros((2, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        d2 = self._update(np.asarray(xy, float), self.s[:2].copy(), H, np.eye(2) * self.n.cam_xy ** 2)
        self.last_cam = self.t
        return d2

    def cam_th(self, th):
        H = np.zeros((1, 6))
        H[0, 2] = 1
        self._update(np.array([th]), np.array([self.s[2]]), H, np.array([[self.n.cam_th ** 2]]), angle_rows=(0,))

    def light_h(self, off, s=None):
        """Where the light should be seen (floor cm) and its Jacobian (2x6) for a
        light at `off` = (forward, left) cm from the robot centre."""
        s = self.s if s is None else s
        f, l = off
        c, sn = math.cos(s[2]), math.sin(s[2])
        h = np.array([s[0] + c * f - sn * l, s[1] + sn * f + c * l])
        H = np.zeros((2, 6))
        H[0, 0] = H[1, 1] = 1
        H[0, 2] = -sn * f - c * l
        H[1, 2] = c * f - sn * l
        return h, H

    def light_d2(self, xy, off, sigma, t=None):
        """Squared Mahalanobis distance of a light fix from where the light should be."""
        s, P = (self.s, self.P) if t is None else self.predicted(t)
        h, H = self.light_h(off, s)
        S = H @ P @ H.T + np.eye(2) * sigma ** 2
        d = np.asarray(xy) - h
        return float(d @ np.linalg.solve(S, d))

    def cam_light(self, xy, off, sigma=None):
        sigma = self.n.blink_xy if sigma is None else sigma
        h, H = self.light_h(off)
        d2 = self._update(np.asarray(xy, float), h, H, np.eye(2) * sigma ** 2)
        self.last_cam = self.t
        return d2

    def heading(self, th, sigma):
        """Heading fix th (rad) with noise sigma (rad). Returns its squared
        Mahalanobis distance (before the update)."""
        H = np.zeros((1, 6))
        H[0, 2] = 1
        return self._update(np.array([th]), np.array([self.s[2]]), H, np.array([[sigma ** 2]]), angle_rows=(0,))

    def heading_d2(self, th):
        y = wrap(th - self.s[2])
        return y, self.P[2, 2]

    def lost(self, sigma_xy=100.0, sigma_th=math.pi):
        """The robot was picked up: we no longer know where it is."""
        self.P[0:3, :] = 0
        self.P[:, 0:3] = 0
        self.P[0, 0] = self.P[1, 1] = sigma_xy ** 2
        self.P[2, 2] = sigma_th ** 2
        self.s[3] = self.s[4] = 0.0

    def reset_position(self, x, y, sigma_xy=1.0, th=None, sigma_th=None):
        self.s[0], self.s[1] = x, y
        self.P[0:2, :] = 0
        self.P[:, 0:2] = 0
        self.P[0, 0] = self.P[1, 1] = sigma_xy ** 2
        if th is not None:
            self.s[2] = th
            self.P[2, :] = 0
            self.P[:, 2] = 0
            self.P[2, 2] = (sigma_th or 10 * D2R) ** 2
        self.last_cam = self.t

    @property
    def sigma_xy(self):
        return math.sqrt(max(self.P[0, 0], self.P[1, 1]))
