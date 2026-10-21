"""Shared helpers for the E02 runner tests (lane L): import paths and a small kinematic robot that
produces what the camera and the robot's RAW log would show."""
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
EXP = os.path.dirname(HERE)
TOOLS = os.path.dirname(EXP)
for p in (EXP, TOOLS, os.path.join(TOOLS, "recognition"), os.path.join(TOOLS, "live"), os.path.join(TOOLS, "sim")):
    if p not in sys.path:
        sys.path.insert(0, p)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class KinRobot:
    """A robot with a gyro that counts 1/g of every turn and creeps b deg/s, and wheels that travel w real cm
    per counted cm. It drives commands with its own (uncorrected) sensors, as a U run does, and records
    camera samples (true pose + noise) and RAW rows (counted pose, raw yaw)."""

    def __init__(self, g=1.075, b=0.2, w=0.978, x=10.0, y=10.0, th=0.0, cam_sd=0.3, seed=1, t0=1000.0,
                 boot_ms=50000):
        self.g, self.b, self.w = g, b, w
        self.x, self.y, self.th = x, y, th
        self.raw = 37.0
        self.lx = self.ly = 0.0
        self.t = t0
        self.ms0 = boot_ms
        self.t0 = t0
        self.rng = random.Random(seed)
        self.cam_sd = cam_sd
        self.cam, self.rawrows = [], []
        self._next_cam = t0
        self._next_raw = t0

    def _tick(self, dt, v_counted, w_true):
        self.th = wrap180(self.th + w_true * dt)
        self.raw += (w_true / self.g + self.b) * dt
        vt = v_counted * self.w
        self.x += vt * dt * math.cos(math.radians(self.th))
        self.y += vt * dt * math.sin(math.radians(self.th))
        self.lx += v_counted * dt * math.cos(math.radians(self.raw))
        self.ly += v_counted * dt * math.sin(math.radians(self.raw))
        self.t += dt
        if self.t >= self._next_cam:
            self._next_cam += 0.04
            self.cam.append((self.t, self.x + self.rng.gauss(0, self.cam_sd), self.y + self.rng.gauss(0, self.cam_sd)))
        if self.t >= self._next_raw:
            self._next_raw += 0.05
            ms = self.ms0 + (self.t - self.t0) * 1000.0
            self.rawrows.append(dict(t_recv=self.t + 0.004 + abs(self.rng.gauss(0, 0.01)), ms=ms, x_cm=self.lx,
                                     y_cm=self.ly, th_deg=wrap180(self.raw), yaw_deg=wrap180(self.raw)))

    def go(self, d, speed=8.0, dt=0.02):
        """Drive d counted cm holding the raw heading (U: believed = raw)."""
        h0 = self.raw
        done = 0.0
        sgn = 1.0 if d >= 0 else -1.0
        while done < abs(d):
            err = h0 - self.raw
            w_true = max(-30.0, min(30.0, 4.0 * err)) * self.g
            self._tick(dt, sgn * speed, w_true)
            done += speed * dt

    def turn(self, deg, rate=60.0, dt=0.02):
        target = self.raw + deg
        while abs(target - self.raw) > 0.3:
            e = target - self.raw
            self._tick(dt, 0.0, math.copysign(min(rate, 3 * abs(e) + 5), e))

    def wait(self, s, dt=0.02):
        n = int(s / dt)
        for _ in range(n):
            self._tick(dt, 0.0, 0.0)


def u_run(robot, route=1):
    """A find-like start and a route out and back, as a U run would drive it."""
    robot.wait(0.5)
    robot.go(-5)
    robot.turn(-90)
    robot.go(20)
    robot.wait(0.5)
    robot.go(-5)
    robot.turn(180)
    robot.wait(0.3)
    if route == 1:
        robot.go(60)
        robot.wait(1.0)
        robot.turn(180)
        robot.go(60)
    elif route == 2:
        robot.go(30)
        robot.turn(90)
        robot.go(30)
        robot.wait(1.0)
        robot.turn(180)
        robot.go(30)
        robot.turn(-90)
        robot.go(30)
    else:
        for _ in range(4):
            robot.go(40)
            robot.turn(90)
    robot.wait(0.5)
