#!/usr/bin/env python3
"""Which sources tracked each robot, each second (build e13, Nick 10 Nov: "fuse every source present, and log which
sources were used each second"). The runs (freeroam.py, run_plan.py) call add() for every camera fix they take and
tick() about every second; sources.csv gets one row per robot per second:

    t, robot, tags, lock, motion, lights, ml, other, odometry_only

the count of fixes from each source that second (a source's name is the last part of the fix's src, e.g.
tag_bridge:motion -> motion; a tag bridge fix without a kind is a roof tag), and odometry_only 1 when no camera source
gave anything (the robot ran on its own wheels and gyro, fused by its filter). Never stops a run (errors are ignored).
"""
import csv
import os

VERSION = "source-log-2026-11-10e13"
KINDS = ("tags", "lock", "motion", "lights", "ml")


def kind_of(src):
    s = str(src or "")
    k = s.split(":")[-1] if ":" in s else s
    if k in KINDS:
        return k
    if s.startswith("tag_bridge") or k in ("tag", "roof tag", "camera"):
        return "tags"
    return "other"


class SourceLog:
    def __init__(self, folder, robots=()):
        self.robots = list(robots)
        self.n = {}
        self.t_sec = None
        self.f = None
        try:
            os.makedirs(folder, exist_ok=True)
            self.f = open(os.path.join(folder, "sources.csv"), "w", newline="", buffering=1)
            self.w = csv.writer(self.f)
            self.w.writerow(["t", "robot"] + list(KINDS) + ["other", "odometry_only"])
        except OSError:
            self.f = None

    def add(self, robot, src, t=None):
        if robot is None:
            robot = "(unnamed)"
        if robot not in self.robots and robot != "(unnamed)":
            self.robots.append(robot)
        d = self.n.setdefault(robot, {})
        k = kind_of(src)
        d[k] = d.get(k, 0) + 1

    def tick(self, now):
        sec = int(now)
        if self.t_sec is None:
            self.t_sec = sec
            return
        if sec == self.t_sec or self.f is None:
            return
        try:
            for r in self.robots + (["(unnamed)"] if "(unnamed)" in self.n else []):
                d = self.n.get(r, {})
                tot = sum(d.values())
                self.w.writerow([self.t_sec, r] + [d.get(k, 0) for k in KINDS] + [d.get("other", 0),
                                                                                  1 if tot == 0 else 0])
        except (OSError, ValueError):
            pass
        self.n = {}
        self.t_sec = sec

    def close(self):
        try:
            if self.f is not None:
                self.f.close()
        except OSError:
            pass
