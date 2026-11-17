#!/usr/bin/env python3
# COMP 581, Fall 2026 - Lab 4 desk simulation (laptop only; not for the robot)
# Team number: TODO
# Tanish Bachu: 730739311
# Minh Nguyen: 730766544
#
# Runs the real ../main.py against a fake pybricks API and a 2D world:
# obstacle polygons, a body shaped like the Lab 3 robot with the touch rod in
# front, a left sonar with a narrow beam that loses oblique walls, a gyro with
# drift and scale error, and wheels slightly off their calibration. Blocked
# wheels stall; a body sliding along a wall scrapes (odometry error).
#
#   python3 sim.py                    built-in layouts
#   python3 sim.py --random 40        plus 40 random layouts
#   python3 sim.py --only figure,u    just these layouts
#   python3 sim.py --plot             save path plots in out/
#   python3 sim.py --set ALIGN_SCAN=False   override a main.py constant

import argparse
import math
import os
import random
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN = os.path.join(HERE, "..", "main.py")
OUT = os.path.join(HERE, "out")

START = (500.0, 0.0)
GOAL = (2500.0, 2500.0)
WORKSPACE = 4000.0
STEP = 5.0                      # ms per physics step

# Robot frame: x forward from the axle midpoint, y to the left (mm). A guess
# at the Lab 3 robot: the touch sensor sticks out in front of the brick, and
# its rod 24 mm past that. Only the rod's tip presses the button.
BODY = [[(55.0, 93.0), (55.0, -85.0), (-100.0, -85.0), (-100.0, 93.0)],   # brick, sonar
        [(82.0, 25.0), (82.0, -15.0), (55.0, -15.0), (55.0, 25.0)]]       # touch sensor
ROD = ((82.0, 6.0), (103.0, 6.0))   # solid part of the touch rod
TIP = [(106.0, 6.0)]                # pressed when this comes within 2 mm of a wall
# --bumper: a bar across the whole front, on the touch sensor.
BUMPER_BODY = [(103.0, 95.0), (103.0, -90.0), (82.0, -90.0), (82.0, 95.0)]
BUMPER_TIP = [(106.0, y) for y in range(-90, 96, 5)]
SONAR = (28.0, 93.0)
SONAR_OFFSET = 27.0
BEAM = 12.0                     # deg half-width of the sonar beam
INCIDENCE = 25.0                # deg; echoes off walls more oblique than this are lost
DROPOUT = 0.03                  # chance a reading returns no echo anyway
TWIST = math.radians(40.0)      # rad/s a body corner sliding along a wall turns the robot
PUSH_OFF = 40.0                 # mm/s a turn pressing a corner on a wall shoves the body off


# ------------------------------ geometry ------------------------------
def cross(ax, ay, bx, by):
    return ax * by - ay * bx


def segments_cross(p1, p2, q1, q2):
    d1x, d1y = p2[0] - p1[0], p2[1] - p1[1]
    d2x, d2y = q2[0] - q1[0], q2[1] - q1[1]
    den = cross(d1x, d1y, d2x, d2y)
    if abs(den) < 1e-12:
        return False
    ex, ey = q1[0] - p1[0], q1[1] - p1[1]
    t = cross(ex, ey, d2x, d2y) / den
    u = cross(ex, ey, d1x, d1y) / den
    return 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0


def inside(x, y, poly):
    hit = False
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            hit = not hit
    return hit


def point_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
    qx, qy = ax + t * dx, ay + t * dy
    return math.hypot(px - qx, py - qy)


def perimeter(poly):
    return sum(math.dist(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


class Obstacle:
    def __init__(self, pts):
        self.pts = [(float(x), float(y)) for x, y in pts]
        self.edges = [(self.pts[i], self.pts[(i + 1) % len(self.pts)]) for i in range(len(self.pts))]
        cx = sum(p[0] for p in self.pts) / len(self.pts)
        cy = sum(p[1] for p in self.pts) / len(self.pts)
        self.c = (cx, cy)
        self.r = max(math.dist(self.c, p) for p in self.pts)

    def distance(self, x, y):
        if inside(x, y, self.pts):
            return 0.0
        return min(point_segment(x, y, a[0], a[1], b[0], b[1]) for a, b in self.edges)


def poly_distance(a, b):
    for e in a.edges:
        for f in b.edges:
            if segments_cross(e[0], e[1], f[0], f[1]):
                return 0.0
    if inside(a.pts[0][0], a.pts[0][1], b.pts) or inside(b.pts[0][0], b.pts[0][1], a.pts):
        return 0.0
    return min(min(b.distance(*p) for p in a.pts), min(a.distance(*p) for p in b.pts))


class World:
    def __init__(self, polys, bumper=False):
        self.obstacles = [Obstacle(p) for p in polys]
        self.body = BODY + ([BUMPER_BODY] if bumper else [])
        self.tips = BUMPER_TIP if bumper else TIP

    def near(self, x, y, reach):
        return [o for o in self.obstacles if math.hypot(o.c[0] - x, o.c[1] - y) <= o.r + reach]

    def clearance(self, x, y):
        if not self.obstacles:
            return 1e9
        return min(o.distance(x, y) for o in self.obstacles)

    def collides(self, x, y, th):
        near = self.near(x, y, 160.0)
        if not near:
            return False
        c, s = math.cos(th), math.sin(th)

        def w(p):
            return (x + p[0] * c - p[1] * s, y + p[0] * s + p[1] * c)
        parts = [[w(p) for p in part] for part in self.body]
        segs = [(w(ROD[0]), w(ROD[1]))]
        for part in parts:
            segs += [(part[i], part[(i + 1) % len(part)]) for i in range(len(part))]
        for o in near:
            for a, b in segs:
                for e in o.edges:
                    if segments_cross(a, b, e[0], e[1]):
                        return True
            for p in o.pts:
                if any(inside(p[0], p[1], part) for part in parts):
                    return True
            if inside(parts[0][0][0], parts[0][0][1], o.pts):
                return True
        return False

    def contact_tangent(self, x, y, th):
        # Unit vector along the obstacle edge nearest the body at this pose.
        c, s = math.cos(th), math.sin(th)
        pts = [(x + p[0] * c - p[1] * s, y + p[0] * s + p[1] * c)
               for part in self.body for p in part] + [
              (x + p[0] * c - p[1] * s, y + p[0] * s + p[1] * c) for p in ROD]
        best = None
        for o in self.near(x, y, 160.0):
            for a, b in o.edges:
                for px, py in pts:
                    dist = point_segment(px, py, a[0], a[1], b[0], b[1])
                    if best is None or dist < best[0]:
                        L = math.dist(a, b)
                        best = (dist, ((b[0] - a[0]) / L, (b[1] - a[1]) / L), (px, py))
        return None if best is None else best[1:]

    def touched(self, x, y, th):
        c, s = math.cos(th), math.sin(th)
        for px, py in self.tips:
            tx, ty = x + px * c - py * s, y + px * s + py * c
            if any(o.distance(tx, ty) <= 2.0 for o in self.near(tx, ty, 10.0)):
                return True
        return False

    def ray(self, ox, oy, ang):
        dx, dy = math.cos(ang), math.sin(ang)
        best = None
        for o in self.near(ox, oy, 2600.0):
            for a, b in o.edges:
                ex, ey = b[0] - a[0], b[1] - a[1]
                den = cross(dx, dy, ex, ey)
                if abs(den) < 1e-12:
                    continue
                fx, fy = a[0] - ox, a[1] - oy
                t = cross(fx, fy, ex, ey) / den
                u = cross(fx, fy, dx, dy) / den
                if t > 0 and 0.0 <= u <= 1.0 and (best is None or t < best[0]):
                    L = math.hypot(ex, ey)
                    cos_inc = abs(dx * (-ey) + dy * ex) / L
                    best = (t, math.degrees(math.acos(min(1.0, cos_inc))))
        return best

    def sonar(self, x, y, th, rng):
        c, s = math.cos(th), math.sin(th)
        ox, oy = x + SONAR[0] * c - SONAR[1] * s, y + SONAR[0] * s + SONAR[1] * c
        best = None
        for k in range(-4, 5):
            hit = self.ray(ox, oy, th + math.pi / 2 + math.radians(BEAM * k / 4.0))
            if hit is not None and hit[1] <= INCIDENCE and (best is None or hit[0] < best):
                best = hit[0]
        if best is None or rng.random() < DROPOUT or best > 2500:
            return 2550
        return int(best + SONAR_OFFSET + rng.gauss(0, 3))


# --------------------------- fake pybricks ----------------------------
class SimEnd(Exception):
    pass


class Sim:
    def __init__(self, polys, seed, bumper=False, press_at=8000.0, max_time=200000.0):
        self.rng = random.Random(seed)
        self.world = World(polys, bumper)
        self.t = 0.0
        self.pt = 0.0
        self.th = math.pi / 2
        # Measuring point on START facing +Y: axle 106 mm behind, 6 mm right.
        self.x, self.y = START[0] + 6.0, START[1] - 106.0
        self.diam = [52.6 * (1 + self.rng.gauss(0, 0.004)), 52.6 * (1 + self.rng.gauss(0, 0.004))]
        self.track = 120.0 * (1 + self.rng.gauss(0, 0.01))
        self.angle = [0.0, 0.0]
        self.speed = [0.0, 0.0]
        self.target = [0.0, 0.0]
        self.braking = [True, True]
        self.stalled = False
        self.gyro_scale = 1 + self.rng.gauss(0, 0.007)
        self.gyro_drift = self.rng.uniform(-0.03, 0.03)  # deg/s
        self.gyro_value = 0.0
        self.gyro_zero = 0.0
        self.press_at = press_at
        self.max_time = max_time
        self.last_move = None
        self.beeps = []
        self.lines = []
        self.path = []
        self.next_sample = 0.0
        self.blocked_steps = 0
        self.violation = 0.0
        self.body_violation = 0.0
        self.robot = None

    def tip(self):
        c, s = math.cos(self.th), math.sin(self.th)
        return (self.x + 106.0 * c - 6.0 * s, self.y + 106.0 * s + 6.0 * c)

    def advance(self, ms):
        self.t += ms
        if self.t > self.max_time:
            raise SimEnd("simulation time limit")
        while self.pt + STEP <= self.t:
            self.step(STEP / 1000.0)
            self.pt += STEP

    def step(self, dt):
        for i in range(2):
            goal = self.target[i] * (0.1 if self.stalled else 1.0)
            accel = 20000.0 if self.braking[i] else 3000.0
            delta = max(-accel * dt, min(accel * dt, goal - self.speed[i]))
            self.speed[i] += delta
        vl = self.speed[0] * math.pi * self.diam[0] / 360.0
        vr = self.speed[1] * math.pi * self.diam[1] / 360.0
        v, w = (vl + vr) / 2.0, (vr - vl) / self.track
        nx, ny = self.x + v * math.cos(self.th) * dt, self.y + v * math.sin(self.th) * dt
        nth = self.th + w * dt
        old_th = self.th
        moving = abs(v) > 1e-6 or abs(w) > 1e-9
        if moving and not self.world.collides(nx, ny, nth):
            self.x, self.y, self.th = nx, ny, nth
            self.stalled = False
        elif moving:
            # Blocked. Pushing within 20 deg of square on, or wedged: stall.
            # Otherwise slide along the wall by the part of the motion along
            # it, less friction, while the wheels turn at full speed (so
            # odometry runs ahead), or scrape round in place.
            self.blocked_steps += 1
            dx, dy = nx - self.x, ny - self.y
            tries = []
            contact = self.world.contact_tangent(nx, ny, nth)
            d = math.hypot(dx, dy)
            if contact is not None and d > 1e-6:
                t, (cx, cy) = contact
                along = dx * t[0] + dy * t[1]
                if abs(along) > 0.35 * d:
                    # The wheels cannot slip sideways freely: the caught
                    # corner twists the body, pivoting on it, toward lining
                    # up with the wall.
                    wall = math.atan2(t[1], t[0]) if along > 0 else math.atan2(-t[1], -t[0])
                    off = (wall - self.th + math.pi) % (2 * math.pi) - math.pi
                    a = max(-TWIST * dt, min(TWIST * dt, off))
                    ca, sa = math.cos(a), math.sin(a)
                    rx, ry = self.x - cx, self.y - cy
                    sx, sy = 0.8 * along * t[0], 0.8 * along * t[1]
                    tries.append(("slide", cx + rx * ca - ry * sa + sx,
                                  cy + rx * sa + ry * ca + sy, self.th + a))
                    tries.append(("slide", self.x + sx, self.y + sy, self.th))
            if abs(w) * dt > 1e-6:
                tries.append(("turn", self.x, self.y, nth))
                if contact is not None:
                    # A corner pressed on the wall while turning shoves the
                    # body off it a little: the tires slip sideways.
                    t, (cx, cy) = contact
                    nx_, ny_ = -t[1], t[0]
                    if nx_ * (self.x - cx) + ny_ * (self.y - cy) < 0:
                        nx_, ny_ = -nx_, -ny_
                    push = PUSH_OFF * dt
                    tries.append(("turn", self.x + nx_ * push, self.y + ny_ * push, nth))
            for kind, tx, ty, tth in tries:
                if not self.world.collides(tx, ty, tth):
                    self.x, self.y, self.th = tx, ty, tth
                    # Turning in place while the drive forward is blocked
                    # still stalls the wheels.
                    self.stalled = kind == "turn" and abs(v) > 20.0
                    break
            else:
                self.stalled = True
        for i in range(2):
            self.angle[i] += self.speed[i] * dt
        self.gyro_value += -math.degrees(self.th - old_th) * self.gyro_scale + self.gyro_drift * dt
        if self.pt >= self.next_sample:
            self.next_sample = self.pt + 100.0
            px, py = self.tip()
            est = None
            if self.robot is not None and getattr(self.robot, "running", False):
                est = (self.robot.x, self.robot.y)
            self.path.append((self.pt, self.x, self.y, px, py, est))
            if self.pt > self.press_at and py > 0:
                out = max(-px, px - WORKSPACE, -py, py - WORKSPACE)
                if out > 0 and self.world.clearance(px, py) > 300:
                    self.violation = max(self.violation, out)
                c, s = math.cos(self.th), math.sin(self.th)
                for bx, by in [p for part in self.world.body for p in part]:
                    wx, wy = self.x + bx * c - by * s, self.y + bx * s + by * c
                    out = max(-wx, wx - WORKSPACE, -wy, wy - WORKSPACE)
                    if out > 0 and self.world.clearance(px, py) > 300:
                        self.body_violation = max(self.body_violation, out)


SIM = None


class Port:
    A, B, C, D, S1, S2, S3, S4 = "A", "B", "C", "D", "S1", "S2", "S3", "S4"


class Stop:
    COAST, BRAKE, HOLD = "coast", "brake", "hold"


class Direction:
    CLOCKWISE, COUNTERCLOCKWISE = 1, -1


class Button:
    CENTER, DOWN, UP, LEFT, RIGHT = "center", "down", "up", "left", "right"


class SoundFile:
    OUCH = "ouch"


class Motor:
    def __init__(self, port, direction=Direction.CLOCKWISE):
        self.i = 0 if port == Port.B else 1

    def angle(self):
        SIM.advance(0.2)
        return int(SIM.angle[self.i])

    def speed(self):
        SIM.advance(0.2)
        return int(SIM.speed[self.i])

    def run(self, speed):
        SIM.advance(0.3)
        SIM.target[self.i] = float(speed)
        SIM.braking[self.i] = False
        if abs(speed) > 1:
            SIM.last_move = SIM.t

    def stop(self, stop_type=Stop.COAST):
        SIM.advance(0.3)
        SIM.target[self.i] = 0.0
        SIM.braking[self.i] = True


class TouchSensor:
    def __init__(self, port):
        pass

    def pressed(self):
        SIM.advance(0.3)
        return SIM.world.touched(SIM.x, SIM.y, SIM.th)


class GyroSensor:
    def __init__(self, port):
        pass

    def angle(self):
        SIM.advance(0.8)
        return int(round(SIM.gyro_value - SIM.gyro_zero))

    def reset_angle(self, angle):
        SIM.gyro_zero = SIM.gyro_value - angle

    def speed(self):
        SIM.advance(0.8)
        return 0


class UltrasonicSensor:
    def __init__(self, port):
        pass

    def distance(self):
        SIM.advance(2.0)
        return SIM.world.sonar(SIM.x, SIM.y, SIM.th, SIM.rng)


class StopWatch:
    def __init__(self):
        self.start = SIM.t

    def time(self):
        return int(SIM.t - self.start)

    def reset(self):
        self.start = SIM.t


def wait(ms):
    SIM.advance(float(ms))


def sim_print(*args):
    SIM.advance(0.5)
    SIM.lines.append(" ".join(str(a) for a in args))


class Sound:
    def beep(self, *args, **kwargs):
        SIM.beeps.append((SIM.t, SIM.tip()))
        SIM.advance(100.0)

    def file(self, *args, **kwargs):
        SIM.advance(1000.0)


def buttons():
    SIM.advance(0.8)
    return [Button.CENTER] if SIM.press_at <= SIM.t < SIM.press_at + 200 else []


def install():
    pybricks = types.ModuleType("pybricks")
    ev3brick = types.ModuleType("pybricks.ev3brick")
    ev3brick.buttons = buttons
    ev3brick.sound = Sound()
    devices = types.ModuleType("pybricks.ev3devices")
    for cls in (Motor, TouchSensor, GyroSensor, UltrasonicSensor):
        setattr(devices, cls.__name__, cls)
    params = types.ModuleType("pybricks.parameters")
    for cls in (Port, Stop, Direction, Button, SoundFile):
        setattr(params, cls.__name__, cls)
    tools = types.ModuleType("pybricks.tools")
    tools.wait, tools.StopWatch, tools.print = wait, StopWatch, sim_print
    pybricks.ev3brick = ev3brick
    sys.modules.update({"pybricks": pybricks, "pybricks.ev3brick": ev3brick,
                        "pybricks.ev3devices": devices, "pybricks.parameters": params,
                        "pybricks.tools": tools})


# ------------------------------ layouts -------------------------------
def box(cx, cy, w, h, angle=0.0):
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    pts = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in pts]


def circle(cx, cy, r, n=24):
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
            for k in range(n)]


def place(shape, cx, cy, angle):
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    return [(cx + x * c - y * s, cy + x * s + y * c) for x, y in shape]


def u_shape(width, depth, t):
    # Opening toward -y (local), base at +y.
    w, d = width / 2.0, depth
    return [(-w, 0), (-w + t, 0), (-w + t, d - t), (w - t, d - t), (w - t, 0), (w, 0),
            (w, d), (-w, d)]


def l_shape(a, b, t):
    return [(0, 0), (a, 0), (a, t), (t, t), (t, b), (0, b)]


def m(points):
    return [(x * 1000.0, y * 1000.0) for x, y in points]


# Unit vectors along and across the line from the start to the goal.
ML = math.atan2(GOAL[1] - START[1], GOAL[0] - START[0])
MDEG = math.degrees(ML)


def on_line(f, side=0.0):
    # Point f of the way from the start to the goal, `side` mm to its left.
    x = START[0] + f * (GOAL[0] - START[0]) - side * math.sin(ML)
    y = START[1] + f * (GOAL[1] - START[1]) + side * math.cos(ML)
    return x, y


SIDE = [box(3500, 600, 300, 300), circle(800, 3300, 200)]

LAYOUTS = {
    "open": SIDE,
    # Obstacles from the assignment figure, read off it in metres.
    "figure": [
        m([(2.52, 3.68), (2.93, 3.68), (3.49, 3.42), (3.37, 3.02), (2.56, 3.40)]),
        m([(0.56, 3.17), (1.06, 3.17), (1.38, 2.71), (1.10, 2.04), (0.50, 2.15), (0.43, 2.51),
           (0.67, 2.47), (0.78, 2.81)]),
        m([(1.44, 1.79), (1.91, 1.39), (1.81, 0.88), (1.18, 0.73), (1.25, 1.41)]),
        m([(2.19, 0.83), (2.91, 1.14), (3.43, 1.27), (3.78, 1.10), (3.38, 0.65), (3.73, 0.31),
           (2.52, 0.29), (2.95, 0.61)]),
    ],
    # 2 m wall square across the line, its middle on it.
    "wall": [box(*on_line(0.45), 2000, 100, MDEG + 90)] + SIDE,
    # Same wall, hit near its right-hand end (the side the robot turns to).
    "wall_right": [box(*on_line(0.45, 700), 2000, 100, MDEG + 90)] + SIDE,
    # Same wall, hit near its left-hand end: the long way round.
    "wall_left": [box(*on_line(0.45, -700), 2000, 100, MDEG + 90)] + SIDE,
    # Pocket opening toward the start, on the line.
    "u": [place(u_shape(900, 800, 100), *on_line(0.45), MDEG - 90)] + SIDE,
    "circle": [circle(*on_line(0.5), 250)] + SIDE,
    "two_boxes": [box(*on_line(0.3), 500, 500, MDEG), box(*on_line(0.62), 600, 400, MDEG + 20)],
    # Wall across the line, near face 200 mm before the goal: contact in the region.
    "goal_wall_near": [box(*on_line(1.0 - 150 / 3201.6), 600, 100, MDEG + 90)] + SIDE,
    # Near face 450 mm before the goal: contact outside the region.
    "goal_wall_far": [box(*on_line(1.0 - 400 / 3201.6), 600, 100, MDEG + 90)] + SIDE,
    # Box just past the goal, on the line: in the region but not in the way.
    "goal_box_behind": [box(*on_line(1.0 + 450 / 3201.6), 600, 600, MDEG)] + SIDE,
    # Long wall running east almost to the workspace edge.
    "east_wall": [box(2600, 1100, 2400, 100)] + [circle(800, 3300, 200)],
    "zigzag": [[(1100, 1500), (1500, 1250), (1800, 1550), (2100, 1250), (2400, 1550),
                (2400, 1650), (2100, 1350), (1800, 1650), (1500, 1350), (1100, 1600)]],
    "l_pocket": [place(l_shape(1100, 1100, 120), *on_line(0.55, -250), MDEG + 135)] + SIDE,
    "slant": [box(*on_line(0.45), 1500, 120, MDEG + 50)] + SIDE,
}


def random_shape(rng):
    kind = rng.choice(["box", "box", "convex", "circle", "L", "U", "tri", "thin", "star"])
    if kind == "box":
        pts = box(0, 0, rng.uniform(150, 1000), rng.uniform(150, 1000))
    elif kind == "thin":
        pts = box(0, 0, rng.uniform(600, 1800), rng.uniform(60, 120))
    elif kind == "circle":
        pts = circle(0, 0, rng.uniform(80, 600))
    elif kind == "convex":
        n = rng.randint(5, 8)
        angs = sorted(rng.uniform(0, 2 * math.pi) for _ in range(n))
        r = rng.uniform(200, 600)
        pts = [(r * rng.uniform(0.75, 1.0) * math.cos(a), r * rng.uniform(0.75, 1.0) * math.sin(a))
               for a in angs]
    elif kind == "tri":
        pts = [(0, 0), (rng.uniform(300, 1000), 0), (rng.uniform(0, 800), rng.uniform(300, 900))]
    elif kind == "L":
        pts = l_shape(rng.uniform(400, 1100), rng.uniform(400, 1100), rng.uniform(80, 200))
    elif kind == "U":
        pts = u_shape(rng.uniform(600, 1000), rng.uniform(400, 800), rng.uniform(80, 150))
    else:
        n = rng.randint(4, 6)
        r = rng.uniform(250, 600)
        pts = [((r if k % 2 == 0 else r * 0.5) * math.cos(math.pi * k / n),
                (r if k % 2 == 0 else r * 0.5) * math.sin(math.pi * k / n)) for k in range(2 * n)]
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    pts = [(x - cx, y - cy) for x, y in pts]
    per = perimeter(pts)
    if per > 5000:
        pts = [(x * 4800 / per, y * 4800 / per) for x, y in pts]
    elif per < 500:
        pts = [(x * 600 / per, y * 600 / per) for x, y in pts]
    return kind, pts


def random_layout(seed):
    rng = random.Random(1000 + seed)
    polys, kinds = [], []
    for _ in range(rng.randint(2, 6)):
        for attempt in range(300):
            kind, shape = random_shape(rng)
            if rng.random() < 0.6:
                cx, cy = on_line(rng.uniform(0.2, 0.85), rng.gauss(0, 350))
            else:
                cx, cy = rng.uniform(300, 3700), rng.uniform(300, 3700)
            pts = place(shape, cx, cy, rng.uniform(0, 360))
            if any(not (50 <= x <= 3950 and 50 <= y <= 3950) for x, y in pts):
                continue
            o = Obstacle(pts)
            if o.distance(*GOAL) < 80 or o.distance(*START) < 400:
                continue
            if any(poly_distance(o, Obstacle(q)) < 500 for q in polys):
                continue
            polys.append(pts)
            kinds.append(kind)
            break
    return polys, kinds


# ------------------------------- runner -------------------------------
def run(name, polys, seed=1, overrides=None, plot=False, bumper=False):
    global SIM
    SIM = Sim(polys, seed, bumper)
    install()
    source = open(MAIN).read()
    folder = os.path.join(OUT, name)
    os.makedirs(folder, exist_ok=True)
    log = os.path.join(folder, "run_log.txt")
    if os.path.exists(log):
        os.remove(log)
    ns = {"__name__": "lab4", "__file__": os.path.join(folder, "main.py")}
    exec(compile(source, MAIN, "exec"), ns)
    for key, value in (overrides or {}).items():
        ns[key] = value
    original = ns["Robot"].__init__

    def init(self, *a):
        original(self, *a)
        SIM.robot = self
    ns["Robot"].__init__ = init
    error = None
    try:
        ns["main"]()
    except SimEnd as e:
        error = str(e)
    px, py = SIM.tip()
    result = [l for l in SIM.lines if l.startswith("RESULT") or l.startswith("STOPPED")]
    stop_t = SIM.last_move if SIM.last_move is not None else SIM.t
    r = {
        "name": name,
        "error_mm": math.dist((px, py), GOAL),
        "time_s": (stop_t - SIM.press_at) / 1000.0,
        "beeped": bool(SIM.beeps),
        "contacts": sum(1 for l in SIM.lines if l.startswith("CONTACT") or l.startswith("BUMP")),
        "leaves": sum(1 for l in SIM.lines if l.startswith("Leaving")),
        "violation": SIM.violation,
        "body_violation": SIM.body_violation,
        "scrape_s": SIM.blocked_steps * STEP / 1000.0,
        "how": (result[-1] if result else (error or "no result"))[:90],
        "est_error": None,
    }
    if SIM.robot is not None:
        ex, ey = SIM.robot.point()
        r["est_error"] = math.dist((ex, ey), (px, py))
    with open(os.path.join(folder, "console.txt"), "w") as f:
        f.write("\n".join(SIM.lines) + "\n")
    if plot:
        draw(name, polys, SIM.path, os.path.join(OUT, name + ".png"), r)
    return r


def draw(name, polys, path, file, r):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.add_patch(plt.Rectangle((0, 0), WORKSPACE, WORKSPACE, fill=False, color="k"))
    for p in polys:
        ax.add_patch(plt.Polygon(p, color="0.3"))
    ax.add_patch(plt.Circle(GOAL, 300, fill=False, color="g"))
    ax.plot(*GOAL, "g+")
    ax.plot(*START, "ko")
    ax.plot([q[3] for q in path], [q[4] for q in path], "b-", lw=1, label="true measuring point")
    est = [q[5] for q in path if q[5] is not None]
    if est:
        ax.plot([e[0] for e in est], [e[1] for e in est], "r:", lw=1, label="estimated axle")
    ax.set_xlim(-300, 4300)
    ax.set_ylim(-300, 4300)
    ax.set_aspect("equal")
    ax.legend(loc="upper left", fontsize=7)
    ax.set_title("%s: %.0f mm from goal, %.1f s, %d contacts" % (
        name, r["error_mm"], r["time_s"], r["contacts"]), fontsize=9)
    fig.savefig(file, dpi=90)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--random", type=int, default=0)
    parser.add_argument("--only", default="")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--bumper", action="store_true", help="full-width front bumper")
    parser.add_argument("--seeds", type=int, default=1, help="noise seeds per layout")
    parser.add_argument("--set", action="append", default=[], help="NAME=value")
    args = parser.parse_args()
    overrides = {}
    for item in args.set:
        key, value = item.split("=", 1)
        overrides[key] = eval(value)
    jobs = [(k, v) for k, v in LAYOUTS.items() if not args.only or k in args.only.split(",")]
    for i in range(args.random):
        polys, kinds = random_layout(i)
        name = "r%02d" % i
        if not args.only or name in args.only.split(","):
            jobs.append((name, polys))
    rows = []
    print("%-14s %7s %6s %5s %4s %4s %6s %6s %6s  %s" % (
        "layout", "goal_mm", "time_s", "beep", "hit", "left", "out_mm", "est_mm", "scrape", "how"))
    for name, polys in jobs:
        for seed in range(1, args.seeds + 1):
            r = run(name if args.seeds == 1 else "%s_s%d" % (name, seed), polys, seed,
                    overrides, args.plot, args.bumper)
            rows.append(r)
            print("%-14s %7.0f %6.1f %5s %4d %4d %6.0f %6.0f %6.1f  %s" % (
                r["name"], r["error_mm"], r["time_s"], "yes" if r["beeped"] else "no",
                r["contacts"], r["leaves"], max(r["violation"], 0),
                r["est_error"] or 0, r["scrape_s"], r["how"]), flush=True)
    ok = [r for r in rows if r["error_mm"] <= 300]
    print("\n%d/%d stopped in the goal region; median error %.0f mm; median time %.1f s; "
          "worst time %.1f s; workspace exits %d" % (
              len(ok), len(rows), sorted(r["error_mm"] for r in rows)[len(rows) // 2],
              sorted(r["time_s"] for r in rows)[len(rows) // 2],
              max(r["time_s"] for r in rows),
              sum(1 for r in rows if r["violation"] > 0)))


if __name__ == "__main__":
    main()
