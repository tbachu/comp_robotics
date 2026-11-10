# The simulated world behind the fake pybricks modules: a 2D kinematic robot,
# polygon obstacles, a cone sonar, a touch tip and an integer gyro.
# Units: mm, ms, degrees. Heading psi is clockwise from +Y, like main.py's yaw.
import math

D = 52.6                      # true wheel diameter (same as main.py: no scale error)
AXLE = 120.0
TIP_F, TIP_L = 106.0, 6.0     # touch tip (measuring point), forward / left of the axle
SON_F, SON_L = 28.0, 93.0     # sonar face
SONAR_BIAS = 27.0             # the real sensor reads this much long; main.py subtracts it

# Sonar beam models: (half-angle of the beam, steepest echo angle that still returns).
SONAR_MODELS = {"narrow": (8.0, 15.0), "medium": (10.0, 25.0), "wide": (15.0, 40.0)}


class World:
    def __init__(self):
        self.reset([])

    def reset(self, polygons, slip=None, sonar="medium", press_at=14000.0, overhead=10.0):
        # polygons: obstacle outlines. slip(world) -> k: the encoders count k
        # times the real motion (k > 1 makes the estimate lag, k < 1 lead).
        # press_at: sim ms when CENTER is pressed. overhead: ms added to every
        # wait(), standing in for the time the real loop spends computing.
        self.t = 0.0
        self.X, self.Y, self.psi = 2006.0, 394.0, 0.0  # axle: tip starts at (2000, 500)
        self.speed = {"B": 0.0, "C": 0.0}
        self.enc = {"B": 0.0, "C": 0.0}
        self.polygons = polygons
        self.slip = slip
        self.cone, self.specular = SONAR_MODELS[sonar]
        self.gyro_zero = 0.0
        self.press_at = press_at
        self.overhead = overhead
        self.trace = []            # (t_ms, tip x, tip y, heading) every 50 ms
        self.next_trace = 0.0

    # --- geometry -------------------------------------------------------
    def point(self, f, l, X=None, Y=None, psi=None):
        X = self.X if X is None else X
        Y = self.Y if Y is None else Y
        p = math.radians(self.psi if psi is None else psi)
        return (X + f * math.sin(p) - l * math.cos(p),
                Y + f * math.cos(p) + l * math.sin(p))

    def inside(self, x, y):
        for poly in self.polygons:
            c = False
            n = len(poly)
            for i in range(n):
                x1, y1 = poly[i]
                x2, y2 = poly[(i + 1) % n]
                if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                    c = not c
            if c:
                return True
        return False

    def edge_dist(self, x, y):
        return distance_to(x, y, self.polygons)

    # --- sensors --------------------------------------------------------
    def tip(self):
        return self.point(TIP_F, TIP_L)

    def touch(self):
        return self.inside(*self.tip())

    def sonar(self):
        # Nearest echo within the beam, from surfaces hit at a shallow enough angle.
        sx, sy = self.point(SON_F, SON_L)
        best = None
        a = -self.cone
        while a <= self.cone + 1e-9:
            ang = math.radians(self.psi - 90.0 + a)  # the beam points left
            dx, dy = math.sin(ang), math.cos(ang)
            for poly in self.polygons:
                n = len(poly)
                for i in range(n):
                    x1, y1 = poly[i]
                    x2, y2 = poly[(i + 1) % n]
                    ex, ey = x2 - x1, y2 - y1
                    den = dx * ey - dy * ex
                    if abs(den) < 1e-9:
                        continue
                    t = ((x1 - sx) * ey - (y1 - sy) * ex) / den
                    u = ((x1 - sx) * dy - (y1 - sy) * dx) / den
                    if t <= 0 or u < 0 or u > 1:
                        continue
                    incidence = math.degrees(math.acos(min(1.0, abs(den) / math.hypot(ex, ey))))
                    if incidence > self.specular:
                        continue
                    if best is None or t < best:
                        best = t
            a += 1.5
        if best is None or best + SONAR_BIAS > 2550:
            return 2550  # no echo
        return int(round(best + SONAR_BIAS))

    # --- physics --------------------------------------------------------
    def step(self, dt):
        s = dt / 1000.0
        lv = self.speed["B"] * math.pi * D / 360.0
        rv = self.speed["C"] * math.pi * D / 360.0
        self.enc["B"] += self.speed["B"] * s
        self.enc["C"] += self.speed["C"] * s
        v = (lv + rv) / 2.0
        dpsi = math.degrees((lv - rv) / AXLE * s)
        k = self.slip(self) if self.slip else 1.0
        ds = v * s / k
        mid = math.radians(self.psi + dpsi / 2.0)
        nx, ny = self.X + ds * math.sin(mid), self.Y + ds * math.cos(mid)
        npsi = self.psi + dpsi
        tx, ty = self.point(TIP_F, TIP_L, nx, ny, npsi)
        if self.inside(tx, ty) and self.edge_dist(tx, ty) > 2.0:
            nx, ny = self.X, self.Y  # the wall stops the tip (the body passes through walls)
        self.X, self.Y, self.psi = nx, ny, npsi

    def advance(self, ms):
        ms = ms + self.overhead
        while ms > 0:
            dt = min(5.0, ms)
            self.step(dt)
            self.t += dt
            ms -= dt
            if self.t >= self.next_trace:
                tx, ty = self.tip()
                self.trace.append((int(self.t), tx, ty, self.psi))
                self.next_trace += 50.0


def distance_to(x, y, polygons):
    # Distance from a point to the nearest obstacle edge.
    best = 1e9
    for poly in polygons:
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            dx, dy = x2 - x1, y2 - y1
            u = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)))
            best = min(best, math.hypot(x - x1 - u * dx, y - y1 - u * dy))
    return best


WORLD = World()
