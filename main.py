#!/usr/bin/env pybricks-micropython
# COMP 581, Fall 2026 - Lab 3: Obstacle Tracing
# Tanish Bachu: 730739311
# Minh Nguyen: 730766544


from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor, TouchSensor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, SoundFile
from pybricks.tools import wait, StopWatch, print as console_print

try:
    import os
except ImportError:
    import uos as os


# ------------------------ MEASURE / CALIBRATE -------------------------
MODE = "RUN"             # CHECK, STRAIGHT, TURN, or RUN
LEFT_DIRECTION = Direction.CLOCKWISE
RIGHT_DIRECTION = Direction.CLOCKWISE
GYRO_SIGN = 1                  # Set -1 if a physical RIGHT turn reads negative.
LEFT_DIAMETER = 52.6           # Effective rolling diameter: STRAIGHT 500 -> 525 mm
RIGHT_DIAMETER = 52.6          # with 50.1, so 50.1 * 525 / 500.
AXLE_TRACK = 120.0             # Center-to-center tire contact track spacing.
POINT_FORWARD = 106.0          # a: axle midpoint -> measuring point (tip of the
                               # 3-stud rod on the touch button: 82 + 24 mm).
POINT_LEFT = 6.0               # c: measuring point is 6 mm LEFT of centerline.
SONAR_LEFT = 93.0              # b: centerline -> left sonar face, left positive.
SONAR_FORWARD = 28.0           # Axle midpoint -> sonar face, forward positive.
SONAR_OFFSET = 27.0            # Sonar reads this much over the true face distance
                               # (CHECK: 110 mm -> 135, 200 mm -> 229).
# Reported radius "9.3" assumed cm. However, sqrt(93^2+28^2)=97.1 mm
# already reaches the sonar face midpoint, before including the sensor edges.
# Use 120 mm provisionally; measure the complete footprint before running.
BODY_RADIUS = 120.0

WORKSPACE = 4000.0             # mm; square workspace side.
START_X = 2000.0               # Measuring point starts here, facing +Y.
START_Y = 500.0
TARGET_POINT_CLEARANCE = 200.0 # Desired measuring-point distance from obstacle
                               # (hit point is 200 mm; the limit is 300 mm).
INITIAL_AXLE_CLEARANCE = 200.0 # Backup = 200 - 106 = 94 mm before right turn.
APPROACH_LIMIT = 1300.0        # mm; front wall is 400..1200 mm from start.

APPROACH_SPEED = 90.0          # mm/s; final contact approach, use a secure touch mount.
FAST_APPROACH_SPEED = 150.0    # mm/s; open-floor part of the approach.
FAST_APPROACH_DISTANCE = 200.0 # mm; wall is >= 400 mm away, so this leaves margin.
RAMP_ACCEL = 400.0             # mm/s^2; gentle start to avoid wheel slip.
FOLLOW_SPEED = 150.0           # 140/90/80 with 5 bumps reached the wall-end
SLOW_SPEED = 100.0             # search too late to finish it before returning.
ORBIT_SPEED = 85.0             # mm/s while circling a convex corner.
RETURN_SPEED = 150.0
MAX_YAW_RATE = 60.0            # deg/s while following.
TURN_RATE = 75.0               # deg/s; max rate for in-place turns.
DISTANCE_KP = 0.32             # (deg/s) per mm of range error.
DISTANCE_KD = 0.10             # (deg/s) per (mm/s) of filtered range change.
HEADING_KP = 2.0               # (deg/s) per degree of heading error.
PERIOD = 50                    # ms; do not poll sonar in a tight busy loop.
RUN_LIMIT = 117000             # ms from CENTER press; task limit is 120 s.
RETURN_RESERVE = 12000         # ms kept for the return (one took 11 s): past this, skip the
                               # wall-end landmark and head for the start.
GAP_BRIDGE = 40.0              # mm to hold heading through a sonar dropout
                               # before circling the last wall point seen.
GAP_JUMP = 60.0                # mm; a sudden outward jump this big means the wall
                               # fell away (convex corner) or a bad echo.
ORBIT_RADIUS = 215.0           # mm, axle -> corner; puts the measuring point
                               # ~240 mm from the corner and the sonar ~122 mm.
                               # 190 often ended corners with the side scraping.
ORBIT_KP = 0.005               # rad of heading per mm of orbit radius error.
ORBIT_MAX_TURN = 330.0         # deg circled with no echo before giving up.
REACQUIRE_RANGE = 200.0        # mm; while circling, only nearer echoes are the wall.
REACQUIRE_COUNT = 2            # consecutive echoes needed to stop circling.
CORNER_TRUST = 80.0            # mm past target; farther echoes do not move the corner.
ERROR_LIMIT = 60.0             # mm; caps steering at about 19 deg/s from range error.
BUMP_BACKUP = 100.0            # mm to reverse after hitting the wall while following;
                               # 60 let the body scrape during the turn after.
BUMP_SPEED = 150.0             # mm/s for that reverse.
BUMP_TURN = 90.0               # deg to turn right after a bump before resuming;
                               # 60 often bumped the same corner 2-3 times
                               # (~4 s each), using up the wall-end search time.
BUMP_BRIDGE = 120.0            # mm to hold heading after a bump before circling.
APPROACH_ANGLE = 25.0          # deg; most the heading may swing from the last
                               # heading that was parallel to the wall.
LOOP_MIN_TRAVEL = 1000.0       # mm followed before the loop may close (perimeter >= 1.5 m).
LOOP_MIN_TURN = 315.0          # deg of net left turn before the loop may close: heading
                               # south-east or later. 270 let a time-out return start
                               # beside the west face and clip the obstacle's corner.
LOOP_Y_WINDOW = 400.0          # mm; how far from the hit point line odometry may be.
# The final pass along the front wall is recognised by heading, not by X:
# X drifted 300-780 mm per lap, and waiting for it to reach the hit point
# let the robot round the wall's far end and keep tracing.
FRONT_HEADING = 35.0           # deg from a full lap's east that counts as the front
                               # wall (a lap with 4 bumps ended with the gyro ~25 deg off).
WALL_RUN_MIN = 100.0           # mm of wall followed since the last corner or bump
                               # before losing it or bumping counts as the wall's end
                               # (not a flicker while rounding its west corner).
PAST_TURN = 60.0               # deg turned left past a full lap: rounding the far end.
SEARCH_MARGIN = 800.0          # mm past the expected wall end before giving up;
                               # odometry was once 300+ mm ahead, and +300 gave up
                               # just short of the real wall end.
MAX_X_FIX = 1000.0             # mm; worst drift seen was 778 mm, so a bigger X fix
                               # means the wrong corner matched.
FRONT_WALL_CORRECTION = True   # Re-zero yaw and Y on the front wall at the end.
LOG_EVERY = 500                # ms; print telemetry to the wired console.

PI = 3.141592653589793
RAD = PI / 180.0
EAST = 90.0                    # Yaw while following the front wall at the start.


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap180(degrees):
    return (degrees + 180.0) % 360.0 - 180.0


def sine(x):
    # Range-reduced Taylor polynomial: no extra module imports.
    x = (x + PI) % (2.0 * PI) - PI
    if x > PI / 2.0:
        x = PI - x
    elif x < -PI / 2.0:
        x = -PI - x
    x2 = x * x
    return x * (1.0 + x2 * (-1.0 / 6.0 + x2 * (1.0 / 120.0
           + x2 * (-1.0 / 5040.0 + x2 * (1.0 / 362880.0
           - x2 / 39916800.0)))))


def cosine(x):
    return sine(x + PI / 2.0)


def arctan2(y, x):
    # Octant-reduced polynomial, max error about 0.1 degree.
    if x == 0 and y == 0:
        return 0.0
    ax, ay = abs(x), abs(y)
    z = min(ax, ay) / max(ax, ay)
    a = PI / 4.0 * z - z * (z - 1.0) * (0.2447 + 0.0663 * z)
    if ay > ax:
        a = PI / 2.0 - a
    if x < 0:
        a = PI - a
    return -a if y < 0 else a


def brake_motor(motor):
    # v1 accepts stop(Stop.BRAKE); v2 also exposes brake().
    try:
        motor.stop(Stop.BRAKE)
    except TypeError:
        motor.brake()


# ------------------------------ RUN LOGS ------------------------------
# Each run saves runs/run_NNN.txt (everything printed) and runs/run_NNN.csv
# (one row per control tick) next to this file, so runs started from the
# brick's menu are saved too. Get them with fetch_runs.sh on the laptop.
try:
    LOG_DIR = (__file__.rsplit("/", 1)[0] + "/runs") if "/" in __file__ else "runs"
except NameError:
    LOG_DIR = "runs"


class RunLog:
    def __init__(self):
        self.text = self.csv = self.name = None

    def open(self):
        # Number runs, since the brick's clock is not reliable. Logging
        # problems only turn logging off; they never stop the robot.
        try:
            try:
                os.mkdir(LOG_DIR)
            except OSError:
                pass  # Already there.
            number = 1
            for name in os.listdir(LOG_DIR):
                if name.startswith("run_"):
                    try:
                        number = max(number, int(name[4:7]) + 1)
                    except ValueError:
                        pass
            self.name = LOG_DIR + "/run_%03d" % number
            self.text = open(self.name + ".txt", "w")
            self.csv = open(self.name + ".csv", "w")
            self.csv.write("t_ms,phase,x,y,yaw,encoder_yaw,px,py,sonar\n")
        except Exception as error:
            self.close()
            console_print("Run log off:", error)

    def line(self, text):
        if self.text is not None:
            try:
                self.text.write(text + "\n")
                self.text.flush()
            except Exception:
                self.text = None

    def row(self, robot):
        # x, y: axle estimate; px, py: measuring point estimate (mm).
        if self.csv is None:
            return
        try:
            h = (90.0 - robot.old_yaw) * RAD
            px = robot.x + POINT_FORWARD * cosine(h) - POINT_LEFT * sine(h)
            py = robot.y + POINT_FORWARD * sine(h) + POINT_LEFT * cosine(h)
            sonar = "" if robot.sonar_mm is None else str(int(robot.sonar_mm))
            self.csv.write("%d,%s,%d,%d,%.1f,%.1f,%d,%d,%s\n" % (
                robot.clock.time(), robot.phase, int(robot.x), int(robot.y),
                robot.old_yaw, robot.encoder_yaw, int(px), int(py), sonar))
            self.csv.flush()
        except Exception:
            self.csv = None

    def close(self):
        for f in (self.text, self.csv):
            try:
                if f is not None:
                    f.close()
            except Exception:
                pass
        self.text = self.csv = None


LOG = RunLog()


def print(*args):
    # Console and the run's .txt file get the same lines.
    console_print(*args)
    LOG.line(" ".join([str(a) for a in args]))


class Robot:
    def __init__(self):
        self.left = Motor(Port.B, LEFT_DIRECTION)
        self.right = Motor(Port.C, RIGHT_DIRECTION)
        self.touch = TouchSensor(Port.S1)
        self.gyro = GyroSensor(Port.S2)
        self.sonar = UltrasonicSensor(Port.S4)
        self.drift = 0.0       # gyro deg/ms, measured in start()
        self.yaw_offset = 0.0  # deg, from front-wall correction
        self.clock = StopWatch()
        self.running = False
        self.phase = "START"   # for the run log
        self.sonar_mm = None   # last sonar reading, for the run log
        self.lap_end = "lap not finished"
        self.x_fix = None      # mm added to X at the front wall's end, if any
        self.stop()
        self.reset_pose()

    def stop(self):
        # Try both brakes even if one motor reports an error.
        try:
            brake_motor(self.left)
        finally:
            brake_motor(self.right)

    def yaw(self):
        # Subtract the steady gyro drift measured while waiting at the start.
        return (GYRO_SIGN * (self.gyro.angle() - self.drift * self.clock.time())
                + self.yaw_offset)

    def correct_yaw(self, delta):
        # Shift old_yaw too so odometry does not see a false rotation.
        self.yaw_offset += delta
        self.old_yaw += delta

    def reset_pose(self):
        self.old_left = self.left.angle()
        self.old_right = self.right.angle()
        self.old_yaw = self.yaw()
        # At start, measuring point is (START_X, START_Y), forward is +world Y.
        self.x = START_X + POINT_LEFT
        self.y = START_Y - POINT_FORWARD
        self.encoder_yaw = 0.0
        self.travel = 0.0
        self.check_time, self.check_encoder, self.check_yaw = self.clock.time(), 0.0, self.old_yaw

    def update(self):
        la, ra, yaw = self.left.angle(), self.right.angle(), self.yaw()
        dl = (la - self.old_left) * PI * LEFT_DIAMETER / 360.0
        dr = (ra - self.old_right) * PI * RIGHT_DIAMETER / 360.0
        ds = (dl + dr) / 2.0
        # Differential-drive forward kinematics, clockwise-positive yaw.
        self.encoder_yaw += (dl - dr) / AXLE_TRACK / RAD
        dyaw = yaw - self.old_yaw
        heading_mid = (90.0 - (yaw + self.old_yaw) / 2.0) * RAD
        self.x += ds * cosine(heading_mid)
        self.y += ds * sine(heading_mid)
        self.travel += ds
        self.old_left, self.old_right, self.old_yaw = la, ra, yaw
        return ds, dyaw

    def drive(self, speed, yaw_rate):
        # Inverse differential-drive kinematics, wheel mm/s -> motor deg/s.
        differential = yaw_rate * RAD * AXLE_TRACK / 2.0
        self.left.run((speed + differential) * 360.0 / (PI * LEFT_DIAMETER))
        self.right.run((speed - differential) * 360.0 / (PI * RIGHT_DIAMETER))

    def guard(self):
        if Button.DOWN in brick.buttons():
            raise RuntimeError("User abort")
        if self.running and self.clock.time() >= RUN_LIMIT:
            raise RuntimeError("117-second run limit")
        # Odometry-based workspace check; not a replacement for physical tests.
        if not (BODY_RADIUS <= self.x <= WORKSPACE - BODY_RADIUS
                and BODY_RADIUS <= self.y <= WORKSPACE - BODY_RADIUS):
            raise RuntimeError("Estimated workspace boundary")

    def tick(self):
        self.update()
        LOG.row(self)
        self.guard()
        # Every second, compare gyro and encoder rotation: a stuck gyro
        # makes the controllers spin the robot with no sign of turning.
        now = self.clock.time()
        if now - self.check_time >= 1000:
            turned = self.encoder_yaw - self.check_encoder
            seen = self.yaw() - self.check_yaw
            if self.running and abs(turned) > 40 and abs(seen) < 0.25 * abs(turned):
                # Also fires when the body is pinned and the wheels slip.
                raise RuntimeError("Wheels turning but gyro still: robot stuck, or check S2")
            self.check_time, self.check_encoder, self.check_yaw = now, self.encoder_yaw, self.yaw()

    def range_mm(self):
        value = self.sonar.distance()
        # Reject no-echo codes (often 2550 mm) and implausible close echoes.
        self.sonar_mm = None
        if 30 <= value <= 600:
            self.sonar_mm = float(value) - SONAR_OFFSET
        return self.sonar_mm

    def set_phase(self, phase):
        self.phase, self.sonar_mm = phase, None

    def finish(self, *message):
        # End the lap: stop, say why, and keep it for the RESULT line.
        self.stop()
        print(*message)
        self.lap_end = " ".join([str(m) for m in message])

    def fix_x(self, offset, label):
        # The front wall's far end has the same X on both passes, so it
        # resets the X drift. A fix bigger than any lap has drifted means
        # the wrong corner matched: keep odometry instead.
        if abs(offset) <= MAX_X_FIX:
            self.x += offset
            self.x_fix = offset
            self.finish(label, "X off by", int(offset), "mm")
        else:
            self.finish(label, "X off by", int(offset),
                        "mm is too far; stopping by odometry")

    def result(self, outcome):
        try:
            px, py = self.body_point(POINT_FORWARD, POINT_LEFT)
            where = "%d %d" % (int(px), int(py))
        except Exception:
            where = "unknown"
        print("RESULT:", outcome, "at", self.clock.time(), "ms |", self.lap_end,
              "| X fix", "none" if self.x_fix is None else int(self.x_fix),
              "| measuring point estimate", where,
              "| start", int(START_X), int(START_Y))

    def body_point(self, forward, left):
        h = (90.0 - self.yaw()) * RAD
        return (self.x + forward * cosine(h) - left * sine(h),
                self.y + forward * sine(h) + left * cosine(h))

    def bearing(self, tx, ty):
        # Yaw that points the axle midpoint at world point (tx, ty).
        return 90.0 - arctan2(ty - self.y, tx - self.x) / RAD

    def start(self):
        print("Keep robot still. Checking gyro...")
        wait(1000)
        # Switching rate -> angle mode makes the gyro recalibrate, which
        # usually cures startup drift without replugging.
        # A steady drift is fine: start() measures it and yaw() subtracts it.
        previous = None
        for attempt in range(3):
            self.gyro.reset_angle(0)
            wait(2000)
            moved = self.gyro.angle()
            print("Gyro moved", moved, "deg in 2 s")
            if abs(moved) <= 6:
                break
            if (previous is not None and abs(moved - previous) <= 2
                    and abs(moved) <= 12):  # 6 deg/s; the hand-move check trips at 8
                print("Drift is steady; compensating for it")
                break
            previous = moved
            print("Gyro drifting, recalibrating... keep still")
            self.gyro.speed()
            wait(1500)
            self.gyro.angle()
            wait(500)
        else:
            raise RuntimeError("Gyro drifting over 3 deg/s: replug gyro while still")
        # Measure drift from now until CENTER is pressed (robot must be still).
        self.gyro.reset_angle(0)
        drift_timer = StopWatch()
        print("Ready. Press CENTER. DOWN aborts.")
        # Restart the drift window whenever the robot is moved by hand
        # (>= 2 deg in 250 ms is 8 deg/s, far above real drift).
        base_angle, base_time = self.gyro.angle(), drift_timer.time()
        sample_angle, sample_time = base_angle, base_time
        previous = (0, 0, 0, 0)
        # Do not interpret the button used to launch the program as task start.
        while Button.CENTER in brick.buttons():
            wait(20)
        while Button.CENTER not in brick.buttons():
            if Button.DOWN in brick.buttons():
                raise RuntimeError("User abort")
            now = drift_timer.time()
            if now - sample_time >= 250:
                angle = self.gyro.angle()
                if abs(angle - sample_angle) >= 2:
                    # Keep the window that just ended: reaching for CENTER
                    # can jolt the gyro right before the press.
                    previous = (base_angle, base_time, sample_angle, sample_time)
                    base_angle, base_time = angle, now
                sample_angle, sample_time = angle, now
            wait(20)
        end_angle, end_time = self.gyro.angle(), drift_timer.time()
        if end_time - base_time < 3000 and previous[3] - previous[1] > end_time - base_time:
            base_angle, base_time, end_angle, end_time = previous
        still = end_time - base_time
        # Under 3 s still, 1-degree gyro steps make the estimate meaningless.
        # A single 1-degree step is quantization, not drift: 1 deg over 4 s
        # was read as 0.24 deg/s and turned into ~23 deg of false yaw per lap.
        if still >= 3000 and abs(end_angle - base_angle) >= 2:
            self.drift = (end_angle - base_angle) / still  # deg per ms
        print("Gyro drift:", self.drift * 1000.0, "deg/s over", still, "ms still")
        self.clock.reset()  # Task timing starts on press, not release.
        self.running = True
        self.gyro.reset_angle(0)
        self.reset_pose()
        while Button.CENTER in brick.buttons():
            self.guard()
            wait(20)

    def straight(self, distance, speed, contact=False, fast_for=0.0):
        # fast_for: drive at FAST_APPROACH_SPEED for this many mm first,
        # ramping up from the start and down to `speed` near the end.
        origin, heading = self.travel, self.yaw()
        start = self.clock.time()
        timeout = abs(distance) / speed * 1000.0 + 4000.0
        direction = 1.0 if distance >= 0 else -1.0
        while True:
            self.tick()
            if contact and self.touch.pressed():
                break
            if not contact and abs(self.travel - origin) >= abs(distance):
                break
            if contact and self.travel - origin >= distance:
                raise RuntimeError("No front contact within approach limit")
            if self.clock.time() - start > timeout:
                raise RuntimeError("Straight motion stalled")
            if not contact and direction > 0 and self.touch.pressed():
                raise RuntimeError("Unexpected front contact")
            error = heading - self.yaw()
            command = speed
            if fast_for > 0:
                done = abs(self.travel - origin)
                ramp_up = speed + RAMP_ACCEL * (self.clock.time() - start) / 1000.0
                ramp_down = speed + (fast_for - done) * 1.5  # ~40 mm slowdown
                command = clamp(min(ramp_up, ramp_down), speed, FAST_APPROACH_SPEED)
            self.drive(direction * command, clamp(HEADING_KP * error, -18, 18))
            wait(PERIOD)
        self.stop()
        wait(100)
        self.tick()

    def turn(self, degrees):
        target, start = self.yaw() + degrees, self.clock.time()
        settled = 0
        while settled < 3:
            self.tick()
            error = target - self.yaw()
            if self.clock.time() - start > 6500:
                raise RuntimeError("Turn timeout: check gyro/motor directions")
            if abs(error) <= 1.5:
                self.stop()
                settled += 1
            else:
                settled = 0
                rate = clamp(2.0 * abs(error), 9, TURN_RATE)
                self.drive(0, rate if error > 0 else -rate)
            wait(PERIOD)
        self.stop()
        self.tick()

    def go_to(self, tx, ty, speed):
        # Face the target, then drive until the measuring point reaches it.
        # The 6 mm POINT_LEFT offset is ignored when aiming.
        self.turn(wrap180(self.bearing(tx, ty) - self.yaw()))
        start = self.clock.time()
        heading = self.yaw()
        while True:
            self.tick()
            px, py = self.body_point(POINT_FORWARD, POINT_LEFT)
            h = (90.0 - self.yaw()) * RAD
            remaining = (tx - px) * cosine(h) + (ty - py) * sine(h)
            if remaining <= 0:
                break
            if self.clock.time() - start > 20000:
                raise RuntimeError("Return motion stalled")
            if self.touch.pressed():
                raise RuntimeError("Unexpected front contact on return")
            if remaining > 150:
                heading = self.bearing(tx, ty)  # Freeze aim when close.
            command = clamp(remaining * 1.5, 30.0, speed)
            self.drive(command, clamp(HEADING_KP * wrap180(heading - self.yaw()), -18, 18))
            wait(PERIOD)
        self.stop()
        wait(100)
        self.tick()

    def orbit_rate(self, corner):
        # Circle `corner` counterclockwise (corner on the left) at ORBIT_RADIUS.
        dx, dy = corner[0] - self.x, corner[1] - self.y
        distance = (dx * dx + dy * dy) ** 0.5
        theta = (arctan2(dy, dx) - PI / 2.0
                 + clamp(ORBIT_KP * (distance - ORBIT_RADIUS), -0.8, 0.8))
        error = wrap180(90.0 - theta / RAD - self.yaw())
        feedforward = -ORBIT_SPEED / ORBIT_RADIUS / RAD  # Steady left turn.
        return clamp(feedforward + HEADING_KP * error, -MAX_YAW_RATE, MAX_YAW_RATE)

    def follow(self, wall_y):
        # Keep the obstacle on the left until we come back to the hit point.
        target = TARGET_POINT_CLEARANCE + POINT_LEFT - SONAR_LEFT
        hit_x, hit_y = START_X, wall_y - TARGET_POINT_CLEARANCE
        final_yaw = EAST - 360.0  # One full counterclockwise loop.
        filtered = None
        derivative = 0.0
        valid_count = 0
        gap_travel = None   # travel when the current dropout started
        gap_heading = 0.0   # heading held while bridging a dropout
        bridge = 0.0        # mm to bridge this dropout (0 unless following was steady)
        seen = 0            # consecutive echoes while circling
        orbit_yaw = None    # yaw when circling a corner began
        orbit_px = 0.0      # measuring-point X when circling began
        parallel_yaw = self.yaw()  # heading last seen parallel to the wall
        bumped = False      # no echo yet since the last bump
        east_px = None      # measuring-point X where the front wall ended, first pass
        east_kind = None    # how it ended: "corner" (sonar lost it) or "bump" (inside corner)
        on_front = False    # back on the front wall's line heading east after a lap
        final_pass = False  # round the west corner after a lap, south of the front wall
        close_travel = None  # travel when odometry said "hit point" on the front wall
        wall_from = self.travel  # travel when the wall was last picked up
        corner = self.body_point(SONAR_FORWARD, SONAR_LEFT + target)
        origin = self.travel
        last_time = self.clock.time()
        last_log = last_time - LOG_EVERY
        bumps = 0
        self.update()  # Exclude approach, reverse and the initial turn.
        print("FOLLOW: time_ms,followed,sonar,filtered_out,x,y,yaw,encoder_yaw")

        while True:
            self.tick()
            now = self.clock.time()
            dt = max(0.001, (now - last_time) / 1000.0)
            last_time = now
            raw = reading = self.range_mm()

            # While circling, far echoes are other parts of the obstacle.
            if raw is not None and orbit_yaw is not None and raw > REACQUIRE_RANGE:
                raw = None
            # A sudden outward jump means the wall fell away: treat as missing.
            if (raw is not None and filtered is not None
                    and orbit_yaw is None and raw > filtered + GAP_JUMP):
                raw = None
            # One echo while circling is not enough to trust: keep circling.
            if orbit_yaw is not None:
                seen = seen + 1 if raw is not None else 0
                if 0 < seen < REACQUIRE_COUNT:
                    raw = None

            if raw is not None:
                if filtered is None or orbit_yaw is not None:
                    filtered = raw  # fresh start, or wall reacquired after circling
                    derivative = 0.0
                    parallel_yaw = self.yaw()  # best guess until the range settles
                    wall_from = self.travel
                old = filtered
                filtered += 0.40 * (raw - filtered)
                derivative += 0.25 * ((filtered - old) / dt - derivative)
                gap_travel = None
                orbit_yaw = None
                bumped = False
                valid_count += 1
                # Raw hit in world coordinates: the corner to circle if lost.
                # Far echoes are usually oblique and land in the wrong place.
                if raw <= target + CORNER_TRUST:
                    corner = self.body_point(SONAR_FORWARD, SONAR_LEFT + raw)
            else:
                if gap_travel is None:
                    gap_travel, gap_heading = self.travel, self.yaw()
                    # Bridge only a dropout in steady following, not a flicker.
                    bridge = GAP_BRIDGE if valid_count >= 3 else 0.0
                    if bumped:
                        bridge = BUMP_BRIDGE
                valid_count = 0
                derivative = 0.0

            px, py = self.body_point(POINT_FORWARD, POINT_LEFT)
            yaw = self.yaw()
            front_heading = abs(yaw - final_yaw) < FRONT_HEADING
            # Back on the front wall: it runs exactly along +X at wall_y, so
            # steady following there re-zeroes accumulated yaw and Y error.
            # The Y window keeps the wide heading window from matching other
            # shallow walls. No X or tight Y window: laps with bumps drifted
            # 500-700 mm in X and 260 mm in Y.
            steady_front = (raw is not None and valid_count >= 5 and front_heading
                            and abs(filtered - target) < 25 and abs(derivative) < 20
                            and abs(py - hit_y) < LOOP_Y_WINDOW)
            if FRONT_WALL_CORRECTION and steady_front:
                self.correct_yaw(0.1 * (final_yaw - yaw))
                if abs(wall_y - corner[1]) < LOOP_Y_WINDOW:
                    self.y += 0.2 * (wall_y - corner[1])
                px, py = self.body_point(POINT_FORWARD, POINT_LEFT)
                yaw = self.yaw()

            # ---- End of the lap ----
            # The final pass is recognised by heading, which the gyro keeps
            # within ~5-25 deg, not by X, which drifts 300-780 mm a lap. Once
            # back on the front wall, losing it, bumping, or turning on past
            # east all mean the robot has reached the wall's far end, which
            # is past the start: stop there rather than keep tracing.
            if (not on_front and raw is not None and front_heading
                    and abs(py - hit_y) < LOOP_Y_WINDOW):
                on_front = True
                print("Back on the front wall:", int(px), int(py))
            # Followed a real stretch of it, not a flicker at its west corner.
            along_front = (on_front and front_heading
                           and self.travel - wall_from >= WALL_RUN_MIN)
            if (not final_pass and self.travel - origin >= LOOP_MIN_TRAVEL
                    and yaw - EAST <= -LOOP_MIN_TURN and py < wall_y):
                final_pass = True  # Round the west corner: a clear line to the start.
            if final_pass:
                late = self.clock.time() > RUN_LIMIT - RETURN_RESERVE
                # "At or past" the hit point, not "crossing" it: when odometry
                # ran ahead, X was already past it when the lap turn finished.
                if close_travel is None and px >= hit_x:
                    if east_px is None or late:
                        self.finish("Back at hit point:", int(px), int(py),
                                    "followed", int(self.travel - origin), "mm")
                        return
                    # X usually lags, so this is often not yet the hit point.
                    # Follow on to the wall end, where X is known.
                    close_travel = self.travel
                    print("Hit point by odometry; following to the wall end")
                if late:
                    self.finish("Out of time; stopping by odometry")
                    return
                if (close_travel is not None
                        and self.travel - close_travel > east_px - hit_x + SEARCH_MARGIN):
                    self.finish("Wall end not found; stopping by odometry")
                    return
            # Turned well past a full lap: rounding the front wall's far end.
            # One-sided, so circling its west corner (yaw above final) is fine.
            if (on_front or close_travel is not None) and yaw < final_yaw - PAST_TURN:
                if (orbit_yaw is not None and east_kind == "corner"
                        and abs(orbit_yaw - final_yaw) < FRONT_HEADING):
                    # This circle began heading east along the front wall,
                    # so it began at the wall end seen on the first pass.
                    self.fix_x(east_px - orbit_px, "Front wall ended (outside corner):")
                else:
                    self.finish("Turned past the front wall's end; stopping by odometry")
                return

            if self.touch.pressed():
                # Concave corner: back off, turn right, then resume following.
                bumps += 1
                print("BUMP", bumps, ": backing up and turning right")
                bump_px, bump_yaw = px, yaw  # where the touch tip met the wall
                self.stop()
                self.set_phase("BUMP")
                brick.sound.file(SoundFile.OUCH)  # Blocks ~1 s; robot is stopped.
                self.straight(-BUMP_BACKUP, BUMP_SPEED)
                if self.touch.pressed():
                    self.straight(-BUMP_BACKUP, BUMP_SPEED)
                if (east_px is None and bumps == 1 and abs(bump_yaw - EAST) < 30
                        and abs(py - hit_y) < 150):
                    # First pass: the front wall ends in an inside corner. The
                    # touch tip meets it at the same X on the final pass.
                    east_px, east_kind = bump_px, "bump"
                    print("Front wall ends in an inside corner at X", int(bump_px))
                elif along_front or (close_travel is not None
                                     and bump_yaw < final_yaw + FRONT_HEADING):
                    # On the front wall after a lap, a bump is its far end:
                    # past the start. (A short wall once led into a spike, and
                    # the bump turn aimed at the workspace edge.)
                    if along_front and east_kind == "bump":
                        self.fix_x(east_px - bump_px, "Front wall ended (inside corner):")
                    else:
                        self.finish("Bumped while looking for the wall end;"
                                    " stopping by odometry")
                    return
                self.turn(BUMP_TURN)
                # The old corner lies on the wall just hit; circling it drove
                # straight back in. Assume the wall is now beside us instead.
                corner = self.body_point(SONAR_FORWARD, SONAR_LEFT + target)
                bumped = True
                filtered, derivative, valid_count = None, 0.0, 0
                gap_travel, orbit_yaw = None, None
                last_time = self.clock.time()
                self.set_phase("FOLLOW")
                continue

            if raw is None and self.travel - gap_travel < bridge:
                # Brief dropout: hold heading.
                hold = clamp(HEADING_KP * (gap_heading - self.yaw()), -18, 18)
                self.drive(SLOW_SPEED, hold)
            elif raw is None:
                # Convex corner: circle the last wall point seen.
                if orbit_yaw is None:
                    orbit_yaw, orbit_px, seen = self.yaw(), px, 0
                    # The front wall ends at the same place on both passes.
                    # Odometry X drifts (no wall corrects it), so reaching
                    # that end again fixes X, and the start is behind us.
                    # Only on the front wall itself (fresh odometry, no bump
                    # yet): from other start points the first eastward corner
                    # can be elsewhere on the obstacle.
                    if (east_px is None and bumps == 0 and abs(self.yaw() - EAST) < 30
                            and abs(py - hit_y) < 150):
                        east_px, east_kind = px, "corner"
                        print("Front wall ends in an outside corner at X", int(px))
                    # along_front: without a guard like it the catch fired at
                    # the front wall's WEST end, rounding onto the wall.
                    elif along_front and east_kind == "corner":
                        self.fix_x(east_px - px, "Front wall ended (outside corner):")
                        return
                if self.yaw() - orbit_yaw < -ORBIT_MAX_TURN:
                    raise RuntimeError("Obstacle lost")
                self.drive(ORBIT_SPEED, self.orbit_rate(corner))
            else:
                # Capped so a far reading cannot swing the nose into the wall.
                error = clamp(filtered - target, -ERROR_LIMIT, ERROR_LIMIT)
                yaw_rate = clamp(-DISTANCE_KP * error - DISTANCE_KD * derivative,
                                 -MAX_YAW_RATE, MAX_YAW_RATE)
                speed = FOLLOW_SPEED
                if abs(error) > 35 or abs(yaw_rate) > 20 or valid_count < 3:
                    speed = SLOW_SPEED
                yaw = self.yaw()
                if abs(error) < 15 and abs(derivative) < 15:
                    parallel_yaw = yaw  # In band and steady: parallel to the wall.
                # Fence the heading: a range that will not drop (oblique echo)
                # must not keep steering the nose into the wall.
                low, high = parallel_yaw - APPROACH_ANGLE, parallel_yaw + APPROACH_ANGLE
                if yaw < low:
                    yaw_rate = max(yaw_rate, HEADING_KP * (low - yaw))
                elif yaw > high:
                    yaw_rate = min(yaw_rate, HEADING_KP * (high - yaw))
                if filtered < 50:
                    speed = 20.0
                    yaw_rate = MAX_YAW_RATE  # Too close: turn away to the right.
                self.drive(speed, yaw_rate)

            if now - last_log >= LOG_EVERY:
                # reading: corrected sonar mm before the gap/far filters;
                # "*" marks a reading the filters threw out.
                print(now, int(self.travel - origin), reading,
                      "*" if raw is None and reading is not None else "",
                      int(self.x), int(self.y), int(self.yaw()), int(self.encoder_yaw))
                last_log = now
            wait(PERIOD)


def validate():
    if MODE not in ("CHECK", "STRAIGHT", "TURN", "RUN"):
        raise ValueError("Unknown MODE")
    if min(LEFT_DIAMETER, RIGHT_DIAMETER, AXLE_TRACK) <= 0 or GYRO_SIGN not in (-1, 1):
        raise ValueError("Invalid wheel geometry or gyro sign")
    if POINT_FORWARD <= 0 or not (0 < BODY_RADIUS <= 200):
        raise ValueError("Measure point geometry and <=40 cm footprint")
    target = TARGET_POINT_CLEARANCE + POINT_LEFT - SONAR_LEFT
    if not (60 <= target <= 240):
        raise ValueError("Side sonar target must be 60..240 mm; adjust mount/clearance")
    if not (0 <= FAST_APPROACH_DISTANCE <= 300):
        raise ValueError("Fast approach must end well before the 400 mm minimum wall distance")
    if not (0 < TARGET_POINT_CLEARANCE < 300):
        raise ValueError("Measuring point must be within 300 mm")
    if INITIAL_AXLE_CLEARANCE <= max(POINT_FORWARD, BODY_RADIUS + 10):
        raise ValueError("Need safe initial turning clearance")
    if INITIAL_AXLE_CLEARANCE - POINT_LEFT >= 290:
        raise ValueError("Initial measuring-point clearance too large")
    if (ORBIT_RADIUS ** 2 + POINT_FORWARD ** 2) ** 0.5 >= 280:
        raise ValueError("Orbit radius would put the measuring point near 300 mm")
    if ORBIT_RADIUS <= SONAR_LEFT + 30:
        raise ValueError("Orbit radius too small for the sonar to see the corner")


def main():
    validate()
    if MODE != "CHECK":
        LOG.open()
        print("RUN LOG:", LOG.name)
        print("SETTINGS: MODE", MODE, "FOLLOW_SPEED", FOLLOW_SPEED, "SLOW_SPEED", SLOW_SPEED,
              "ORBIT_SPEED", ORBIT_SPEED, "ORBIT_RADIUS", ORBIT_RADIUS,
              "TURN_RATE", TURN_RATE, "BUMP_TURN", BUMP_TURN, "BUMP_BACKUP", BUMP_BACKUP,
              "TARGET_POINT_CLEARANCE", TARGET_POINT_CLEARANCE,
              "FRONT_HEADING", FRONT_HEADING, "WALL_RUN_MIN", WALL_RUN_MIN,
              "PAST_TURN", PAST_TURN, "SEARCH_MARGIN", SEARCH_MARGIN,
              "MAX_X_FIX", MAX_X_FIX, "RETURN_RESERVE", RETURN_RESERVE,
              "RUN_LIMIT", RUN_LIMIT)
    robot = Robot()
    try:
        if MODE == "CHECK":
            print("CHECK: motors remain stopped. DOWN exits.")
            print("Touch S1 | left sonar mm S4 | gyro deg S2")
            while Button.DOWN not in brick.buttons():
                print(robot.touch.pressed(), robot.sonar.distance(), robot.yaw())
                wait(500)
            return

        robot.start()
        if MODE == "STRAIGHT":
            robot.set_phase("STRAIGHT")
            robot.straight(500.0, 60.0)
        elif MODE == "TURN":
            robot.set_phase("TURN")
            robot.turn(90.0)
        else:
            if robot.touch.pressed():
                raise RuntimeError("Front touch already pressed at start")
            robot.set_phase("APPROACH")
            print("APPROACH")
            robot.straight(APPROACH_LIMIT, APPROACH_SPEED, True, FAST_APPROACH_DISTANCE)
            brick.sound.beep()  # Required: obstacle found (in contact).
            # Contact establishes the front wall's Y coordinate.
            wall_y = robot.body_point(POINT_FORWARD, POINT_LEFT)[1]
            print("Hit point:", int(robot.body_point(POINT_FORWARD, POINT_LEFT)[0]),
                  int(wall_y))
            robot.set_phase("BACKUP")
            print("BACK UP")
            robot.straight(-(INITIAL_AXLE_CLEARANCE - POINT_FORWARD), 50.0)
            if robot.touch.pressed():
                raise RuntimeError("Front touch stuck after backing up")
            robot.set_phase("TURN")
            print("TURN RIGHT")
            robot.turn(90.0)
            robot.set_phase("FOLLOW")
            robot.follow(wall_y)
            robot.set_phase("RETURN")
            print("RETURN")
            robot.go_to(START_X, START_Y, RETURN_SPEED)
            print("Done at", robot.clock.time(), "ms")
        robot.stop()
        robot.running = False
        brick.sound.beep()  # Only a completed task/test gets the success beep.
        robot.result("done")
    except Exception as error:
        robot.stop()
        print("STOPPED:", error)
        robot.result("stopped")
    finally:
        robot.stop()
        LOG.close()


if __name__ == "__main__":
    main()
