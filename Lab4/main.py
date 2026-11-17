#!/usr/bin/env pybricks-micropython
# COMP 581, Fall 2026 - Lab 4: Goal Seeking Among Unknown Obstacles
# Team number: TODO
# Tanish Bachu: 730739311
# Minh Nguyen: 730766544
#
# Drives straight for the goal. On contact it backs off, turns right and
# follows the obstacle with it on the left (the sonar side), as in Lab 3,
# until the goal lies ahead on the open side of the wall. Then it heads for
# the goal again. It stops and beeps when the measuring point reaches the
# goal point.


from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor, TouchSensor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch, print as console_print


# ------------------------ MEASURE / CALIBRATE -------------------------
# Same robot and calibration as Lab 3.
MODE = "RUN"                   # CHECK, STRAIGHT, TURN, or RUN
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

# ------------------------------- COURSE -------------------------------
WORKSPACE = 4000.0             # mm; square workspace, corner at (0, 0).
START_X = 500.0                # Measuring point starts on the starting line here.
START_Y = 0.0
START_YAW = 0.0                # deg clockwise from +Y that the robot faces at the start.
GOAL_X = 2500.0
GOAL_Y = 2500.0
GOAL_RADIUS = 300.0            # mm; the goal region. Beep only inside it.
GOAL_STOP = 200.0              # mm; when the goal cannot be driven to directly (an
                               # obstacle is in the way), stop this close: inside the
                               # goal region with 100 mm to spare for odometry error
                               # (250 stopped 335 mm off once in the desk simulation).
GIVE_UP = 170000               # ms; stop wherever we are. The task limit is 180 s, and
                               # a stop before it still earns goal-accuracy points.
OUTSIDE_LIMIT = 350.0          # mm the measuring point may be outside the workspace
                               # while following a wall before heading for the goal.

# ---------------------------- DRIVE TO GOAL ---------------------------
DRIVE_SPEED = 200.0            # mm/s toward the goal on open floor.
MIN_SPEED = 40.0               # mm/s; slowest command while closing on the goal.
RAMP_ACCEL = 400.0             # mm/s^2; gentle start to avoid wheel slip.
GOAL_SLOWDOWN = 1.5            # (mm/s) per mm left: slows over the last ~130 mm.
GO_YAW_RATE = 30.0             # deg/s; most steering while driving to the goal.
FACE_GOAL = 20.0               # deg; a bigger heading error turns in place first.
GO_PERIOD = 20                 # ms; no sonar here, so poll the touch sensor faster.
CONTACT_BACKUP = 110.0         # mm to reverse after driving into an obstacle: puts the
                               # axle ~200 mm off a square wall, as in Lab 3, allowing
                               # ~15 mm of overshoot at DRIVE_SPEED.
STALL_FRACTION = 0.3           # Wheels this far below the commanded speed for
STALL_TIME = 400               # this many ms count as a contact: the body hit
STALL_MIN_COMMAND = 80.0       # something the touch rod missed. Only above this mm/s.
DEFLECT_ANGLE = 8.0            # deg off the aim, after tracking it within 3, for
DEFLECT_TIME = 150             # this many ms also counts as a contact: a body corner
                               # caught on a slanted wall twists the robot as it slides.

# ---------------------------- WALL FOLLOWING --------------------------
# From Lab 3 (tested there), plus its fence re-aim for slanted walls.
TARGET_POINT_CLEARANCE = 200.0 # Desired measuring-point distance from the obstacle.
FOLLOW_SPEED = 150.0           # mm/s along a steady wall.
SLOW_SPEED = 100.0
ORBIT_SPEED = 85.0             # mm/s while circling a convex corner.
MAX_YAW_RATE = 60.0            # deg/s while following.
TURN_RATE = 75.0               # deg/s; max rate for in-place turns.
TURN_TIMEOUT = 6500            # ms; give up on an in-place turn after this.
DISTANCE_KP = 0.32             # (deg/s) per mm of range error.
DISTANCE_KD = 0.10             # (deg/s) per (mm/s) of filtered range change.
HEADING_KP = 2.0               # (deg/s) per degree of heading error.
PERIOD = 50                    # ms; do not poll sonar in a tight busy loop.
GAP_BRIDGE = 40.0              # mm to hold heading through a sonar dropout
                               # before circling the last wall point seen.
GAP_JUMP = 60.0                # mm; a sudden outward jump this big means the wall
                               # fell away (convex corner) or a bad echo.
ORBIT_RADIUS = 215.0           # mm, axle -> corner; puts the measuring point
                               # ~240 mm from the corner and the sonar ~122 mm.
ORBIT_KP = 0.005               # rad of heading per mm of orbit radius error.
ORBIT_MAX_TURN = 330.0         # deg circled with no echo before giving up on the wall.
REACQUIRE_RANGE = 200.0        # mm; while circling, only nearer echoes are the wall.
REACQUIRE_COUNT = 2            # consecutive echoes needed to stop circling.
CORNER_TRUST = 80.0            # mm past target; farther echoes do not move the corner.
ERROR_LIMIT = 60.0             # mm; caps steering at about 19 deg/s from range error.
BUMP_BACKUP = 100.0            # mm to reverse after hitting the wall while following.
BUMP_SPEED = 150.0             # mm/s for that reverse.
BUMP_SETTLE = 150              # ms to let the robot stop before reversing.
BUMP_TURN = 90.0               # deg to turn right after a contact (no sonar fix).
BUMP_BRIDGE = 120.0            # mm to hold heading after a contact before circling.
APPROACH_ANGLE = 25.0          # deg; most the heading may swing from the last
                               # heading that was parallel to the wall.
FENCE_RATE = 10.0              # mm/s; a range changing faster than this re-aims the
                               # fence at the wall's direction (asin(rate / speed)).
STUCK_TIME = 2500              # ms with under STUCK_TRAVEL mm of travel and under
STUCK_TRAVEL = 40.0            # STUCK_TURN deg of turning while following counts as a
STUCK_TURN = 20.0              # contact: pinned with the wall flush on the left, the
                               # "too close" crawl never reached the stall speed.

# ---------------------- TURNING AWAY FROM A CONTACT -------------------
# After a contact the wall can run at any angle, not square to the robot as
# in Lab 3. Turning right while watching the left sonar finds the heading
# where it faces the wall square on: the robot is then parallel to the wall.
ALIGN_SCAN = True              # False: always turn BUMP_TURN instead.
SCAN_RATE = 60.0               # deg/s while scanning.
SCAN_MIN = 30.0                # deg; turn at least this far.
SCAN_MAX = 150.0               # deg; stop scanning here whatever the sonar says.
SCAN_RANGE = 350.0             # mm; farther echoes are not the wall just touched.
SCAN_FLAT = 15.0               # mm; echoes this close to the nearest are "square on".
SCAN_LOST = 20.0               # deg turned past the last echo: the wall is behind us.

# --------------------------- LEAVING THE WALL -------------------------
LEAVE_SLACK = 50.0             # mm the goal may lie on the wall's side of the line
                               # along the wall through the axle (the body clears it).
LEAVE_MAX_ANGLE = 90.0         # deg; never leave for a goal further right than this:
                               # behind us, the line to it can cross a pocket's far side.
LEAVE_MIN_FOLLOW = 50.0        # mm followed since the last contact before leaving.
LEAVE_REVISIT = 350.0          # mm; no leaving within this of an earlier leave point,
                               # so a pocket that sends the robot back cannot loop it.
LOG_EVERY = 500                # ms; telemetry rows on the console and in the log.

PI = 3.141592653589793
RAD = PI / 180.0


class OutOfTime(Exception):
    pass


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


def wall_angle(rate, speed):
    # Degrees the heading is turned clockwise (away) from a wall on the left
    # whose range changes at `rate` mm/s while driving at `speed` mm/s.
    if speed < 30.0:
        return 0.0
    s = clamp(rate / speed, -0.7, 0.7)
    return arctan2(s, (1.0 - s * s) ** 0.5) / RAD


def goal_distance(point):
    dx, dy = GOAL_X - point[0], GOAL_Y - point[1]
    return (dx * dx + dy * dy) ** 0.5


def outside(point):
    # mm the point lies outside the workspace (0 or less when inside).
    return max(-point[0], point[0] - WORKSPACE, -point[1], point[1] - WORKSPACE)


def brake_motor(motor):
    # v1 accepts stop(Stop.BRAKE); v2 also exposes brake().
    try:
        motor.stop(Stop.BRAKE)
    except TypeError:
        motor.brake()


# ------------------------------- RUN LOG ------------------------------
# Everything printed also goes to run_log.txt next to this file, so runs
# started from the brick's menu are kept too. Each run is appended.
try:
    LOG_PATH = (__file__.rsplit("/", 1)[0] + "/" if "/" in __file__ else "") + "run_log.txt"
except NameError:
    LOG_PATH = "run_log.txt"


class RunLog:
    def __init__(self):
        self.file = None

    def open(self):
        # Logging problems only turn logging off; they never stop the robot.
        try:
            self.file = open(LOG_PATH, "a")
        except Exception as error:
            self.file = None
            console_print("Run log off:", error)

    def line(self, text):
        if self.file is not None:
            try:
                self.file.write(text + "\n")
                self.file.flush()
            except Exception:
                self.file = None

    def close(self):
        try:
            if self.file is not None:
                self.file.close()
        except Exception:
            pass
        self.file = None


LOG = RunLog()


def print(*args):
    # Console and the run log get the same lines.
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
        self.clock = StopWatch()
        self.running = False
        self.phase = "START"
        self.contacts = 0
        self.leaves = []       # axle (x, y) wherever the robot left a wall
        self.stuck = False     # wheels turned but the gyro did not
        self.stall_since = None
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
                + START_YAW)

    def reset_pose(self):
        self.old_left = self.left.angle()
        self.old_right = self.right.angle()
        self.old_yaw = self.yaw()
        self.old_time = self.clock.time()
        # The measuring point is on (START_X, START_Y); the axle is behind it.
        h = (90.0 - self.old_yaw) * RAD
        self.x = START_X - POINT_FORWARD * cosine(h) + POINT_LEFT * sine(h)
        self.y = START_Y - POINT_FORWARD * sine(h) - POINT_LEFT * cosine(h)
        self.encoder_yaw = 0.0
        self.travel = 0.0
        self.ground_speed = 0.0  # mm/s, filtered, from the encoders
        self.check_time, self.check_encoder, self.check_yaw = self.clock.time(), 0.0, self.old_yaw

    def update(self):
        la, ra, yaw = self.left.angle(), self.right.angle(), self.yaw()
        now = self.clock.time()
        dl = (la - self.old_left) * PI * LEFT_DIAMETER / 360.0
        dr = (ra - self.old_right) * PI * RIGHT_DIAMETER / 360.0
        ds = (dl + dr) / 2.0
        # Differential-drive forward kinematics, clockwise-positive yaw.
        self.encoder_yaw += (dl - dr) / AXLE_TRACK / RAD
        heading_mid = (90.0 - (yaw + self.old_yaw) / 2.0) * RAD
        self.x += ds * cosine(heading_mid)
        self.y += ds * sine(heading_mid)
        self.travel += ds
        if now > self.old_time:
            self.ground_speed += 0.5 * (ds * 1000.0 / (now - self.old_time) - self.ground_speed)
        self.old_left, self.old_right, self.old_yaw, self.old_time = la, ra, yaw, now

    def drive(self, speed, yaw_rate):
        # Inverse differential-drive kinematics, wheel mm/s -> motor deg/s.
        differential = yaw_rate * RAD * AXLE_TRACK / 2.0
        self.left.run((speed + differential) * 360.0 / (PI * LEFT_DIAMETER))
        self.right.run((speed - differential) * 360.0 / (PI * RIGHT_DIAMETER))

    def guard(self):
        if Button.DOWN in brick.buttons():
            raise RuntimeError("User abort")
        if self.running and self.clock.time() >= GIVE_UP:
            raise OutOfTime()

    def tick(self):
        self.update()
        self.guard()
        # Every second, compare gyro and encoder rotation: wheels turning
        # with no sign of turning means the body is pinned and slipping.
        now = self.clock.time()
        if now - self.check_time >= 1000:
            turned = self.encoder_yaw - self.check_encoder
            seen = self.yaw() - self.check_yaw
            if self.running and abs(turned) > 40 and abs(seen) < 0.25 * abs(turned):
                print("Wheels turning but gyro still: pinned against something")
                self.stuck = True
            self.check_time, self.check_encoder, self.check_yaw = now, self.encoder_yaw, self.yaw()

    def blocked(self, command):
        # Wheels held back while driving forward: the body has run into
        # something the touch rod missed, such as a corner hit off center.
        if command >= STALL_MIN_COMMAND and self.ground_speed < STALL_FRACTION * command:
            if self.stall_since is None:
                self.stall_since = self.clock.time()
            return self.clock.time() - self.stall_since >= STALL_TIME
        self.stall_since = None
        return False

    def range_mm(self):
        value = self.sonar.distance()
        # Reject no-echo codes (often 2550 mm) and implausible close echoes.
        if 30 <= value <= 600:
            return float(value) - SONAR_OFFSET
        return None

    def set_phase(self, phase):
        self.phase = phase

    def body_point(self, forward, left):
        h = (90.0 - self.yaw()) * RAD
        return (self.x + forward * cosine(h) - left * sine(h),
                self.y + forward * sine(h) + left * cosine(h))

    def point(self):
        return self.body_point(POINT_FORWARD, POINT_LEFT)

    def bearing(self, tx, ty):
        # Yaw that points the axle midpoint at world point (tx, ty).
        return 90.0 - arctan2(ty - self.y, tx - self.x) / RAD

    def goal_ahead(self, point):
        # mm the goal lies ahead of `point` along the current heading.
        h = (90.0 - self.yaw()) * RAD
        return (GOAL_X - point[0]) * cosine(h) + (GOAL_Y - point[1]) * sine(h)

    def log_row(self, now, *extra):
        px, py = self.point()
        print(self.phase, now, int(self.x), int(self.y), int(px), int(py),
              int(self.yaw()), int(goal_distance((px, py))), *extra)

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
        # A single 1-degree step is quantization, not drift.
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

    def straight(self, distance, speed):
        origin, heading = self.travel, self.yaw()
        start = self.clock.time()
        timeout = abs(distance) / speed * 1000.0 + 2000.0
        direction = 1.0 if distance >= 0 else -1.0
        while True:
            self.tick()
            if abs(self.travel - origin) >= abs(distance):
                break
            if self.clock.time() - start > timeout:
                # Backing into something unseen: go on from here.
                print("Straight move blocked after", int(self.travel - origin), "mm")
                break
            error = heading - self.yaw()
            self.drive(direction * speed, clamp(HEADING_KP * error, -18, 18))
            wait(PERIOD)
        self.stop()
        wait(100)
        self.tick()

    def turn(self, degrees):
        target, start = self.yaw() + degrees, self.clock.time()
        settled = 0
        self.stuck = False
        while settled < 3:
            self.tick()
            if self.stuck:
                # Pinned against the wall mid-turn: back off and carry on.
                self.stuck = False
                self.stop()
                self.straight(-BUMP_BACKUP / 2.0, BUMP_SPEED)
                continue
            error = target - self.yaw()
            if self.clock.time() - start > TURN_TIMEOUT:
                print("Turn timed out", int(error), "deg short")
                break
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

    def back_off(self, distance):
        self.set_phase("BACKUP")
        self.straight(-distance, BUMP_SPEED)
        if self.touch.pressed():
            self.straight(-distance, BUMP_SPEED)

    def turn_from_wall(self):
        # Turn right until the obstacle just touched is beside us on the
        # left, parallel to the robot, then follow it from there.
        self.set_phase("TURN")
        if not ALIGN_SCAN:
            self.turn(BUMP_TURN)
            return
        # Turn right while the left sonar sweeps across the wall. Its echoes
        # are nearest, and steady, while it faces the wall square on: aim
        # for the middle of that stretch. A wall seen only at an angle
        # gives no echo, so a sweep that sees nothing falls back to 90 deg.
        start, began = self.yaw(), self.clock.time()
        best, last_seen, seen = None, None, []
        self.stuck = False
        while True:
            self.tick()
            turned = self.yaw() - start
            reading = self.range_mm()
            if reading is not None and reading <= SCAN_RANGE:
                best = reading if best is None else min(best, reading)
                last_seen = turned
                seen.append((turned, reading))
            if turned >= SCAN_MAX or (turned >= SCAN_MIN and last_seen is not None
                                      and turned - last_seen >= SCAN_LOST):
                break
            if self.stuck or self.clock.time() - began > TURN_TIMEOUT:
                break
            self.drive(0, SCAN_RATE)
            wait(PERIOD)
        self.stop()
        if best is None:
            aim = BUMP_TURN
        else:
            square = [t for (t, r) in seen if r <= best + SCAN_FLAT]
            aim = clamp((square[0] + square[-1]) / 2.0, SCAN_MIN, SCAN_MAX)
        print("Turn from wall: nearest echo", None if best is None else int(best),
              "mm; aiming", int(aim), "deg right of the contact heading")
        self.turn(start + aim - self.yaw())

    def go_to_goal(self):
        # Head straight for the goal. Returns "goal" once the measuring point
        # is there, or "contact" when the touch rod (or a stall) finds an
        # obstacle in the way.
        self.set_phase("GO")
        error = wrap180(self.bearing(GOAL_X, GOAL_Y) - self.yaw())
        if abs(error) > FACE_GOAL:
            self.turn(error)
        start = self.clock.time()
        heading = self.bearing(GOAL_X, GOAL_Y)
        self.stuck, self.stall_since = False, None
        on_aim = False         # heading has been within 3 deg of the aim
        deflected = None       # time the heading was first pushed off the aim
        last_log = start - LOG_EVERY
        while True:
            self.tick()
            now = self.clock.time()
            p = self.point()
            remaining = self.goal_ahead(p)
            if remaining <= 0:
                self.stop()
                return "goal"
            command = clamp(min(MIN_SPEED + RAMP_ACCEL * (now - start) / 1000.0,
                                remaining * GOAL_SLOWDOWN), MIN_SPEED, DRIVE_SPEED)
            if remaining > 150:
                heading = self.bearing(GOAL_X, GOAL_Y)  # Freeze aim when close.
            error = wrap180(heading - self.yaw())
            on_aim = on_aim or abs(error) < 3.0
            if not on_aim or abs(error) < DEFLECT_ANGLE:
                deflected = None
            elif deflected is None:
                deflected = now
            if (self.touch.pressed() or self.stuck or self.blocked(command)
                    or (deflected is not None and now - deflected >= DEFLECT_TIME)):
                self.stop()
                if deflected is not None:
                    print("Pushed", int(error), "deg off the aim: contact")
                return "contact"
            self.drive(command, clamp(HEADING_KP * error, -GO_YAW_RATE, GO_YAW_RATE))
            if now - last_log >= LOG_EVERY:
                self.log_row(now)
                last_log = now
            wait(GO_PERIOD)

    def orbit_rate(self, corner):
        # Circle `corner` counterclockwise (corner on the left) at ORBIT_RADIUS.
        dx, dy = corner[0] - self.x, corner[1] - self.y
        distance = (dx * dx + dy * dy) ** 0.5
        theta = (arctan2(dy, dx) - PI / 2.0
                 + clamp(ORBIT_KP * (distance - ORBIT_RADIUS), -0.8, 0.8))
        error = wrap180(90.0 - theta / RAD - self.yaw())
        feedforward = -ORBIT_SPEED / ORBIT_RADIUS / RAD  # Steady left turn.
        return clamp(feedforward + HEADING_KP * error, -MAX_YAW_RATE, MAX_YAW_RATE)

    def clear_to_goal(self, wall):
        # True when the goal lies on the open side of the wall, so a straight
        # line to it moves away from the wall (exactly so on a convex
        # obstacle): no more than LEAVE_SLACK past the line along the wall
        # through the axle, and not behind us. wall: the wall's heading.
        dx, dy = GOAL_X - self.x, GOAL_Y - self.y
        distance = (dx * dx + dy * dy) ** 0.5
        right = wrap180(90.0 - arctan2(dy, dx) / RAD - wall)  # deg right of the wall
        if right > LEAVE_MAX_ANGLE or right < -60.0:
            return False
        if right < 0 and distance * sine(-right * RAD) > LEAVE_SLACK:
            return False
        for (lx, ly) in self.leaves:
            if (lx - self.x) ** 2 + (ly - self.y) ** 2 < LEAVE_REVISIT ** 2:
                return False
        return True

    def follow(self):
        # Keep the obstacle on the left until the goal lies ahead on its open
        # side. Returns "leave" to head for the goal, or "goal" to stop here.
        target = TARGET_POINT_CLEARANCE + POINT_LEFT - SONAR_LEFT
        filtered = None
        derivative = 0.0
        valid_count = 0
        gap_travel = None   # travel when the current dropout started
        gap_heading = 0.0   # heading held while bridging a dropout
        bridge = 0.0        # mm to bridge this dropout (0 unless following was steady)
        seen = 0            # consecutive echoes while circling
        orbit_yaw = None    # yaw when circling a corner began
        parallel_yaw = self.yaw()  # heading last seen parallel to the wall
        bumped = True       # no echo yet since the last contact
        contact_travel = self.travel  # travel at the last contact
        command = 0.0       # forward speed last commanded
        corner = self.body_point(SONAR_FORWARD, SONAR_LEFT + target)
        self.stuck, self.stall_since = False, None
        self.set_phase("FOLLOW")
        self.update()  # Exclude the reverse and the turn.
        last_time = self.clock.time()
        last_travel = self.travel
        last_log = last_time - LOG_EVERY
        progress = (last_time, self.travel, self.yaw())  # last real movement

        while True:
            self.tick()
            now = self.clock.time()
            dt = max(0.001, (now - last_time) / 1000.0)
            last_time = now
            ground_speed = (self.travel - last_travel) / dt  # mm/s, from encoders
            last_travel = self.travel
            if (abs(self.travel - progress[1]) >= STUCK_TRAVEL
                    or abs(self.yaw() - progress[2]) >= STUCK_TURN):
                progress = (now, self.travel, self.yaw())
            elif now - progress[0] >= STUCK_TIME:
                print("No progress for", STUCK_TIME, "ms: pinned against the wall")
                self.stuck = True
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

            p = self.point()
            distance = goal_distance(p)
            # Passing the goal with the wall in the way of a straight line to
            # it: stop where it comes abeam, the closest this wall allows.
            if distance <= GOAL_STOP and self.goal_ahead(p) <= 0:
                self.stop()
                print("Goal abeam,", int(distance), "mm away")
                return "goal"
            if outside(p) > OUTSIDE_LIMIT:
                self.stop()
                print("Wall leads out of the workspace; heading for the goal")
                self.leaves.append((self.x, self.y))
                return "leave"
            # The wall's heading: ours while circling a corner, else the
            # last heading seen parallel to it.
            wall_heading = None
            if orbit_yaw is not None:
                wall_heading = self.yaw()
            elif raw is not None and valid_count >= 3:
                wall_heading = parallel_yaw
            if (wall_heading is not None and self.travel - contact_travel >= LEAVE_MIN_FOLLOW
                    and self.clear_to_goal(wall_heading)):
                self.stop()
                print("Leaving the wall for the goal at", int(self.x), int(self.y),
                      "wall heading", int(wall_heading))
                self.leaves.append((self.x, self.y))
                return "leave"

            if self.touch.pressed() or self.stuck or self.blocked(command):
                # Concave corner: back off, turn right, then resume following.
                self.stop()
                self.contacts += 1
                print("BUMP", self.contacts, "at", int(p[0]), int(p[1]),
                      "touch" if self.touch.pressed() else "stall")
                if distance <= GOAL_STOP:
                    print("Bumped next to the goal,", int(distance), "mm away")
                    return "goal"
                wait(BUMP_SETTLE)
                self.back_off(BUMP_BACKUP)
                self.turn_from_wall()
                self.set_phase("FOLLOW")
                # The old corner lies on the wall just hit; circling it drove
                # straight back in. Assume the wall is now beside us instead.
                corner = self.body_point(SONAR_FORWARD, SONAR_LEFT + target)
                bumped = True
                filtered, derivative, valid_count = None, 0.0, 0
                gap_travel, orbit_yaw = None, None
                contact_travel = self.travel
                command = 0.0
                self.stuck, self.stall_since = False, None
                last_time = self.clock.time()
                last_travel = self.travel
                progress = (last_time, self.travel, self.yaw())
                continue

            if raw is None and self.travel - gap_travel < bridge:
                # Brief dropout: hold heading.
                hold = clamp(HEADING_KP * (gap_heading - self.yaw()), -18, 18)
                command = SLOW_SPEED
                self.drive(SLOW_SPEED, hold)
            elif raw is None:
                # Convex corner: circle the last wall point seen.
                if orbit_yaw is None:
                    orbit_yaw, seen = self.yaw(), 0
                if self.yaw() - orbit_yaw < -ORBIT_MAX_TURN:
                    self.stop()
                    print("Obstacle lost; heading for the goal")
                    self.leaves.append((self.x, self.y))
                    return "leave"
                command = ORBIT_SPEED
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
                elif (valid_count >= 3 and abs(derivative) > FENCE_RATE
                        and ground_speed > 30.0):
                    # The range is steadily changing, so the wall runs at
                    # asin(rate / speed) from our heading. Without this, a wall
                    # picked up mid-corner left the fence anchored up to ~30 deg
                    # off, and the robot drove straight away from it. An echo
                    # that will not change (oblique) has no rate, so it still
                    # cannot pull the nose into the wall.
                    wall = yaw - wall_angle(derivative, ground_speed)
                    parallel_yaw += 0.2 * (wall - parallel_yaw)
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
                command = speed
                self.drive(speed, yaw_rate)

            if now - last_log >= LOG_EVERY:
                # reading: corrected sonar mm before the gap/far filters;
                # "*" marks a reading the filters threw out.
                self.log_row(now, reading, "*" if raw is None and reading is not None else "")
                last_log = now
            wait(PERIOD)

    def navigate(self):
        # Alternate driving at the goal and following whatever is in the way.
        # Returns how the run ended.
        while True:
            if self.go_to_goal() == "goal":
                return "Reached the goal point"
            self.contacts += 1
            p = self.point()
            distance = goal_distance(p)
            print("CONTACT", self.contacts, "at", int(p[0]), int(p[1]), ",",
                  int(distance), "mm from the goal")
            if distance <= GOAL_STOP:
                return "Obstacle at the goal"
            wait(BUMP_SETTLE)
            self.back_off(CONTACT_BACKUP)
            self.turn_from_wall()
            if self.follow() == "goal":
                return "Stopped beside the obstacle next to the goal"

    def finish(self, how):
        self.stop()
        self.running = False
        p = self.point()
        distance = goal_distance(p)
        if distance <= GOAL_RADIUS:
            brick.sound.beep()  # Required: stopped in the goal region.
        print("RESULT:", how, "| time", self.clock.time(), "ms | measuring point",
              int(p[0]), int(p[1]), "|", int(distance), "mm from the goal | contacts",
              self.contacts)


def validate():
    if MODE not in ("CHECK", "STRAIGHT", "TURN", "RUN"):
        raise ValueError("Unknown MODE")
    if min(LEFT_DIAMETER, RIGHT_DIAMETER, AXLE_TRACK) <= 0 or GYRO_SIGN not in (-1, 1):
        raise ValueError("Invalid wheel geometry or gyro sign")
    target = TARGET_POINT_CLEARANCE + POINT_LEFT - SONAR_LEFT
    if not (60 <= target <= 240):
        raise ValueError("Side sonar target must be 60..240 mm; adjust mount/clearance")
    if not (0 < GOAL_STOP <= GOAL_RADIUS):
        raise ValueError("GOAL_STOP must lie inside the goal region")
    if not (0 < GIVE_UP < 180000):
        raise ValueError("GIVE_UP must come before the 3-minute limit")
    if ORBIT_RADIUS <= SONAR_LEFT + 30:
        raise ValueError("Orbit radius too small for the sonar to see the corner")
    if not (0 < SCAN_MIN < SCAN_MAX <= 180):
        raise ValueError("Scan limits out of order")


def main():
    validate()
    if MODE != "CHECK":
        LOG.open()
        print("==== LAB 4 RUN, log", LOG_PATH)
        print("SETTINGS: MODE", MODE, "DRIVE_SPEED", DRIVE_SPEED, "FOLLOW_SPEED", FOLLOW_SPEED,
              "ALIGN_SCAN", ALIGN_SCAN, "GOAL_STOP", GOAL_STOP, "LEAVE_SLACK", LEAVE_SLACK,
              "LEAVE_REVISIT", LEAVE_REVISIT, "GIVE_UP", GIVE_UP)
        print("Rows: phase, time_ms, x, y, px, py, yaw, goal_mm [, sonar, filtered_out]")
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
            robot.straight(500.0, 60.0)
        elif MODE == "TURN":
            robot.turn(90.0)
        else:
            if robot.touch.pressed():
                raise RuntimeError("Front touch already pressed at start")
            robot.finish(robot.navigate())
            return
        robot.stop()
        robot.running = False
        brick.sound.beep()  # Test finished.
    except OutOfTime:
        robot.finish("Out of time")
    except Exception as error:
        robot.stop()
        print("STOPPED:", error)
    finally:
        robot.stop()
        LOG.close()


if __name__ == "__main__":
    main()
