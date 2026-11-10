#!/usr/bin/env pybricks-micropython
# COMP 581, Fall 2026 - Lab 3: Obstacle Tracing
# Tanish Bachu: 730739311
# Minh Nguyen: 730766544


from pybricks import ev3brick as brick
from pybricks.ev3devices import Motor, TouchSensor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, SoundFile
from pybricks.tools import wait, StopWatch, print


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
FOLLOW_SPEED = 140.0           # 120/80/70 finished at 112 s with no time left
SLOW_SPEED = 90.0              # for ouch pauses or the front-wall-end catch.
ORBIT_SPEED = 80.0             # mm/s while circling a convex corner.
RETURN_SPEED = 150.0
MAX_YAW_RATE = 60.0            # deg/s while following.
TURN_RATE = 75.0               # deg/s; max rate for in-place turns.
DISTANCE_KP = 0.32             # (deg/s) per mm of range error.
DISTANCE_KD = 0.10             # (deg/s) per (mm/s) of filtered range change.
HEADING_KP = 2.0               # (deg/s) per degree of heading error.
PERIOD = 50                    # ms; do not poll sonar in a tight busy loop.
RUN_LIMIT = 117000             # ms from CENTER press; task limit is 120 s.
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
BUMP_SPEED = 100.0             # mm/s for that reverse.
BUMP_TURN = 60.0               # deg to turn right after a bump before resuming;
                               # 35 left the nose ~100 mm from the same wall.
BUMP_BRIDGE = 120.0            # mm to hold heading after a bump before circling.
APPROACH_ANGLE = 25.0          # deg; most the heading may swing from the last
                               # heading that was parallel to the wall.
LOOP_MIN_TRAVEL = 1000.0       # mm followed before the loop may close (perimeter >= 1.5 m).
LOOP_MIN_TURN = 270.0          # deg of net left turn before the loop may close.
LOOP_Y_WINDOW = 400.0          # mm; how far from the hit point line odometry may be.
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
        if 30 <= value <= 600:
            return float(value) - SONAR_OFFSET
        return None

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
        parallel_yaw = self.yaw()  # heading last seen parallel to the wall
        bumped = False      # no echo yet since the last bump
        east_px = None      # measuring-point X where the front wall ended, first pass
        on_front = False    # steadily followed the front wall after a full lap
        corner = self.body_point(SONAR_FORWARD, SONAR_LEFT + target)
        origin = self.travel
        last_px = self.body_point(POINT_FORWARD, POINT_LEFT)[0]
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
            # Back on the front wall: it runs exactly along +X at wall_y, so
            # steady following there re-zeroes accumulated yaw and Y error.
            steady_front = (raw is not None and valid_count >= 5
                            and abs(self.yaw() - final_yaw) < 20
                            and abs(filtered - target) < 25 and abs(derivative) < 20
                            and abs(py - hit_y) < LOOP_Y_WINDOW)
            if steady_front:
                on_front = True  # Final pass along the front wall has begun.
            # No X or tight Y window here: laps with bumps drifted 500-700 mm
            # in X and 260 mm in Y, which 800/150 mm limits never corrected.
            if FRONT_WALL_CORRECTION and steady_front:
                self.correct_yaw(0.1 * (final_yaw - self.yaw()))
                if abs(wall_y - corner[1]) < LOOP_Y_WINDOW:
                    self.y += 0.2 * (wall_y - corner[1])
                px, py = self.body_point(POINT_FORWARD, POINT_LEFT)

            # Loop closes when the measuring point crosses the hit point's X
            # heading east again, after a full lap of net left turning.
            if (self.travel - origin >= LOOP_MIN_TRAVEL
                    and self.yaw() - EAST <= -LOOP_MIN_TURN
                    and abs(py - hit_y) < LOOP_Y_WINDOW
                    and last_px < hit_x <= px):
                self.stop()
                print("Back at hit point:", int(px), int(py),
                      "followed", int(self.travel - origin), "mm")
                return
            last_px = px

            if self.touch.pressed():
                # Concave corner: back off, turn right, then resume following.
                bumps += 1
                print("BUMP", bumps, ": backing up and turning right")
                self.stop()
                brick.sound.file(SoundFile.OUCH)  # Blocks ~1 s; robot is stopped.
                self.straight(-BUMP_BACKUP, BUMP_SPEED)
                if self.touch.pressed():
                    self.straight(-BUMP_BACKUP, BUMP_SPEED)
                self.turn(BUMP_TURN)
                # The old corner lies on the wall just hit; circling it drove
                # straight back in. Assume the wall is now beside us instead.
                corner = self.body_point(SONAR_FORWARD, SONAR_LEFT + target)
                bumped = True
                filtered, derivative, valid_count = None, 0.0, 0
                gap_travel, orbit_yaw = None, None
                last_time = self.clock.time()
                last_px = self.body_point(POINT_FORWARD, POINT_LEFT)[0]
                continue

            if raw is None and self.travel - gap_travel < bridge:
                # Brief dropout: hold heading.
                hold = clamp(HEADING_KP * (gap_heading - self.yaw()), -18, 18)
                self.drive(SLOW_SPEED, hold)
            elif raw is None:
                # Convex corner: circle the last wall point seen.
                if orbit_yaw is None:
                    orbit_yaw, seen = self.yaw(), 0
                    # The front wall ends at the same place on both passes.
                    # Odometry X drifts (no wall corrects it), so if we reach
                    # that end again the hit point was missed: fix X and stop.
                    if east_px is None and abs(self.yaw() - EAST) < 30:
                        east_px = px
                    # on_front: without it the catch fired at the front wall's
                    # WEST end, rounding onto the wall for the final pass.
                    elif east_px is not None and on_front:
                        self.stop()
                        print("Front wall ended: passed hit point, X off by",
                              int(east_px - px), "mm")
                        self.x += east_px - px
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
            print("APPROACH")
            robot.straight(APPROACH_LIMIT, APPROACH_SPEED, True, FAST_APPROACH_DISTANCE)
            brick.sound.beep()  # Required: obstacle found (in contact).
            # Contact establishes the front wall's Y coordinate.
            wall_y = robot.body_point(POINT_FORWARD, POINT_LEFT)[1]
            print("BACK UP")
            robot.straight(-(INITIAL_AXLE_CLEARANCE - POINT_FORWARD), 50.0)
            if robot.touch.pressed():
                raise RuntimeError("Front touch stuck after backing up")
            print("TURN RIGHT")
            robot.turn(90.0)
            robot.follow(wall_y)
            print("RETURN")
            robot.go_to(START_X, START_Y, RETURN_SPEED)
            print("Done at", robot.clock.time(), "ms")
        robot.stop()
        robot.running = False
        brick.sound.beep()  # Only a completed task/test gets the success beep.
    except Exception as error:
        robot.stop()
        print("STOPPED:", error)
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
