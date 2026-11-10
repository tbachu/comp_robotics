from pybricks._world import WORLD


class Motor:
    # Speed changes instantly; angle() counts what the wheel turned (with slip).
    def __init__(self, port, direction=None):
        self.port = port

    def run(self, speed):
        WORLD.speed[self.port] = float(speed)

    def stop(self, stop_type=None):
        WORLD.speed[self.port] = 0.0

    def brake(self):
        WORLD.speed[self.port] = 0.0

    def angle(self):
        return int(round(WORLD.enc[self.port]))


class TouchSensor:
    def __init__(self, port):
        pass

    def pressed(self):
        return WORLD.touch()


class GyroSensor:
    # Whole degrees, clockwise positive, no drift.
    def __init__(self, port):
        pass

    def angle(self):
        return int(round(WORLD.psi - WORLD.gyro_zero))

    def reset_angle(self, angle):
        WORLD.gyro_zero = WORLD.psi - angle

    def speed(self):
        return 0


class UltrasonicSensor:
    def __init__(self, port):
        pass

    def distance(self):
        return WORLD.sonar()
