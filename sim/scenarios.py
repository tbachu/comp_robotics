# Obstacle shapes for the desk simulator.
# Each entry: name -> (polygons in mm, slip or None, what it tests).
# The robot starts with its measuring point at (2000, 500) facing +Y, so it
# hits whatever is directly north of x = 2000.


def back_west(y_min, k):
    # Wheel slip on the obstacle's back face: while heading west north of
    # y_min, the encoders count k times the real motion. k > 1 makes the
    # position estimate lag along the front wall (the usual case on the
    # robot: 600-780 mm), k < 1 makes it run ahead.
    def slip(w):
        west = abs(((w.psi - 270.0) + 180.0) % 360.0 - 180.0) < 45.0
        return k if (west and w.Y > y_min) else 1.0
    return slip


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def sawtooth(teeth_out, rise, top):
    # Box with its east face a sawtooth of 5 teeth.
    pts = [(1400, 1100), (2300, 1100)]
    for i in range(5):
        pts += [(2300 + teeth_out, 1100 + rise // 2 + rise * i), (2300, 1100 + rise * (i + 1))]
    return pts + [(1400, top)]


def slanted(shift):
    # South face is a zigzag of 300 mm panels sloped +/-20 deg.
    xs = [1200, 1500, 1800, 2100, 2400, 2700]
    ys = [1150, 1041, 1150, 1041, 1150, 1041]
    return [(x + shift, y) for x, y in zip(xs, ys)] + [(2700 + shift, 1900), (1200 + shift, 1900)]


SCENARIOS = {
    "a_short_wall_lag": ([rect(1750, 1100, 2200, 1900)], back_west(1950, 1.8),
                         "short front wall, hit 200 mm from its outside corner; estimate lags"),
    "b_inside_corner_lag": ([[(1400, 1100), (2300, 1100), (2300, 800), (2600, 800),
                              (2600, 1700), (1400, 1700)]], back_west(1750, 1.6),
                            "front wall ends in an inside corner 300 mm east; estimate lags"),
    "c_ahead_west_end": ([rect(1900, 1100, 2600, 1700)], back_west(1750, 0.7),
                         "estimate runs ahead; hit 100 mm from the front wall's west end"),
    "d_baseline": ([rect(1400, 1100, 2300, 1700)], None, "plain box, no slip"),
    "e_no_obstacle": ([], None, "nothing to hit: checks a STOPPED run still logs"),
    "f_long_wall_lag": ([rect(1400, 1100, 2300, 1700)], back_west(1750, 1.5),
                        "plain box; estimate lags"),
    "g_tiny_wall_lag": ([rect(1850, 1100, 2100, 1800)], back_west(1850, 1.8),
                        "250 mm front wall; estimate lags"),
    "h_near_inside_corner_ahead": ([[(1400, 1100), (2150, 1100), (2150, 850), (2500, 850),
                                     (2500, 1700), (1400, 1700)]], back_west(1750, 0.75),
                                   "inside corner 150 mm east; estimate ahead"),
    "z1_sawtooth_east": ([sawtooth(150, 240, 2300)], None,
                         "east face is a sawtooth, teeth 150 mm deep"),
    "z2_staircase_back": ([[(1400, 1100), (2300, 1100), (2300, 1700), (2150, 1700),
                            (2150, 1850), (2000, 1850), (2000, 2000), (1850, 2000),
                            (1850, 2150), (1700, 2150), (1700, 2300), (1400, 2300)]], None,
                          "back face is a staircase of 150 mm steps"),
    "z3_deep_sawtooth_lag": ([sawtooth(250, 300, 2600)], back_west(2650, 1.4),
                             "east face is a sawtooth, teeth 250 mm deep; estimate lags"),
    "s1_slant_ese": ([slanted(0)], None,
                     "front is slanted zigzag panels; hits one sloping down to the east"),
    "s2_slant_ese_lag": ([slanted(0)], back_west(1950, 1.3), "as s1; estimate lags"),
    "s3_slant_ene": ([slanted(300)], None, "as s1, but hits a panel rising to the east"),
    "s4_slant_ene_lag": ([slanted(300)], back_west(1950, 1.3), "as s3; estimate lags"),
    "s5_slant_near_notch_lag": ([slanted(150)], back_west(1950, 1.3),
                                "as s1, hit 50 mm from a notch; estimate lags"),
}

# Run when no scenario is named (e_no_obstacle only checks logging).
DEFAULT = [name for name in SCENARIOS if name != "e_no_obstacle"]
