# Lab 3 Obstacle Tracing Handoff

Nov 9, 2026 · Tanish Bachu · Live version with diagram: https://claude.ai/artifact/PKz1RnyVvKDFdZYy1CqE4w

## Status

The robot completes the lab and returns within 5–15 cm of the start when the front wall has an outside corner just past the hit point. Where the front wall is very short or ends in an inside corner, it falls back on dead reckoning and misses by roughly 20–50 cm.

- **Code that runs on the robot:** `main.py`. It's the newest version, but the last two changes in it haven't been tested on the robot yet (see Known issues).
- **Last version tested and known to work:** `main_working_v3.py`.
- **Hardware:** EV3 running pybricks-micropython (old `ev3brick` API). Motors on B (left) and C (right). Sensors: touch on S1, gyro on S2, ultrasonic on S4, mounted on the left side.
- **Task:** drive forward to the obstacle, beep, follow it all the way round with the wall on the left and the measuring point within 30 cm, then return to the start at (2.0 m, 0.5 m), all in under 2 minutes.

## Files

All files are in `COMP581_Assignment_3/`. Only `main.py` runs on the robot; the others are saved copies.

| File | What's in it | Tested? |
| --- | --- | --- |
| `main.py` | `main_working_v3.py`, plus the 35° front-wall check and the two stop rules for the wall-end search | No |
| `main_working_v3.py` | 90° bump turn, faster speeds, wider wall-end search, 12 s return reserve | Yes, within 5–15 cm from start points with a clear front-wall corner |
| `main_working_v2.py` | Same, but turns 60° after a bump | Yes, 10–20 cm |
| `main_working.py` | First version with the front-wall-end landmark | Yes, 5–10 cm on one run |
| `main_landmark.py` | Early landmark version, before the fixes for missing the end of the lap | Partly |
| `main_latest.py` | Ouch sound, gyro drift fix, faster speeds; no landmark | Ran out of time or drifted 50 cm or more |
| `main_backup.py` | Code before the ouch sound: bump recovery, 25° limit on turning into the wall, gyro startup check | Drifted about 50 cm |

To go back to a saved version, copy it over `main.py` and download the project to the brick again.

## How a run works

The robot traces the obstacle by sonar, then trusts the front wall's end, not its position estimate, to know where it is before returning.

1. **Drive to the wall.** Straight ahead until the touch sensor hits, then beep.
2. **Back up and turn right.** Back 94 mm, then turn 90° right, so the wall is on the left.
3. **Follow the wall.** The sonar keeps the measuring point about 20 cm out. On a bump it backs up and turns 90° right. On the first pass it records where the front wall ends (the landmark).
4. **Lap done.** The lap counts as done once the position estimate says the robot is back at the hit point's X.
   - **No landmark recorded:** return to the start straight away.
   - **Landmark recorded:** keep following past the estimated hit point, looking for the wall end.
5. **Wall end found:** fix X from the first pass (it was off by 0.3–0.8 m in testing), then return.
6. **Search gives up** on a bump, a turn of more than 60° off east, the 105 s mark, or 80 cm past the expected end. It then returns using the estimate.
7. **Return to start.** Turn away from the obstacle and drive to (2.0 m, 0.5 m).

All of this is in `follow()`. The front-wall heading and Y correction runs whenever the robot follows the front wall steadily on the final pass. With no landmark (short or rounded front wall), it returns using the estimate alone.

## What testing taught us

Over a lap, the robot's estimate of its X position along the front wall drifts by 30–80 cm, in either direction. That drift is why the front-wall-end landmark exists.

- **Where position comes from.** Distance comes from the wheel encoders and direction from the gyro (`update()`). Nothing corrects X during the lap. Measured errors at the landmark were +637, −305, +778, −352, +601 and +739 mm.
- **What causes the drift.** Bumps and scrapes against the wall. The encoders count wheel turns that don't move the robot that way, and the body gets pushed sideways. It's worst when the sonar reads under 50 mm after a corner.
- **Gyro.** It's usually right within 5° by the end of a lap. One lap with four bumps and a lot of circling ended 25° off.
- **The encoder heading** (`encoder_yaw` in the log) is only for the stuck check and for comparison. It slips by 30–100° during turns in place.
- **Gyro drift at startup.** A single 1° tick while waiting used to be read as 0.24°/s of drift, adding about 23° of false heading per lap. It's now ignored.
- **Bumps cost time.** Each one takes about 4–4.5 s, with the ouch pause, the back-up and the turn. A 60° turn after a bump often hit the same inside corner two or three times. 90° cut a lap from six bumps to three.
- **Time.** Good runs finish in 98–110 s, against the code's 117 s limit (`RUN_LIMIT`; the task allows 120 s). The return takes 8–11 s, so 12 s is reserved for it.
- **The ouch sound** stays, with its 1 s pause before backing up. Playing it in a separate thread to skip the pause made runs worse.

## Test results

When the landmark fired, the robot ended 5–20 cm from the start. Every miss of 28 cm or more came from a run that returned using only its position estimate. Runs below are from 2026-11-09, newest first, from several start points around the course.

| # | Code version | How the lap ended | X error caught | Done at | Bumps | Distance from start |
| --- | --- | --- | --- | --- | --- | --- |
| 10 | `main.py`, before the two stop rules | Short front wall; kept searching, bumped, hit the workspace edge | none | stopped at 99.7 s | 4 | way off, no return |
| 9 | `main_working_v3.py` | Didn't recognise the front wall (gyro 25° off); wall end not found | none | 114.8 s | 4 | about 50 cm |
| 8 | `main_working_v3.py` | Not logged | — | — | — | 30 cm or more |
| 7 | `main_working_v3.py` | Landmark | +601 mm | 97.8 s | 3 | about 15 cm |
| 6 | `main_working_v3.py` | Landmark | −352 mm | 100.4 s | 3 | 5–10 cm |
| 5 | `main_working_v2.py` | Time guard at 105 s | none | 113.7 s | 6 | about 28 cm |
| 4 | `main_working_v2.py` | Landmark | +778 mm | 103.1 s | 5 | 5–10 cm |
| 3 | `main_working_v2.py` | Landmark | −305 mm | 106.3 s | 5 | 10–20 cm |
| 2 | `main_working.py` + search limit of +30 cm | Gave up 30 cm short of the wall end | none | 114.8 s | 5 | 40–50 cm |
| 1 | `main_working.py` | Landmark | +637 mm | 110.5 s | 5 | 5–10 cm |

Run 10 led to the two stop rules now in `main.py`. Run 9 led to the 35° front-wall check.

## Settings to tune

These are the constants worth changing, with their values in `main.py` and what to try if a run shows the symptom. Change one at a time and keep a copy that works.

| Constant | Now | Change it when |
| --- | --- | --- |
| `FOLLOW_SPEED` / `SLOW_SPEED` | 150 / 100 mm/s | Running out of time: raise them. Losing the wall on straights: lower them. |
| `ORBIT_SPEED` | 85 mm/s | Swings wide or loses the wall at outside corners: go back to 70–80. |
| `ORBIT_RADIUS` | 215 mm | Sonar under 50 mm after corners (scraping): raise it. The measuring point must stay under 30 cm; the startup check refuses values that break it. |
| `TURN_RATE` | 75°/s | Turns in place overshoot or wobble: lower to 60. |
| `BUMP_TURN` | 90° | Bumps the same corner twice: raise it. Wanders away from the wall after a glancing bump: try 75. |
| `BUMP_BACKUP` / `BUMP_SPEED` | 100 mm at 150 mm/s | Scrapes the wall while turning after a bump: back up further. |
| `TARGET_POINT_CLEARANCE` | 200 mm | Measuring point needs more margin under the 30 cm rule: try 160 (expect more bumps). |
| `RETURN_RESERVE` | 12,000 ms | The return gets cut off by the run limit: raise it. |
| `RUN_LIMIT` | 117,000 ms | Leave as is: it leaves 3 s of margin under the 120 s task limit. |
| Front-wall heading window (inside `follow()`) | 35° | Landmark fires on the wrong wall: lower it. Never recognises the front wall: raise it. |
| Wall-end search limit (inside `follow()`) | expected distance + 800 mm | Gives up just before the real wall end: raise it. |

## Known issues and next steps

The first job is to test the newest changes in `main.py` from the start points behind runs 9 and 10.

- [ ] Test the 35° front-wall check from run 9's start point. Look for `Front wall ended` before `RETURN`.
- [ ] Test the two stop rules from run 10's start point (short front wall). It should print `Bumped while looking for the wall end` or `Wall end not found`, then return.
- [ ] Watch for the landmark firing on the wrong wall. That shows up as `Front wall ended` while the robot is clearly not at the end of the front wall. If it happens, lower the 35° window.
- [ ] Short or rounded front walls give no usable landmark, so those runs miss by 20–50 cm. The only fix so far is fewer bumps and less scraping.
- [ ] Check the 30 cm rule at outside corners. Wide arcs (sonar showing `None` for 2–5 s) may take the measuring point past 30 cm.
- [ ] Time is tight on laps with 5–6 bumps. Each bump costs about 4 s, about 1 s of it the ouch pause.
- [ ] The Points section of the assignment hasn't been read yet. It decides whether accuracy at the start or staying under 2 minutes matters more.

## How to run it and read the log

Run it from VS Code with the EV3 MicroPython extension. A run started from the brick's own menu prints nothing you can save.

1. Download the project to the brick, so it gets the current `main.py`.
2. Put the robot on the start point and let go. Start the program from the laptop.
3. Wait for `Ready. Press CENTER.`, then leave it still for about 10 s so the gyro drift reading is good.
4. Press CENTER. DOWN aborts at any time.

The `MODE` constant at the top switches between the full run (`RUN`) and the test modes. `CHECK` prints the sensors without moving; `STRAIGHT` and `TURN` drive 500 mm or turn 90° to check calibration.

While following, the log prints a row every 500 ms: `time_ms, followed, sonar, filtered_out, x, y, yaw, encoder_yaw`. A `*` marks a sonar reading the filters threw out, and `None` means no echo.

| Last line before `RETURN` | Meaning | Expected accuracy |
| --- | --- | --- |
| `Front wall ended: passed hit point, X off by N mm` | The landmark fired; X was fixed from the front wall's end | 5–20 cm |
| `Back at hit point: ...` | No landmark from this start point; stopped using the estimate | 20–50 cm |
| `Wall end not found; stopping by odometry` | Ran out of search distance or time, or turned more than 60° off east | 20–50 cm |
| `Bumped while looking for the wall end; ...` | Hit something past the hit point; returned from there | 20–50 cm |
| `STOPPED: ...` with no `RETURN` | A safety stop: run limit, workspace edge, stuck wheels, or DOWN pressed | no return |
