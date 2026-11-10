# Lab 3 Obstacle Tracing Handoff

Nov 9, 2026 · Jim Nguyen · Follows Tanish's handoff. The live page linked from that one (https://claude.ai/artifact/PKz1RnyVvKDFdZYy1CqE4w) describes the code before the changes below.

## Status

The code on the robot is `main.py` on branch `jims_side` (commit `386aac6`). It fixes four problems seen on the robot on Nov 9:
- it kept tracing past the start;
- it stopped 5–15 cm short of the start;
- it drove straight past a zigzag, far from the walls;
- it ended a lap one panel early and backed far away.

The last fix has only been checked in a desk simulation, not on the robot yet.

- **Branches:** `main` still has the code from before these changes (`88203d2`). Merge `jims_side` into `main` once it has been tested on the robot.
- **Last version known to work on the robot:** `main_working_v3.py`. It returns within 5–15 cm from start points where the front wall has a clear outside corner just past the hit point.
- **Hardware:** EV3 running pybricks-micropython (old `ev3brick` API). Motors on B (left) and C (right). Sensors: touch on S1, gyro on S2, ultrasonic on S4, mounted on the left side.
- **Task:** drive forward to the obstacle and beep. Follow it all the way round with the wall on the left and the measuring point (tip of the touch rod) within 30 cm. Then return to the start at (2.0 m, 0.5 m). All in under 2 minutes.

## What changed on Nov 9

Each fix came from a run that went wrong on the robot. The commits are on `jims_side`.

| Commit | What the robot did | Cause | Fix |
| --- | --- | --- | --- |
| `3ebdbfa` | From some start spots it went past the start and kept rounding corners | The lap only ended once the position estimate reached the hit point. X drifts 30–80 cm a lap, and every backup stop rule waited on that estimate | The final pass is recognised by heading (gyro), not X. Reaching the starting wall's far end in any way ends the lap. Inside corners became landmarks too. Every run is saved to a log file |
| `356d010` | Stopped 5–15 cm short of the start | The return steered the rod tip onto the start point. The robot arrives facing the opposite way, so its body ended about 2 × 106 mm back | Returns to the full start pose: backs its axle to where the axle started, then turns to face the obstacle |
| `356d010` | Drove straight past a zigzag, 30–40 cm from the walls | The ±25° heading limit stayed anchored to the heading where the wall was picked up mid-corner, so the robot could not turn parallel to the next wall | The limit now re-aims at the wall's direction, worked out from how fast the sonar range changes |
| `386aac6` | Ended a lap one panel before the starting panel and backed straight away from it | The code assumed the starting panel runs east. On the zigzag obstacle it was slanted, so the panel before it also matched, and resetting the heading to "east" bent it by the slant | The starting panel's direction is measured on the first pass and used to recognise it. Bumps in zigzag notches no longer end the lap. Only real corners count as the panel's end. The end fixes the position both along and across the wall |

## Files

All files are in the repo folder (`comp_robotics/`). Only `main.py` runs on the robot; the `main_*.py` files are saved copies.

| File | What's in it | Tested on the robot? |
| --- | --- | --- |
| `main.py` | Everything above | `3ebdbfa` and `356d010` versions yes; `386aac6` not yet |
| `fetch_runs.sh` | Copies run logs off the brick into `runs/` | — |
| `sim/` | Desk simulator: runs `main.py` unchanged against simulated obstacles. `python3 sim/run.py`; see `sim/README.md` | — |
| `runs/sim/` | Saved simulated runs of `356d010` against `386aac6` on 15 obstacle shapes. Start with `SUMMARY.txt`, then `paths_slanted_after.png` | — |
| `main_working_v3.py` | 90° bump turn, faster speeds, wider wall-end search, 12 s return reserve | Yes, 5–15 cm with a clear front-wall corner |
| `main_working_v2.py` | Same, but turns 60° after a bump | Yes, 10–20 cm |
| `main_working.py` | First version with the front-wall-end landmark | Yes, 5–10 cm on one run |
| `main_landmark.py` | Early landmark version | Partly |
| `main_latest.py` | Ouch sound, gyro drift fix, faster speeds; no landmark | Ran out of time or drifted 50 cm+ |
| `main_backup.py` | Before the ouch sound: bump recovery, 25° turn-in limit, gyro startup check | Drifted about 50 cm |

To go back to a saved version, copy it over `main.py` and download the project to the brick again. Any commit can also be checked out with git.

## How a run works

The robot traces the obstacle by sonar. Its position estimate drifts, so before going home it re-locates on the far end of the panel it first hit (the landmark).

1. **Drive to the wall.** Straight ahead until the touch sensor hits, then beep. That contact point is on the starting panel.
2. **Back up and turn right.** Back 94 mm, then turn 90° right, so the wall is on the left.
3. **First pass.** Follow the wall with the measuring point about 20 cm out. On a bump, back up and turn 90° right.
   - **Measure the starting panel.** Its direction comes from the contact point to the sonar's points on it, so a slanted panel is fine.
   - **Record the landmark.** This is where that side of the obstacle ends. Either the sonar loses the wall at an outside corner, or the touch sensor hits the next wall at an inside corner.
   - **Real corners only.** An outside corner counts only if the robot circled 30°+ before picking a wall up again, or turned 60°+ in all. Zigzag bends turn less.
4. **Back on the starting panel.** After a full lap the gyro heading comes back within 25° of the panel's measured heading. That is how the robot knows it is on its final pass, not from X.
5. **Panel end reached.** This point is past the start.
   - **Same kind of corner as on the first pass:** fix the position from it and return. The corner fixes it along the wall, and the sonar range fixes it across the wall. If the robot was already round onto the next wall, it backs out to the corner's outside first, so the way home does not clip the corner.
   - **Turned 60°+ past the panel's heading with no match:** return using the estimate.
   - **A bump before the expected end** (a notch in a zigzag) is handled as a normal bump.
6. **Fallbacks using the estimate.** These apply once the robot is round the last corner before the starting panel:
   - **No landmark recorded:** return when the estimate reaches the hit point, but only while heading along the starting panel.
   - **Landmark recorded but not found:** give up 80 cm past where it should be.
   - **Short on time:** past about 102 s, or earlier when far from the start, return straight away.
7. **Return to start.** Drive the axle back to where the axle started, usually reversing in, then turn to face the obstacle as at the start. The whole robot ends in its starting pose, not just the rod tip.

All of this is in `follow()` and `go_home()` in `main.py`. The landmark handling is at the end of `follow()`'s loop, and the panel measurement is in the `StartPanel` class.

## What testing taught us

- **Position drift.** Distance comes from the wheel encoders and direction from the gyro (`update()`). Over a lap, X along the front wall drifted 30–80 cm in either direction. Measured at the landmark: +637, −305, +778, −352, +601, +739 mm. That is why the robot re-locates on the landmark before going home.
- **What causes the drift.** Bumps and scrapes: the encoders count wheel turns that don't move the robot that way. Fewer bumps means less drift.
- **Gyro.** Usually within 5° after a lap. One lap with four bumps and a lot of circling ended 25° off. A single 1° tick at startup used to be read as drift, adding about 23° of false heading per lap. It's now ignored.
- **The encoder heading** (`encoder_yaw` in the log) is only used for the stuck check. It slips 30–100° during turns in place.
- **Bumps cost time.** About 4–4.5 s each, including the 1 s ouch pause, the back-up and the turn. A 90° bump turn cut laps from six bumps to three. Playing the ouch sound in a thread to skip the pause made runs worse.
- **Measure the robot, not just the rod tip.** The rod tip is 106 mm in front of the axle. Returning the tip to the start left the body 15–25 cm short, because the robot came back facing the other way.
- **Slanted panels.** A zigzag obstacle means the panel the robot hits can be slanted. Anything that assumes "the front wall runs east" breaks there.

## Test results

**On the robot.** Runs 1–10 are Tanish's, with logs. Runs 11–14 are from Nov 9 evening, observed but not logged. The robot runs all happened on Nov 9; the number order isn't strictly newest first.

| # | Code version | How the lap ended | X error caught | Done at | Bumps | Distance from start |
| --- | --- | --- | --- | --- | --- | --- |
| 14 | `356d010` | Ended one panel before the starting panel, then backed straight away from that panel | — | — | — | far off |
| 13 | `3ebdbfa` | Drove straight past about 5 zigzag walls, far from them | — | — | — | about 20 cm |
| 12 | `3ebdbfa` | Landmark | — | — | — | 5–15 cm short (rod tip was on the start; body short) |
| 11 | `88203d2` | Went past the start and kept rounding corners | — | — | — | far off |
| 10 | `main.py`, before the two stop rules | Short front wall; kept searching, bumped, hit the workspace edge | none | stopped at 99.7 s | 4 | way off, no return |
| 9 | `main_working_v3.py` | Didn't recognise the front wall (gyro 25° off) | none | 114.8 s | 4 | about 50 cm |
| 8 | `main_working_v3.py` | Not logged | — | — | — | 30 cm or more |
| 7 | `main_working_v3.py` | Landmark | +601 mm | 97.8 s | 3 | about 15 cm |
| 6 | `main_working_v3.py` | Landmark | −352 mm | 100.4 s | 3 | 5–10 cm |
| 5 | `main_working_v2.py` | Time guard at 105 s | none | 113.7 s | 6 | about 28 cm |
| 4 | `main_working_v2.py` | Landmark | +778 mm | 103.1 s | 5 | 5–10 cm |
| 3 | `main_working_v2.py` | Landmark | −305 mm | 106.3 s | 5 | 10–20 cm |
| 2 | `main_working.py` + search limit of +30 cm | Gave up 30 cm short of the wall end | none | 114.8 s | 5 | 40–50 cm |
| 1 | `main_working.py` | Landmark | +637 mm | 110.5 s | 5 | 5–10 cm |

**In the desk simulation** (`sim/`, results saved in `runs/sim/`). This is a 2D model of the robot, sonar, touch sensor and wheel slip that runs the real `main.py` unchanged. It covered 15 shapes, including box corners, inside corners, a sawtooth, a staircase and slanted zigzag panels, with three sonar beam models:

| | Runs ending more than 15 cm off (of 45) | Median distance from start | Worst |
| --- | --- | --- | --- |
| `356d010` | 14 | 37 mm | 96 cm |
| `386aac6` (now) | 11 | 28 mm | 66 cm |

With the middle sonar model, box shapes end 0.3–6 cm from the start, except a very deep sawtooth (36 cm). Four of the five slanted-panel cases end within 0–3 cm; the fifth ends 56 cm off. Which cases fail depends on how wide the sonar beam is, and the real one hasn't been measured. To rerun: `python3 sim/run.py --rev 356d010 --rev HEAD --sonar narrow` (also `medium`, `wide`).

## Settings to tune

The constants worth changing, with their values in `main.py`. Change one at a time and keep a copy that works.

| Constant | Now | Change it when |
| --- | --- | --- |
| `FOLLOW_SPEED` / `SLOW_SPEED` | 150 / 100 mm/s | Running out of time: raise them. Losing the wall on straights: lower them. |
| `ORBIT_SPEED` | 85 mm/s | Swings wide or loses the wall at outside corners: go back to 70–80. |
| `ORBIT_RADIUS` | 215 mm | Sonar under 50 mm after corners (scraping): raise it. The startup check refuses values that break the 30 cm rule. |
| `TURN_RATE` | 75°/s | Turns in place overshoot or wobble: lower to 60. |
| `BUMP_TURN` | 90° | Bumps the same corner twice: raise it. Wanders off after a glancing bump: try 75. |
| `BUMP_BACKUP` / `BUMP_SPEED` | 100 mm at 150 mm/s | Scrapes the wall while turning after a bump: back up further. |
| `TARGET_POINT_CLEARANCE` | 200 mm | Measuring point needs more margin under the 30 cm rule: try 160 (expect more bumps). |
| `APPROACH_ANGLE` / `FENCE_RATE` | 25° / 10 mm/s | Still drives straight away from a wall after a corner: lower `FENCE_RATE` to 6–8. Turns its nose into walls: raise it. |
| `FRONT_HEADING` | 25° | Ends the lap on the wrong panel: lower it. Never prints `Back on the starting panel`: raise it. |
| `CORNER_CONFIRM` / `CORNER_TURN` | 30° / 60° | Never prints `Starting panel ends in an outside corner` at a real corner: lower them. Takes a zigzag bend as the landmark: raise them. |
| `PANEL_SPAN` / `PANEL_BEND` | 200 mm / 10° | Heading or position corrections on a slanted starting panel make the return worse: raise `PANEL_SPAN`. |
| `WALL_RUN_MIN` | 100 mm | Landmark fires while rounding onto the starting panel: raise it. |
| `PAST_TURN` | 60° | `Turned past the front wall's end` while still on the starting panel: raise it to 70–75. |
| `SEARCH_MARGIN` | 800 mm | Gives up just before the real panel end: raise it. |
| `MAX_X_FIX` | 1,000 mm | A correct landmark is rejected as `too far`: raise it. |
| `RETURN_RESERVE` | 15,000 ms | Minimum time kept for the return; more is kept when far from the start. Raise it if the return gets cut off or the log says `No time to turn to the start heading`. |
| `RUN_LIMIT` | 117,000 ms | Leave as is: 3 s of margin under the 120 s task limit. |

## Next steps

- [ ] **Test `386aac6` on the robot.** Use the start spots behind runs 11–14: the slanted panel that ended a panel early, the zigzag, the spot that stopped short, and the spot that went past the start. For each run, measure the distance from the start using the robot's body, not just the rod tip. Also note the finish time, then run `./fetch_runs.sh`.
- [ ] **If a run goes wrong, read its log.** These lines show which rule fired where:
  - `Starting panel heading ...`: whether the panel was measured;
  - `Starting panel ends in ...`: whether a landmark was recorded;
  - `Back on the starting panel`;
  - the last line before `RETURN`.
- [ ] **Watch the finish time on slanted zigzag fronts.** The lap now runs on to the real corner past the start and comes back via that corner. Simulated runs took 89–109 s (box shapes 57–86 s).
- [ ] **Shallow corners.** Where the starting panel's side ends in a shallow corner (about 40°, like a zigzag bend), no landmark is recorded and the return uses the estimate (20–50 cm). One pass can't tell such a corner from a zigzag bend.
- [ ] **Deep, sharp zigzags.** Teeth over about 20 cm deep still take the measuring point past 30 cm for a few seconds.
- [ ] **Landmark accuracy.** If the robot is still closing in on the wall when the sonar loses the corner, the landmark fix can be 5–12 cm off. Correcting that needs the real sonar's beam width. Measuring it in `CHECK` mode would also show which simulation results to trust.
- [ ] **One shape stops early.** If the obstacle's outline turns more than 60° past a full lap before reaching the starting panel (for example an L with a leg sticking out south, west of the hit point), the lap stops early and returns using the estimate.
- [ ] **Before each robot test,** run `python3 sim/run.py` with all three sonar models after any change to `main.py`, and compare with the previous commit (`--rev`). It catches logic mistakes in seconds. It can't confirm accuracy: the model leaves out scrapes, gyro error and sensor noise (see `sim/README.md`).
- [ ] **Merge `jims_side` into `main`** once it works on the robot.
- [ ] **Read the assignment's Points section.** It decides whether accuracy at the start or staying under 2 minutes matters more.

## How to run it and read the log

Run it from VS Code with the EV3 MicroPython extension.

1. Download the project to the brick, so it gets the current `main.py`.
2. Put the robot on the start point and let go. Start the program from the laptop.
3. Wait for `Ready. Press CENTER.`, then leave it still for about 10 s so the gyro drift reading is good.
4. Press CENTER. DOWN aborts at any time.

The `MODE` constant at the top switches between the full run (`RUN`) and the test modes. `CHECK` prints the sensors without moving; `STRAIGHT` and `TURN` drive 500 mm or turn 90° to check calibration.

While following, the log prints a row every 500 ms: `time_ms, followed, sonar, filtered_out, x, y, yaw, encoder_yaw`. A `*` marks a sonar reading the filters threw out; `None` means no echo. Other lines to look for:

| Line | Meaning |
| --- | --- |
| `Starting panel heading N deg, measured over M mm` | Direction of the panel it hit. `too short to correct to` means it will recognise the panel by it but not correct heading or position to it. |
| `Starting panel ends in an outside corner` / `inside corner N mm past the hit point` | The landmark was recorded on the first pass. |
| `Back on the starting panel: x y` | Final pass recognised. |
| `Home: via x y`, then `Home: axle to ...` | Backing out to the corner's outside, then going home (`reversing` or `forward`). |

The last line before `RETURN` says how the lap ended:

| Last line before `RETURN` | Meaning | Expected accuracy |
| --- | --- | --- |
| `Front wall ended (outside corner): off by N mm along the wall and M mm across it` | Corner landmark matched; position fixed from it | 5–20 cm |
| `Front wall ended (inside corner): off by ...` | Inside-corner landmark matched | 5–20 cm (desk-tested only) |
| `... off by N mm along the wall is too far; stopping by odometry` | A landmark matched, but the fix was over 1 m, so it was ignored | 20–50 cm |
| `Back at hit point: N mm along the panel, followed M mm` | No landmark from this start point; stopped using the estimate | 20–50 cm |
| `Turned past the front wall's end; stopping by odometry` | Rounded the far end with no landmark match | 20–50 cm |
| `Wall end not found; stopping by odometry` | Went 80 cm past where the landmark should be | 20–50 cm |
| `Out of time; stopping by odometry` | Short on time on the final pass | 20–50 cm |
| `Bumped while looking for the wall end; stopping by odometry` | Bumped past where the panel should end | 20–50 cm |
| `STOPPED: ...` with no `RETURN` | A safety stop: run limit, workspace edge, stuck wheels, or DOWN pressed | no return |

### Run logs

Every run in `RUN`, `STRAIGHT` or `TURN` mode saves two files in `runs/` next to `main.py` on the brick, numbered `run_001`, `run_002` and so on. This includes runs started from the brick's own menu.

- **`run_NNN.txt`:** everything the console shows. It starts with the settings used and ends with a `RESULT` line, which gives:
  - done or stopped, and the time;
  - how the lap ended, and the landmark fix along the wall;
  - the estimated end pose (measuring point, axle, heading), next to the start pose.
- **`run_NNN.csv`:** one row per control step (about 20 a second), for plotting the path.
  - Columns: `t_ms,phase,x,y,yaw,encoder_yaw,px,py,sonar`.
  - `x, y` is the axle estimate and `px, py` the measuring point estimate, in mm.
  - `phase` is APPROACH, BACKUP, TURN, FOLLOW, BUMP or RETURN.

To copy them to the laptop, run `./fetch_runs.sh` from the repo folder with the brick connected (password `maker`). They land in `runs/`. `.vscode/settings.json` keeps `runs/` from being downloaded back to the brick.
