# Desk simulator

Runs the robot program (`main.py`, unchanged) on the laptop against simulated obstacles. It reports where the robot ended up compared with where it started. Fake `pybricks` modules stand in for the EV3, backed by a simple 2D model of the robot, the sonar, the touch sensor and wheel slip.

Use it to check the logic before a robot run. It shows when the lap ends, which rule fires, whether the landmark matches, and the path home. Don't use it to tune speeds or gains: the model is much simpler than the real robot (see below).

This folder is never downloaded to the brick (`.vscode/settings.json` excludes it).

## Running it

Needs Python 3 on the laptop. `--plot` also needs matplotlib (`pip install matplotlib`). Run from the repo folder:

```
python3 sim/run.py                          # current main.py, all shapes, medium sonar
python3 sim/run.py --list                   # list the shapes
python3 sim/run.py d_baseline s2_slant_ese_lag
python3 sim/run.py --rev 356d010 --rev HEAD # compare two commits' main.py
python3 sim/run.py --code main_working_v3.py --sonar wide --plot
```

The 15 default shapes take a few seconds per version.

## Reading the table

| Column | Meaning |
| --- | --- |
| `body` | True distance of the robot's axle from where the axle started, in mm. This is what you see when you look at the robot. |
| `tip` | True distance of the measuring point (touch-rod tip) from (2000, 500), in mm. |
| `head` | True final heading; 0 is facing the obstacle, as at the start. |
| `wall max` | Worst true distance of the measuring point from the obstacle while following, in mm. The task allows 300. |
| `>30cm` | Seconds the measuring point spent more than 300 mm from the obstacle. |
| `bumps`, `time` | Bumps in the lap, and the task time from the `RESULT` line. |
| `how the lap ended` | The program's own log line. The log table in `HANDOFF.md` explains each one. |

`wall max`, `>30cm` and `time` need the run logs that `main.py` has written since commit `3ebdbfa`. Older versions show `-`.

Each run is saved in `sim/out/<version>_<sonar>/<scenario>/`:
- `console.txt`: everything the program printed;
- `runs/run_001.txt` and `.csv`: the program's own run log, as it would be on the brick;
- `truth.csv`: the true path.

`sim/out/summary_<sonar>.txt` has the last table, and `--plot` adds `paths.png`. `sim/out/` is not committed.

## The shapes

Defined in `scenarios.py`; add your own there. Each one is:
- **polygons:** the obstacle outline in mm. The robot starts with its measuring point at (2000, 500) facing +Y, so it hits whatever is directly north of x = 2000.
- **slip:** optional wheel slip. `back_west(y, k)` makes the encoders count k times the real motion while the robot heads west along the back face. That makes the position estimate lag (k > 1) or run ahead (k < 1) along the front wall, like the 300–780 mm drift seen on the robot.
- **a description.**

## Sonar models (`--sonar`)

The real EV3 sonar's beam hasn't been measured, and some results depend on it, so there are three models:

| Model | Beam half-angle | Steepest surface that still echoes |
| --- | --- | --- |
| `narrow` | 8° | 15° |
| `medium` (default) | 10° | 25° |
| `wide` | 15° | 40° |

A wider beam picks walls up earlier when rounding corners. Check a change against all three.

## What the model leaves out

- **Only the touch tip hits walls.** The rest of the body passes through them, so scrapes don't happen. The only slip is the slip a shape adds.
- **The gyro is perfect.** Whole degrees, no drift, no scale error.
- **Motors change speed instantly.** The program's loop takes 50 ms plus 10 ms per wait, and nothing else costs time apart from the 1 s ouch sound.
- **The sonar is ideal.** It returns the nearest echo within the beam from any surface hit at a shallow enough angle. There's no noise and no multipath.
- **CENTER is pressed once at 14 s, and DOWN never.**

So a good result here is necessary, not sufficient. Confirm it on the robot and compare with its run log (`./fetch_runs.sh`).

## Files

- `run.py`: runs the shapes and prints the table.
- `run_one.py`: runs one program in one shape. `run.py` starts it as a fresh process each time.
- `plot.py`: draws the true paths over the obstacles.
- `scenarios.py`: the shapes.
- `fake/pybricks/`: the stand-in `pybricks` package. `_world.py` holds the model.
