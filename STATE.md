# Team AVAA — project state

**Read this first when resuming.** Written 2026-09-02. Phase 1 deadline **2026-09-15**.

Nothing important lives in a chat transcript. Everything is in this folder, in the repo, or
in the git history — which carries the reasoning, not just the diffs.

---

## Repository

`https://github.com/AIsmail17/avaa-erc-2026` — branch **`avaa`**, private.
Local: `~/erc/erc_sim_2026` inside WSL (physically on D: via the VHDX).

The commit messages are the design record. `git log` is worth reading before changing
anything, because most non-obvious decisions have a measurement behind them.

## Documents in this folder

| File | What it holds |
|---|---|
| `NOTES.md` | Competition rules, scoring, deliverables |
| `SETUP.md` | Environment build, with every trap hit along the way |
| `PERCEPTION.md` | Colour and marker detection, measured accuracy, 3D localisation |
| `MANIPULATION.md` | Gripper curve, reach envelope, arm kinematics, tuck pose |
| `ORGANISER_QUESTIONS.md` | Six items. The committee's answer was that teams solve these themselves; kept as the record, and item 1 is corrected in place |
| `STATE.md` | This file |

---

## Read this before trusting any measurement

**The simulator does not run at a fixed speed, and a figure per second of wall clock is
not a figure about the robot.** Measured with `tools/rtf.py`:

| | real-time factor |
|---|---|
| freshly launched | 0.47 – 0.60 |
| after one `set_pose` teleport | 0.03 – 0.06 |

A teleport costs the simulation most of its speed, permanently — at every height tried,
with the robot resting level and unpenetrated, nothing in contact on any instrumented
link. Killing every node does not bring it back. Teleporting somewhere else does not
bring it back. Only relaunching does. **Driving costs nothing**: 0.471 untouched, 0.550
after driving eight simulated seconds, 0.561 after driving further.

`tools/place_robot.py` teleports, and it set up every grasp experiment before 2026-09-02.
So all of those ran at a fifteenth of the speed of a scored run, which never teleports —
the robot spawns once and drives. Several conclusions drawn under those conditions are
suspect and are flagged below.

Use `tools/drift.py` (per simulated second, prints the RTF beside the answer) and
`tools/rtf.py`. Do not use a stopwatch to wait for anything the simulator does.

---

## Where the work stands

| Piece | State |
|---|---|
| Environment | ✅ WSL + Docker, RTF ~0.5 on a fresh launch |
| Perception — column marker (1–5) | ✅ verified, 9 viewpoints, no misreads |
| Perception — book colour and row | ✅ verified, 16/16 books, 0 false positives |
| Perception — collection bin | ✅ new; found reliably, located to ~20 mm |
| **Scoring topics** | ✅ **published in a real run** — column 3, row 2, both correct |
| Annotated images | ✅ written, timestamped |
| Mission sequencing | ✅ new; one state machine owns the phase order and the trial clock |
| `solution.launch.py` | ✅ now starts move_group, the grasp and the delivery too |
| Approach — search, centre | ⚠️ **improved, still not reliable** |
| Approach — acquire, square | ⚠️ **improved, still not reliable** |
| Arm kinematics + IK | ✅ exact to 0.7 mm; all four rows reachable |
| Grasp controller | ⚠️ written; **never yet run at a healthy real-time factor** |
| Place in bin | ⚠️ **written, never run end to end** |
| Video (D2) | ❌ not started |
| Report (D3) | ❌ not started |

**115 unit tests**, no simulator required:

```bash
sim shell
cd /opt/erc_ws/src/avaa_solution && python3 -m pytest test/ -q
```

---

## The next step

**The approach is the critical path.** Nothing physical scores without it, and it is the
one piece that has never been reliable. It now gets through searching and centring and
part of acquiring, where a week ago it timed out in the first of them, but it does not
finish.

Run it and watch:

```bash
tools/sim restart --fast --headless
# then, inside the container:
ros2 launch avaa_solution solution.launch.py shelf_column_number:=3 book_colour:=red
```

### What is known about why it fails

Five faults were found and fixed on 2026-09-02, each by watching one run (see commit
`8b7ac70`). The pattern behind four of them is worth stating on its own:

> **The base has no friction across the roller axis, so it keeps whatever velocity it is
> given.** Every proportional controller written against it is an oscillator. Centring
> turned for ninety seconds without landing inside a twelve-pixel window; the acquire
> strafe walked its error from 132 px to 315 px in one direction. Turns are now damped
> against the yaw rate from odometry — odom is blind to sliding, which is why nothing
> else trusts it, but turning is the one thing that genuinely rotates these wheels.

What still goes wrong, as of the last run: centring can latch onto a bearing at the very
edge of the frame (+314 px, a red book at the image edge rather than the target column)
and turn at full rate without the error changing. The suspect is
`_track_book_without_marker`, which publishes a steering bearing for whichever
target-coloured book is nearest the image centre — reasonable when parked in front of the
right column, wrong when the robot has turned away from it. It should probably not
publish a bearing at all until the column has been reached.

### The other thing that would move the needle

**Hold the base whenever nothing is driving it.** Silence on `/cmd_vel` is not an
instruction to stay put, and no node was doing this. Measured against Gazebo per
simulated second:

| | translation | yaw |
|---|---|---|
| arm still, nothing commanded | 6.8 mm/s | 0.81 deg/s |
| arm still, zero twist at 20 Hz | 3.1 mm/s | 0.42 deg/s |
| arm swinging, nothing commanded | 9.8 mm/s | 0.88 deg/s |
| arm swinging, zero twist at 20 Hz | 2.1 mm/s | 0.43 deg/s |

The grasp and the delivery now both do it. An earlier version of this experiment
concluded a zero twist changed nothing; it was measured per wall second at RTF 0.013,
where almost no simulated time passes and every condition looks identical.

---

## What will bite you

- **`use_sim_time:=true` on every node.** Gazebo stamps TF with `/clock`, hours behind
  wall time. Without it tf2 floods with `TF_OLD_DATA` and lookups silently return nothing.
- **A node's clock reads zero until the first `/clock` arrives.** Anything that latches a
  start time in its constructor latches the simulator's uptime instead. The mission node
  did, and reported a delivery 54.5 s into a trial that had not started.
- **`/bin_contacts` fires from the first instant** — the bin stands on a table and is
  permanently in contact with it. Only a contact whose other party is a *book* counts.
- **`/avaa/perception/target_column` is frame-relative.** It counts the columns currently
  in view. It is for steering. The judges get `/avaa/perception/shelf_column`, which is
  only published when all five markers are in one frame.
- **Never teleport.** See the top of this file.
- **Recreating the container wipes `install/` and `build/`.** Expected; rebuild takes 15 s.
- **Re-publishing a `JointTrajectory` restarts it.** A trajectory re-sent on a timer never
  completes — which is a bug in a plan and the definition of a servo.
- **The torso undershoots by 2.5–3 cm**, repeatably. `TORSO_BIAS` compensates.
- **Don't read gripper state from `/joint_states`** — the linkage joints are not published
  there. Use TF.
- **The stowed arm sits in the LiDAR plane.** Returns within 0.45 m of `base_footprint`
  are the robot seeing itself.

## Known-unresolved

1. **The approach is not reliable.** See above.
2. **The grasp has never run at a healthy real-time factor.** Everything measured about it
   — that trajectories report complete while the arm is still travelling, that the
   controller cannot follow, that the base slides tens of millimetres during a reach — was
   measured after a teleport, at RTF 0.03. Some of it will survive re-measurement. None of
   it should be assumed.
3. **Depth has a systematic vertical bias.** Worked around by taking height from the
   identified row, and from the known bin rim height, and only x/y from depth.
4. **`tools/drive_to.py` does not converge.** It is the teleport-free replacement for
   `place_robot.py`; the base coasts after each burst, so its open-loop corrections
   oscillate about the heading. Same root cause as item 1.

## Decisions taken, and why

- **MoveIt is in use**, with an SRDF written here — the image ships none for this robot.
  `moveit.launch.py` is now included by `solution.launch.py`.
- **The last centimetres of a grasp are servoed, not planned.** A plan-execute-check cycle
  costs six to ten seconds and the target moves twenty to thirty millimetres inside one,
  so the loop cannot converge. The servo runs at 5 Hz on the ordinary tick.
- **No lateral base motion.** Commanding pure `vy` yaws the base by roughly the magnitude
  it strafes. Rotate-then-drive, or drive to a pose.
- **Map-less Nav2 in the `odom` frame.** No prior map, start pose not guaranteed.
- **Row numbering is a launch parameter** (`rows_top_down`), and so is column numbering
  (`columns_left_to_right`), because the rules state neither.
