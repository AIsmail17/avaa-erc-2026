# Team AVAA — project state

**Read this first when resuming.** Written 2026-09-02, brought up to date 2026-09-10.
Phase 1 deadline **2026-09-15**.

Nothing important lives in a chat transcript. Everything is in this folder, in the repo, or
in the git history — which carries the reasoning, not just the diffs.

---

## Repository

`https://github.com/AIsmail17/avaa-erc-2026` — branch **`avaa`**, which is also the
default branch, so a plain `git clone` lands on the right code. **Public.**

Local on Ahmed's machine: `~/erc/erc_sim_2026` inside WSL (physically on D: via the
VHDX). The VHDX itself is not in any repository and must not be — it is 15 GB, and
GitHub refuses any file over 100 MB.

### Starting from a fresh clone

```bash
git clone https://github.com/AIsmail17/avaa-erc-2026.git
cd avaa-erc-2026
```

Then `SETUP.md`, which is the long one: it is the environment build with every trap
hit along the way, and it is worth reading before running anything. `README.md` is the
organisers' own and covers the base simulator only — it says nothing about our solution.

Once the container is up:

```bash
tools/sim restart                    # from the host: relaunch Gazebo
tools/sim gui                        # attach a viewer window to a running simulation
```

```bash
# inside the container
ros2 launch avaa_solution solution.launch.py shelf_column_number:=3 book_colour:=red
```

`tools/` holds about sixty small measurement programs, each answering one question and
each carrying in its docstring what it measured and when. `tools/in-sim <tool>.py` runs
one inside the container. When a number in this file disagrees with a number in a tool,
the tool is the one that was run — check its date.

The commit messages are the design record. `git log` is worth reading before changing
anything, because most non-obvious decisions have a measurement behind them.

## Documents in this folder

| File | What it holds |
|---|---|
| `NOTES.md` | Competition rules, scoring, deliverables |
| `SETUP.md` | Environment build, with every trap hit along the way — **start here on a fresh clone** |
| `PERCEPTION.md` | Colour and marker detection, measured accuracy, 3D localisation |
| `MANIPULATION.md` | Gripper curve, reach envelope, arm kinematics, tuck pose |
| `ORGANISER_QUESTIONS.md` | Six items. The committee's answer was that teams solve these themselves; kept as the record, and item 1 is corrected in place |
| `WATCHING.md` | Running a simulation you can SEE, on the NUC through remote desktop |
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
| Environment | ✅ WSL + Docker, RTF ~0.5 headless; ~0.2 with the GUI and the view stack up |
| Perception — column marker (1–5) | ✅ verified, 9 viewpoints, no misreads |
| Perception — book colour and row | ✅ verified, 16/16 books, 0 false positives |
| Perception — collection bin | ✅ new; found reliably, located to ~20 mm |
| **Scoring topics** | ✅ **published in a real run** — column 3, row 2, both correct |
| Annotated images | ✅ written, timestamped |
| Mission sequencing | ✅ new; one state machine owns the phase order and the trial clock |
| `solution.launch.py` | ✅ now starts move_group, the grasp and the delivery too |
| Approach — end to end | ✅ 2026-09-08: **three consecutive clean runs, 53.4 / 56.3 / 61.9 s** to hand-over |
| Arm kinematics + IK | ✅ exact to 0.7 mm; holds all four rows to within 6 mm in open air |
| Grasp — reach and servo | ✅ reaches into the shelf, clamps, lifts, hands over to delivery |
| Grasp — actually holding it | ✅ **2026-09-10: the book comes off the shelf.** Two runs in a row, the servo arriving 1 and 2 mm off the book's centre line and the grasp aid taking it at 65 and 84 mm. Carried 1.27 m and stowed |
| Place in bin | ⚠️ **written, never run with a book in the gripper** |
| Video (D2) | ❌ not started |
| Report (D3) | ❌ not started |

**159 unit tests**, no simulator required:

```bash
sim shell
cd /opt/erc_ws/src/avaa_solution && python3 -m pytest test/ -q
```

---

---

## 2026-09-10 — the arm was aimed 110 mm low, at every row, all along

`base_link` is **0.0762 m** above the floor. This project used **0.186** in eighteen
places. `ROW_HEIGHTS_BASE` is world minus that constant, so every row was 110 mm low, and
the grasp aims a further 45 mm below the book centre — so the gripper was sent 155 mm
below the middle of a book 250 mm tall. That is 30 mm below its bottom edge, into the
board it stands on.

It is now one constant in `avaa_solution/arena.py`, and everything is derived from it.

| | old | corrected |
|---|---|---|
| row 1 | 1.391 | **1.5008** |
| row 2 | 1.061 | **1.1708** |
| row 3 | 0.731 | **0.8408** |
| row 4 | 0.401 | **0.5108** |
| bin rim | 0.764 | **0.8738** |

Three sources agree, none of them the URDF quoting itself: TF measures +0.0762 and puts
`base_link` at the wheel axle height; Gazebo and TF agree on `torso_lift_link` to a tenth
of a millimetre, which also settles that `base_footprint` is the floor (`tools/worldtf.py`);
and every book fix through the camera lands within 6 mm of ground-truth-minus-0.0762 and
115 mm above ground-truth-minus-0.186 (`tools/bookheight.py`).

**Why nothing caught it, and the general lesson.** The tools that measured the miss
converted ground truth into `base_link` with the same constant as the code that aimed the
arm. Both sides moved together, the difference came out as zero, and a reach 110 mm low
reported as perfect — "+24 mm in depth and −5 mm in height at the clamp" is in a comment
in `tools/arenafeed.py` as evidence the arm was fine. **A constant on both sides of a
comparison is never tested by that comparison.** `test_arena.py` now checks it against the
URDF, which cannot move with it.

Three things that were read as separate faults were this:

* **Row 4 "unreachable"** — 0.401 is 110 mm below where that row is, and down there the
  forearm meets the base.
* **`DEPTH_HEIGHT_BIAS = 0.152`** — almost all of it was this. The real residual is +6 mm
  with 9 mm of spread over four rows, three head tilts and 0.65–1.6 m. Rows are 330 mm
  apart, so the measured height can now name a row outright, and `_cross_check_row` is
  allowed to overrule the marker row by one, which it never was before.
* **The shelf boards in the planning scene** were 110 mm low, so the planner avoided
  boards that were not there and drove through ones that were.

### What it bought

First grasp after the fix, row 3, book fed from ground truth:

```
the grasping frame is +0 mm depth, +2 mm sideways, +1 mm height of its target,
but the PADS are on the book: 80 mm into it, 8 mm off centre sideways, 42 mm in height
the jaws stopped at 27.1 mm on a 30.0 mm book, so there is something between them
```

The jaws closed **on the book** for the first time. The book has still never been carried
to the bin — see below.

## 2026-09-10 — the yaw can be measured after all: the IMU

STATE.md said above that uncommanded rotation is not reliably measured, and that the depth
camera is the only instrument left. It is not: `/base_imu` publishes at 100 Hz and it is
very good. Integrated over four ten-second windows, against Gazebo (`tools/imudrift.py`):

| | 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| IMU | −13.616 | −16.472 | −5.914 | −6.482 |
| truth | −13.291 | −16.892 | −5.642 | −6.943 |
| odom | −4.210 | −2.856 | −2.680 | −0.893 |

Under half a degree, against odom seeing about a quarter of the turn.

**Filter hard, and this is the part that matters.** The raw signal is a base that shakes:
mean −0.74 deg/s, spread 4.70, range −18.27 to +17.16. The spread is six times the mean,
and it is not sensor noise — the URDF gives the gyro a stddev of 2e-4 rad/s and the
measured spread is 0.082, four hundred times it. Braking on the instantaneous rate
saturates the command clip and halves the drift at best. Filtered at alpha 0.01 (about a
second), gain 2 (`tools/yawbrake.py`):

| gain | turned over 10 s | note |
|---|---|---|
| 0 | −8.391 deg | the control |
| **2** | **−0.517 deg** | 94% removed, largest command 0.040 rad/s against a 0.15 clip |
| 0 | −10.231 deg | the control again |
| 4 | +4.997 deg | overshoot, it turned the base the other way |

This is now `hold_imu_yaw_gain` in `grasp_node`. Measured in an arena grasp: the base
finished **1.1 degrees** off square, against 19.4 before.

Note why it works where `hold_base.py`'s damping term failed. That one differenced a pose
over a 1.5 s loop and oscillated out to 503 mm — derivative feedback through a long delay
doing exactly what that predicts. The IMU has no delay.

## 2026-09-10 — the book comes off the shelf

Two runs in a row, end to end, `idle` through `done`:

```
square to the shelf at -1.2 deg after 2.2 s
at the staging point; ... closing to a point 20 mm in front of the book's face
    and leaving the rest to the servo
squaring up 65 mm sideways and 2 mm in height before going in
servo is on the book (-0 mm depth, +2 mm sideways, +0 mm height) after 3.8 s; clamping
sim_grasp_fix: picked up book_col_3_row_4_blue, 84 mm from the grasping link
...
the book moved 1271 mm during the run
```

What it took, beyond the 110 mm row-height correction above, is in the commit
`f628166` and summarised here because three of the four are traps rather than tuning:

* **A killed node leaves a standing `cmd_vel`.** Nothing damps this base, so every run
  inherited whatever the last one was doing when it was killed — one grasp opened with
  the base 107.4 degrees off square, forty seconds after being teleported to yaw zero.
  `tools/stopbase.py`, and `place_robot.py` now does it as part of placing.
* **Square the base BEFORE latching the target.** Everything the grasp holds is measured
  in base_link at the moment the target is planned, so squaring up afterwards swings all
  of it — 90 mm for a book 0.72 m out through 7 degrees — and the hold's angular and
  linear channels then fight each other.
* **The reach used to stop *inside* the book.** It planned all the way to the face plus
  110 mm and only then handed to the servo, so the first lateral correction happened with
  the jaws already straddling the book. A parallel gripper closes about its own centre
  line: a jaw that arrives off centre does not miss, it pushes. Measured across four runs
  the book was shoved 148, 163, 202 and 207 mm sideways and toppled. It now stops 20 mm
  in front of the face and the servo squares up there before advancing in depth at all.
* **The base hold was starved by its own executor.** `_tick` blocks for tens of seconds
  on planning, so on one thread the 20 Hz hold timer never fired — one `hold:` line in a
  whole posture search, and the base 500 mm out by the end of it. The hold and its inputs
  now have their own callback groups under a `MultiThreadedExecutor`. That works for the
  waits, which are sleeps; it cannot work for the posture search, which is Python holding
  the GIL, so that gets a wall-clock budget instead.

### Still to do on it

* The place into the bin has never run with a book in the gripper. It is written, and the
  bin rim was 110 mm low until today.
* The whole grasp takes about 85 simulated seconds, most of it MoveIt planning rather
  than moving. Against a 3–8 mm/s drift that is the margin being spent.
* Perception has not been re-run since the height cross-check was allowed to overrule the
  marker row. Every grasp above was fed from ground truth by `tools/arenafeed.py`.

---

## 2026-09-10 — what was left, and it was one thing

**The base's linear coast.** ~3 to 8 mm per simulated second, on one heading, forever, and
the arm makes no difference to it (`tools/holdwhilereaching.py`: 8.0 mm/s arm still,
7.7 mm/s arm swinging — so it is not the arm's reaction, which had been the assumption).
A grasp takes about 85 simulated seconds, so that is 250 to 680 mm.

That is what now breaks a grasp, and the mechanism is exact. The base retreats until the
book is **past the end of the arm**, and grasp_node then does the right thing and refuses
the sighting:

```
ignoring a sighting 1240 mm from the shoulder; the arm reaches 1088 mm
```

having last aimed at where the book was 200 mm ago. It clamps there, on air. The clamp
report on that run: −190 mm depth, +158 mm sideways.

**The drive is not the problem, and a comment in `hold_base.py` saying otherwise is
wrong.** It reads "commanded 0.02 m/s the base simply sits there", and the hold's gain,
clip and deadband were all chosen around that belief. Measured (`tools/drivecurve.py`),
each command paired with a silent control window so the base's own coast is subtracted
rather than attributed to the command:

| commanded m/s | 0.010 | 0.020 | 0.030 | 0.040 | 0.060 | 0.100 |
|---|---|---|---|---|---|---|
| arrived mm/s | 6.9 | 14.9 | 25.2 | 32.7 | 53.7 | 88.3 |
| fraction | 0.69 | 0.75 | 0.84 | 0.82 | 0.89 | 0.88 |

No floor anywhere: 10 mm/s commanded moves the base at 6.9. So the hold clipping at
40 mm/s has 33 mm/s of real authority against a 3–8 mm/s drift, four times what it needs.

Which leaves one explanation standing — **the hold was not commanding** — and that is a
question about the run rather than about the arithmetic, so `tools/holdtrace.py` records
what the hold asked for, what book point it was correcting from, and where the base truly
was, one row a second, alongside a live grasp.

The other direction is worth taking regardless: **take less time.** 85 simulated seconds
against a 3–8 mm/s drift and roughly 150 mm of reach margin means the grasp has to fit in
about 30 seconds for drift alone not to break it. Most of that time is MoveIt planning,
not moving.

### Two faults in the simulation grasp aid, both found by it refusing a good grasp

* **One failed pick disabled it for the life of the process.** The "too far to hold from"
  branch returned without clearing `attach_asked`, and the only thing that cleared it
  needed something already held. It is launched once with the simulator and lives for
  hours.
* It composed the gripper's world height from Gazebo's base_footprint pose and TF's
  base_link gripper without the 76 mm between them, and decided on a **cached base pose up
  to three seconds old** against a live arm. Three seconds of this base is tens of
  millimetres against a 90 mm tolerance.

Both fixed. `tools/putback.py` stands books back on the shelf between attempts, with
`--jitter` to restore the randomiser's sideways spread when perception is what is being
tested.

---

## The first complete grasp, 2026-09-07

Kept at `~/erc/runs/FIRST-GRASP-20260907-1143.log`.

```
target red book is on row 2 (15 of 15 readings agree)
shelf removed from the planning scene; the reach in is checked waypoint by waypoint
reaching along 9 waypoints to [0.783, 0.07, 1.016], stopping 35 mm short
at the staging point; the book moved 36 mm while I reached, closing the last 83 mm
servo is on the book (-1 mm depth, -0 mm sideways, -24 mm height) after 4.8 s; clamping
clamping -> lifting -> withdrawing -> stowing
```

The reach was not obstructed. "the reach is obstructed 12 per cent of the way in", which
had blocked every grasp for days, did not appear -- the cause had already been found and
fixed (the line was being drawn from `pre_target` rather than from the point the chosen
posture actually reaches) and this is the first run that got far enough to prove it.

The run was stopped at `stowing` to deploy the next fix, so the handover to delivery is
still unobserved. **Do not stop a run that is going well.** Two runs were cut short that
afternoon at the exact moment they were succeeding, and each one cost thirteen minutes
to reproduce.

### What was blocking it

`MAX_AREA` in the book detector was 3000 square pixels, and every measurement it was
calibrated from was taken from across the room -- "6-8 px wide, 14-26 px tall" is a book
three to five metres away. At grasping range the target measures 4616 with an aspect of
4.20:

```
red    (252, 264)   35 x 147   area 4616   aspect 4.20   AREA ABOVE 3000
green  (472,  72)   50 x 145   area 6436   aspect 2.90   AREA ABOVE 3000
```

So the detector went blind at exactly the range the arm works at. Perception reported "no
red book in view at close range" with the book in the middle of the frame, and the
approach held at 0.95 m refusing to drive in on the LiDAR alone -- correctly, by its own
rule. Aspect, not area, is what separates a book from the bin: 1.75-4.33 against the
bin's 0.49.

---

## 2026-09-08 — the approach is done; the grasp closes on nothing

**The approach now works.** Three consecutive runs handed over to the grasp in 53.4,
56.3 and 61.9 simulated seconds, on rows 4, 3 and 4. A week ago it timed out searching.
What fixed it, in order: aiming the head where the markers actually are, holding the
marker reader to its previous answer, refusing book fixes that are on the wrong shelf,
capping forward speed by the sideways error still to be corrected, and letting a back-off
finish instead of resetting every 35 degrees of turn. Each has a commit with its
measurement.

**The grasp reaches, clamps, lifts and withdraws — around nothing.** After a clamp that
reported success and a delivery that carried an empty gripper to the bin:

    both finger joints        0.0010  ->  span 28.7 mm
    the book                          30.0 mm thick
    every book in Gazebo      still on its shelf at x = 2.900

A 30 mm book cannot be inside 28.7 mm of jaw. The clamp now checks that span before
lifting and fails the grasp if the jaws met each other, so this cannot be reported as a
success again.

### What is known about the miss

The servo judges arrival by the grasping frame, and the pads are 30 mm behind it —
confirmed in flight, matching the 29.7 mm this project has always assumed. The frame
consistently stops about 45 mm short in depth and 45 mm low in height, with **zero IK
solves rejected**, and the arm was measured the same day holding every row to within 6 mm
in open air. So it is not the arm.

The pads-on-the-book check reported the pads 65 mm into a 160 mm book when they were not
in it at all. That number is measured from **perception's estimate of the book face**,
which `PERCEPTION.md` records as reading the front face about 35 mm nearer than the
centre — so a check built on it can be self-consistent and still wrong.

**The next measurement is the pad midpoint against Gazebo ground truth at the moment of
clamping**, not against anything perception produced. That single number says whether the
face estimate is wrong or the arm is stopping short, and those have different fixes.

---

## 2026-09-07 — most of a week's observations were made through a broken sensor

Read this before trusting any conclusion in this file dated earlier than 2026-09-07.

Perception reported "no camera images arrived at all in the last 10 s" partway through a
run and then narrated a stored frame for the remaining ten minutes. That reads like the
Gazebo camera dying, and it was diagnosed that way at first. It was not that. A
subscriber created at that exact moment, on the same topic, with perception's own QoS,
received 30 frames a simulated second.

From `/proc/net/snmp`, mid-run:

| | |
|---|---|
| UDP `RcvbufErrors` | 3,119,527, rising by ~200 a second |
| `net.core.rmem_max` | 212,992 (the Linux default) |
| one colour frame | 640x360 BGR8 = 691 KB |

A frame is three times the whole socket receive buffer, so each one fragments across
hundreds of datagrams into a buffer that cannot hold one, and losing any fragment
discards the image. Two of the five kept runs from that morning show the stall outright,
and every run before the fix was losing frames continuously whether or not it stalled.

**So: the book fixes that jumped 3 metres, the bearings that held still while the base
turned, the depth readings of 4.88 m with the shelf 1.07 m ahead — much of that was a
starved image stream, not the detectors.** Anything tuned against those numbers is worth
re-checking. `sim status` now reports the drop rate, and after the fix perception
receives 306 frames per ten-second window against 96–126 before.

Fixed in `src/erc_bringup/config/cyclonedds.xml` (32 MB receive buffer, defrag limits)
plus `tools/sim` raising `net.core.rmem_max` when it starts the container. The container
is privileged and on the host network, so it does this itself and needs no password.
Perception also rebuilds its subscriptions after a window with no frames.

### The other four faults found that day

Each has a commit with the measurement in its message; `git log` is the record.

**The search looked where the marker cannot be.** The camera sits 1.160 m up with a
28.1 degree vertical half-angle and the markers are at 2.26 m, so with the head level
they leave the frame inside 2.06 m — and the approach leaves the head tilted 41 degrees
*down* to watch the bottom shelf. Every route into SEARCH is a close-range state. One
run turned on the spot for 150 seconds reporting "2 red book(s) in view" and read a
marker on 0 of every 50 frames. SEARCH now aims the head at the marker band and backs
off when it is standing too close, using the rear laser — bridged since the beginning
and, until then, subscribed to by nothing.

**It timed out centring while sitting correctly centred.** The marker reader decides
which of five near-identical plates carries the target digit afresh on every frame and
occasionally picks wrong. The approach was fed 340 px and 14 px alternately and turned
back and forth: "asked for 33 deg of turn over 84 ticks, base has turned 4 deg". The
reader is now held to its own previous answer, with the threshold taken from the
measured column spacing in the frame.

**One depth frame could overturn a fifteen-frame vote.** `_cross_check_row` switched the
target row on a single measurement that `_watch_jump` had flagged in the same instant as
15.2 m/s. An override now needs the same kind of majority the vote did, and only runs
while the markers are in view — it exists to catch a row miscounted from the markers,
which is a question about the column and needs the column visible to answer.

**The tracker stepped one column left and one shelf up.** Inside 1.6 m the markers leave
the frame and the book tracker is choosing among identically coloured books with no
anchor but its own last answer. The approach anchored at `[1.52, 0.99, 1.51]` in
base_link: 0.99 m across is one column spacing, and 1.51 m up is row 1 where row 4 had
been identified 15 readings to 15. The target row's height is known from three metres
out and cannot drift afterwards, so a fix more than 0.20 m from it is on another shelf
and is no longer offered. Rows are 0.330 m apart, so that tolerance cannot confuse two
of them.

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

### The base coasts, and this is the thing to fix

**Corrected 2026-09-03. The table that used to be here was wrong, and the conclusion
drawn from it — that a zero twist holds the base — does not survive re-measurement.**

The base keeps whatever velocity it is given, indefinitely, and nothing damps it. The
wheel model asks for exactly that: `mu2` is 0 across the roller axis, so there is no
friction to shed a slide, and commanding zero wheel speed asks the wheels not to turn
rather than asking the base to stop.

`tools/coast.py`, eight consecutive windows with a zero twist published at 20 Hz
throughout, per simulated second against Gazebo:

| window | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| speed (mm/s) | 8.1 | 7.4 | 6.9 | 8.5 | 8.7 | 6.8 | 7.7 | 7.8 |
| heading (deg) | −155 | −159 | −162 | −172 | −156 | −179 | −173 | +177 |

Heading agreement **0.98 of 1.0**. That is one velocity held, not a wander.

And a zero twist does nothing to it. Four conditions that should have differed:

| | translation | yaw |
|---|---|---|
| arm still, nothing commanded | 6.5 mm/s | 0.57 deg/s |
| arm still, zero twist at 20 Hz | 7.1 mm/s | 0.55 deg/s |
| arm swinging, nothing commanded | 7.2 mm/s | 0.73 deg/s |
| arm swinging, zero twist at 20 Hz | 7.1 mm/s | 0.39 deg/s |

**Why the old table read differently.** It was taken with the robot standing where it
had stood for a long time, which is a robot that has already shed its velocity. That is
not the state the grasp inherits — the approach hands over having just been driving.
Over a 100 s grasp this is 700 mm of travel and 55 degrees of turn, which is enough on
its own to explain every grasp failure recorded here.

**The base does strafe.** Worth stating because the lateral command saturating for a
whole approach looks exactly like a strafe that does nothing, and `mu2 = 0` across the
roller axis makes that plausible. `tools/strafecheck.py`, per simulated second against
Gazebo: commanded ±0.100 m/s sideways it went 0.080 and −0.081, against forward and
backward trials at 0.077 and −0.084. Between 78 and 85 per cent in all four directions,
no meaningful cross-coupling.

**What can and cannot see it:**

| sensor | verdict |
|---|---|
| odom, translation | blind — a wheel that is not turning reports nothing |
| odom, rotation | **accurate for commanded turns**, and an earlier claim here that it was blind was wrong. `tools/turncheck.py`: commanded 0.15, 0.30, 0.45 rad/s → odom 0.142, 0.270, 0.407 against a true 0.142, 0.235, 0.356. The claim came from `tools/spinhold.py` subscribed to `/mobile_base_controller/odom`, **which does not exist on this robot** — the only odometry topic is `/odom` — so it drove against a variable still holding its initial zero. Feeding the rate back still does not cancel the residual rotation (0.657–0.814 deg/s at gains 0.5–2.0 against 0.646 for a zero twist), but that is a different statement |
| the book alone | **not enough.** `tools/bookcoast.py` fitted 61.9 and −76.9 mm/s against a true 6.0 and −6.0, and driving on it made the drift worse, 8.5 → 22.3 mm/s. `dp/dt = −v − w × p`, so at 0.8 m the rotation swamps the translation. The rotation-corrected version of that tool was never actually tested — it read the non-existent odom topic above |
| the depth camera | the only instrument left, and the one the shelf-plane fit already uses |

**Uncommanded ROTATION is not reliably measured yet.** `tools/spinhold.py` cannot
measure it: each of its windows inherits the velocity the previous one left, and a base
that coasts has no way to settle in between, so it reports anything from 0.06 to
0.81 deg/s across conditions that ought to be comparable. The translational figures
above do not have this problem — they used one condition throughout and agree to 0.98.

**Cancelling the translation works, once it is measured.** `tools/stopcoast.py`, taking the slide
from Gazebo: 7.07 mm/s coasting, 3.39 at a gain of 1, 2.02 at a gain of 2, overshooting
back to 4.82 at 4. The wheels can do it. The problem is entirely one of measurement.

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
