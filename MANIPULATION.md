# Manipulation — measured characteristics

Everything here is measured against the running simulation. Feeds report section 4
(Manipulation).

---

## 1. ❌ The startup error is the whole problem — this section had it backwards

**Corrected 2026-09-08.** This section used to be headed "The gripper works, despite the
startup error ✅" and concluded the error was harmless. That was wrong, it stood for
several days, and it is why the grasp was hunted everywhere except where the fault was.

The measurements below are real. The conclusion drawn from them was not: they are
fingertip separations in FREE AIR, and a linkage that moves correctly with nothing
between the pads tells you nothing about whether it can hold something. Measured on the
bench (`tools/lab`, `tools/jawtest.py`) with a book actually between the jaws, the
closing jaws pass straight through it — see §1b.

`erc_bringup` logs this at every startup, in red:

```
[Err] [Physics.cc:1906] Attempting to create a mimic constraint for joint
[gripper_left_inner_finger_left_joint] but the chosen physics engine does not
support mimic constraints, so no constraint will be created.
```

The gripper is a linkage: one actuated joint (`gripper_left_finger_joint`) with the rest
of the finger joints declared as mimics. DART does not implement mimic constraints, so the
warning is real — and it makes grasping impossible.

The linkage is therefore not simulated at all: no constraint exists. What keeps the
fingers together is `gz_ros2_control`, which notices the mimic parameters in the
`<ros2_control>` block — the mimic joints declare no command or state interface, only
these:

```xml
<joint name="gripper_left_inner_finger_left_joint">
  <param name="mimic">gripper_left_finger_joint</param>
  <param name="multiplier">-8.28</param>
</joint>
```

and, every update, servos each one towards where the leader says it should be
(`gz_system.cpp`, "set values of all mimic joints with respect to mimicked joint"):

```cpp
double position_error = position_mimic_joint - position_mimicked_joint * multiplier;
double velocity_sp    = -1.0 * position_error * update_rate;
// ... written to sim::components::JointVelocityCmd
```

So the fingers are **not** teleported, and an earlier note here that said they were is
wrong. They are velocity-servoed, through exactly the same `JointVelocityCmd` path a real
position-commanded joint takes.

That is the actual fault, and it is a narrower one. `JointVelocityCmd` asks the engine to
*make the joint move at this velocity*, and the engine will spend whatever force that
takes. A book between the pads does not stop the joint; it just raises the price, and
nobody is checking the price. The gain makes it worse — `update_rate` is the full
controller rate, so a millimetre of tracking error asks for a very large velocity.

The consequence for grasping is the same either way: nothing in the finger chain
generates a bounded contact force, which is also why the fingers visibly pass through each
other.

Measured fingertip separation (TF distance between
`gripper_left_fingertip_left_link` and `gripper_left_fingertip_right_link`):

| `gripper_left_finger_joint` | Fingertip span |
|---|---|
| 0.000 | **0.0280 m** |
| 0.020 | 0.0444 m |
| 0.040 | 0.0605 m |

Close to linear: `span ≈ 0.028 + 0.82 × joint`.

**A book is 30 mm thick and the closed span is 28 mm.** This was read as "the gripper
closes past the book and can clamp it". It is the opposite: closing past the book is the
fault. A jaw that can reach 28 mm around a 30 mm object has gone through 2 mm of it.

Open at 0.040 gives 60.5 mm — twice the book thickness, and that part is still useful:
there is ample clearance for approaching.

### 1b. The bench measurement, on the two engines

`tools/lab up <engine>` then `tools/in-sim jawtest.py`. The test needs no planning, no
approach and no camera: open the jaws, teleport the book to the midpoint between the
fingertips, close, stop holding it, and see whether it falls.

| | dartsim (default) | bullet-featherstone |
|---|---|---|
| mimic constraints refused at startup | 1 (per gripper) | **0** |
| jaws open | 73.3 mm | 75.9 mm |
| finger commanded to 0.000, reached | **−0.0004** | **0.0535** |
| jaws around a 30.0 mm book closed to | **27.4 mm** | 70.8 mm |
| book, once released | fell to the floor | not carried |
| real-time factor, same world | 0.25 | **0.67–1.00** |

Under dartsim the finger reaches its commanded position as if the book were not there.
Under bullet-featherstone it **stalls at 0.0535** — it hits the book and cannot continue,
which is a contact force, which is the thing that has never existed in this simulation.

Note what that implies about the remedy. The problem is not that the fingers are
positioned by fiat; it is that they are velocity-servoed with no force ceiling. Anything
that puts a ceiling back — a real mimic constraint under bullet, or an effort command
interface on the gripper joints — should change this number.

That is the mechanism confirmed from both directions. Neither engine has yet carried a
book: under bullet the finger stalls too early to close on the book at all.

### 1d. There is a force ceiling, and it is the thing to tune

`tools/graspforce.py` scales the left gripper's effort limits in the URDF; the bench then
picks them up with no rebuild. Scaled by 0.001 — 0.01 N on the driven prismatic instead of
10 N — the finger commanded to 0.000 **stalled at 0.0634**, and the pads moved 2 mm in the
whole close.

So dartsim *does* honour the joint effort limit when it applies `JointVelocityCmd`. The
ceiling exists. At the stock 10 N it is simply far above what a 30 mm book can resist, so
the jaws win. That reframes the problem again, and in a much more tractable direction:
not "position control cannot grasp", but "this gripper is allowed a hundred times the
force it needs".

Still open, and the next thing to measure: whether the pads ever generate a *contact* at
all. The closed span has come out at 27.4 mm and at 31.0 mm on different runs against the
same 30.0 mm book, which is the kind of spread that suggests the fingers are sometimes
missing it entirely rather than squeezing it. `tools/padcontact.sh` reads the fingertips'
own contact sensors — they publish per link, not to a single `/contacts` topic, which is
why an earlier attempt to watch `/contacts` saw nothing and proved nothing.

### 1c. bullet-featherstone is not a free swap

Worth knowing before reaching for it:

  - `gz_ros2_control` [issue #440](https://github.com/ros-controls/gz_ros2_control/issues/440)
    reports controllers failing to activate under bullet-featherstone. **Not reproduced
    here** — all seven activate in 12 s. The one time it looked like it did, the cause was
    a stale `controller_manager` from the previous bench that had not finished dying;
    `tools/lab` now waits for the processes to go rather than sleeping and hoping.
  - **It does not die on its own.** This section first recorded that `gz sim` kept
    dying a few minutes in under bullet, with nothing in the log, and treated it as a
    stability problem with the engine. It is not. The *dartsim* bench died the same way,
    and the cause is the workstation: `erc_sim` had `RestartCount=0`, `ExitCode=0`, no
    OOM, and a `StartedAt` a few minutes old — the container had been restarted from
    scratch because **WSL shuts itself down when no command is running**, taking Docker
    and the bench with it. The same idle shutdown wiped `/tmp` between commands and
    killed a `nohup setsid` probe.

    The practical consequence, on the workstation only: bring-up and test must go in
    **one** invocation. Every bench measurement above came from a command that did
    `lab up` and `jawtest` together; every "it died" came from splitting them.
  - `/world/erc_world/dynamic_pose/info` only carries models that are MOVING. Under
    dartsim everything jitters enough to keep publishing; a settled bullet world can go
    silent on that topic, and several tools here read it — including `sim_grasp_fix`.

Working values for the grasp:

| Purpose | Command | Span |
|---|---|---|
| Open, clear of the book | `0.040` | 60.5 mm |
| Contact | `~0.002` | ~30 mm |
| Clamp | `0.000` | 28 mm |

> Do not infer gripper state from `/joint_states`. **The linkage joints are not published
> there** — only `gripper_left_finger_joint` and `gripper_right_finger_joint` appear. An
> earlier test concluded "only the actuated joint moved, the linkage does not follow",
> which was wrong: the other joints simply were not being reported. TF carries the real
> link poses and is the honest measure.

## 2. Arm joint limits

| Joint | Lower | Upper |
|---|---|---|
| `arm_left_1_joint` | −0.524 | 4.712 |
| `arm_left_2_joint` | −2.443 | 1.134 |
| `arm_left_3_joint` | −2.618 | 2.618 |
| `arm_left_4_joint` | −2.443 | 1.134 |
| `arm_left_5_joint` | −3.665 | 1.571 |
| `arm_left_6_joint` | −1.885 | 3.002 |
| `arm_left_7_joint` | −2.443 | 2.443 |

All joints spawn at **zero**, which is fully extended — see below.

## 3. ⚠️ The spawn pose collides with the shelf

At all-zero the gripper reaches **0.838 m forward, 0.478 m beyond the front of the base**.
Driving at the shelf in that posture wedges the arm into it. Measured at the point the
robot stalled:

- **six simultaneous contacts** — both grippers, both `arm_6` links, against `erc_shelf`
- forward motion cut from ~0.27 m to **0.041 m** per command
- the LiDAR meanwhile reporting 0.94 m of clear space, because it looks over the obstruction

Each contact event costs **−0.5**. The arms must be stowed before any driving.

### Driving posture (measured)

```python
TUCK_POSE = [-0.5, -2.4, 0.0, -2.4, 0.0, 0.0, 0.0]
```

| Metric | Value | Base limit |
|---|---|---|
| Furthest forward | 0.319 m | 0.36 m (half-length) |
| Furthest lateral | 0.174 m | 0.249 m (half-width) |
| Contacts | none | — |

Joint 2 does most of the work; the elbow pulls the forearm in laterally; joint 1 finishes
it. Found by measuring candidates with `tools/try_tuck.py`, not by guessing — the first
three guesses all left the arm outside the footprint.

> **The stowed arm sits inside the LiDAR plane.** The laser plane is at z = 0.209 m and the
> tucked arm reaches 0.319 m forward, so the robot sees its own arm as an obstacle ~0.35 m
> ahead. The approach controller discards returns within 0.45 m of `base_footprint` for
> this reason. Any manipulation code that reads the laser needs the same filter.

## 3b. ⚠️ Correction — the base is fine; the arms were the problem

An earlier measurement showed that commanding pure lateral velocity yawed the base by
roughly the magnitude it strafed, and a great deal was built on that: lateral motion was
disabled in the Nav2 configuration, and the approach controller was designed around
rotate-then-drive.

**That reading was taken with the arms extended.** Repeated with the arms stowed:

| Command | Result |
|---|---|
| `vx = +0.20` | dx +0.244, dy 0.000, dyaw **0.000** |
| `vy = +0.20` | dy +0.233, dx 0.000, dyaw **0.000** |
| `wz = +0.40` | dyaw +0.465, dx 0.000, dy 0.000 |

**The base is properly omnidirectional and cleanly decoupled.** The coupling came from the
arms hanging out 0.48 m beyond the footprint — asymmetric mass and, once near the shelf,
intermittent contact.

What this changes:

- **Strafing is available.** The approach could correct laterally without rotating, which
  is both faster and avoids losing the marker from frame during a turn. Completion time is
  the tie-breaker, so this is worth revisiting.
- **The Nav2 `vy` limits could be reopened**, though Nav2 is only doing the gross move.
- The rotate-then-drive design is not *wrong*, just more conservative than necessary.

Left as-is for now because the approach finally works end to end and stability matters more
than the seconds this would save. Worth doing once a full trial runs reliably.

> The general lesson, which cost time twice: **measure the robot in the configuration it
> will actually be in.** The same mistake produced the "torso only lifts 0.163 m" reading
> (the arm was still settling) and the FK model looking 3.5 cm wrong (commanded joint
> values rather than actual ones).

## 4. Interfaces

| Purpose | Topic | Type |
|---|---|---|
| Left arm | `/arm_left_controller/joint_trajectory` | `JointTrajectory` |
| Right arm | `/arm_right_controller/joint_trajectory` | `JointTrajectory` |
| Left gripper | `/gripper_left_controller_raw/joint_trajectory` | `JointTrajectory` |
| Right gripper | `/gripper_right_controller_raw/joint_trajectory` | `JointTrajectory` |
| Torso | `/torso_controller/joint_trajectory` | `JointTrajectory` |
| Head | `/head_controller/joint_trajectory` | `JointTrajectory` |
| Contacts | `/contacts` | `ros_gz_interfaces/Contacts` |
| Bin contact | `/bin_contacts` | — |

`/contacts` reports collisions by link name on both sides, which makes it a direct check
for whether a motion caused a penalty — worth asserting on during development.

> **Re-publishing a `JointTrajectory` restarts it.** Each message replaces the one in
> progress and resets its `time_from_start`, so a trajectory re-sent on a timer never
> completes. Publish once (or a few times within a fraction of a second to cover the
> controller not yet being subscribed), then wait.

## 5. Shelf geometry for grasp planning

| Item | Value |
|---|---|
| Shelf unit centre | X = 3.0, Y = 0.0, Z = 1.1 |
| Front face | X ≈ 2.85 (unit is 0.30 m deep) |
| Books sit at | X = 2.90 |
| Column centres | Y = 2.1, 1.05, 0, −1.05, −2.1 (columns 1–5) |
| Stocked row heights | Z = 1.577, 1.247, 0.917, 0.587 (rows 1–4, top-down) |
| Row spacing | 0.33 m |
| Book | 25 × 16 × 3 cm, 300 g |
| Torso lift | 1 DoF, 35 cm travel — needed to reach the top and bottom rows |

Row 1 at 1.577 m and row 4 at 0.587 m are 0.99 m apart, close to the torso's 0.35 m travel
plus the arm's 0.92 m vertical reach. The torso will have to move for the extreme rows.

---

## 6. ⚠️ Reach envelope — the spawn arm pose cannot reach any shelf row

Measured with the arm at all-zeros (the pose that reaches furthest forward) while driving
the torso through its full travel. Gripper pose is `gripper_left_grasping_link` in
`base_footprint`, which sits on the floor, so gripper z compares directly with world row
heights.

| Torso | Gripper x | Gripper y | Gripper z |
|---|---|---|---|
| 0.000 | 0.983 | 0.493 | 0.413 |
| 0.175 | 0.983 | 0.493 | 0.457 |
| 0.350 | 0.983 | 0.493 | 0.576 |

Against the shelf rows:

| Row | Height | Verdict |
|---|---|---|
| 1 | 1.577 m | 1.001 m above the range |
| 2 | 1.247 m | 0.671 m above |
| 3 | 0.917 m | 0.341 m above |
| 4 | 0.587 m | 0.011 m above |

**Not one row is reachable from the spawn arm pose, even with the torso fully raised.**
The arm points forward and slightly down; reaching a shelf needs the shoulder raised, so
every grasp pose has to lift the arm as well as extend it.

Two further constraints fall out of the same measurement:

- **The gripper sits 0.493 m to the LEFT of base centre.** Centring the *base* on a column
  therefore leaves the gripper most of a column-width off target. The approach controller
  currently centres the base, which is correct for driving but wrong for grasping — the
  centring target needs a lateral offset so the *gripper* lines up with the book, or the
  arm has to bring it inboard.
- **Torso travel does not translate one-for-one into gripper height.** The torso reached a
  confirmed 0.350 m (checked in `/joint_states`) but the gripper rose only 0.163 m, and
  non-uniformly (+0.044 m for the first half, +0.119 m for the second). The arm appears to
  sag under gravity as it lifts. Any pose table must be measured at the torso height it
  will actually be used at, not computed by adding the torso offset.

### What this means for the MoveIt question

Hand-picking joint configurations looked attractive while it seemed like four poses were
needed, one per row. It is less attractive now: the arm must be raised as well as extended,
the gripper is laterally offset, and the torso contribution is not linear. That is three
coupled unknowns per row.

The middle path is probably to use MoveIt's IK **offline** to solve for the four row poses
once, verify them in the simulator, and then execute them at runtime as plain joint
trajectories — avoiding MoveIt's planning cost during the trial while not hand-searching a
7-DoF space.

## 7. MoveIt is installed but not configured — we solve IK ourselves

The Phase 1 document states that MoveIt 2 is "pre-installed and configured for TIAGo Pro".
The first half is true: 26 MoveIt packages are present. **The second half is not.** There is
no SRDF anywhere in the image for this robot — `find` over `/opt/erc_ws` and
`/opt/ros/humble/share` turns up only MoveIt's own test fixtures (`gonzo.srdf`,
`kermit.srdf`). Without an SRDF there is no planning group to ask for IK, and authoring a
robot configuration is not a good use of the remaining time.

Available instead: **PyKDL** and **scipy 1.8.0**. `kdl_parser_py` is *not* installed.

`avaa_solution/kinematics/arm_chain.py` builds the chain straight from the URDF. The path
`base_link -> gripper_left_grasping_link` runs through 12 joints, 8 of them moving: the
prismatic torso lift plus seven revolute arm joints, every one rotating about its own
local Z. IK is scipy `least_squares` over those 8 with joint limits as bounds, position
only — the arm has seven joints for three constraints, so pinning orientation as well tends
to make the solve fail rather than return something usable.

### The model is exact ✅

Validated against the running simulator across five postures (`tools/validate_fk.py`):

| Compared against | Max error | Mean |
|---|---|---|
| **Commanded** joint values | 0.0355 m | 0.0286 m |
| **Actual** joint values | **0.0007 m** | 0.0004 m |

0.7 mm against the joint values the robot actually holds. The kinematics are right; the
residual against commanded values is the controller not having arrived.

Judging a model against commanded values would have condemned it wrongly. The x and y
components matched to three decimals in every posture from the start — only z was off,
and only because of the torso.

### ⚠️ The torso undershoots by ~2.5–3 cm, systematically

| Commanded | Actual |
|---|---|
| 0.000 | 0.022 |
| 0.150 | 0.122 |
| 0.200 | 0.176 |
| 0.350 | 0.320 |

Repeatable, not lag — it settles short every time, in both directions of travel. Grasp
poses must either compensate or close the loop on the measured torso position. At 0.33 m
row spacing, 3 cm of error is a tenth of the gap between shelves.

### Every row is reachable ✅

IK to a book 0.80 m in front of the base and laterally centred on it:

| Row | World z | Target z (base_link) | Solved | Torso |
|---|---|---|---|---|
| 1 | 1.577 | 1.391 | ✅ 0.0 mm | 0.322 |
| 2 | 1.247 | 1.061 | ✅ 0.0 mm | 0.287 |
| 3 | 0.917 | 0.731 | ✅ 0.0 mm | 0.279 |
| 4 | 0.587 | 0.401 | ✅ 0.0 mm | 0.252 |

**All four rows reach exactly.** The torso values cluster in 0.252–0.322, so a single
torso height may serve every row — worth exploiting, since a full torso stroke costs 10
seconds and completion time is the tie-breaker.

> `base_link` sits 0.186 m above the floor, so target z in `base_link` is world z − 0.186.

17 unit tests cover the chain, the rotation maths, reachability of all four rows, limit
compliance, IK round-tripping and correct failure when out of reach. 45 tests in total.

## Open work

1. **Grasp sequence.** Not started. Deploy from tuck → position in front of the book →
   open → advance → clamp → withdraw → return to tuck with the book held.
2. **Whether to use MoveIt or direct joint trajectories.** MoveIt 2 is installed and
   configured for TIAGo Pro, but it is heavy and the shelf is a tight, cluttered workspace.
   Direct trajectories to a small number of measured poses may be more reliable and much
   cheaper. Worth trying the simple route first given the deadline.
3. **Gentle placement is worth +4 against +2 for a drop.** That is the single largest
   scoring item in the task; the placement motion deserves proportionate attention.
4. **Confirm the row numbering direction** with the organisers before trusting row → height
   (see `ORGANISER_QUESTIONS.md`).

---

## The arm does not sag (2026-09-08)

Measured with `tools/in-sim sagcheck.py`, robot parked 2.46 m from the shelf so every
target is in open air, torso pinned as the grasp controller would pin it:

| Row | Target z (base_link) | Torso | Reach | Est. torque | Miss at x = 0.83 |
|---|---|---|---|---|---|
| 1 | 1.391 | 0.350 | 78% | 94% | **4 mm** |
| 2 | 1.061 | 0.350 | 70% | 131% | **3 mm** |
| 3 | 0.731 | 0.304 | 74% | 122% | **6 mm** |
| 4 | 0.401 | 0.000 | 75% | 123% | **4 mm** |

**The arm holds every row to within 6 mm, at up to 131 per cent of estimated rated
torque.** Sag is not a thing that happens here, and the torque estimate is not a
predictor of anything — which matches the note above about it reading 6.6 Nm on a joint
drawing 27.

### What this corrects

Earlier the same day, the same tool reported the arm settling 36 to 51 mm below the row-4
target and that was written up as sag agreeing with a servo failure. It was wrong. The
robot was parked 0.675 m from the shelf at the time, so targets at x = 0.75 and 0.83 in
base_link were 75 to 155 mm INSIDE the shelf boards: the arm was not sagging, it was
pressing against a shelf. The agreement with the servo was a coincidence.

**Any sagcheck run with the robot in front of the shelf measures the shelf.** The tool
checks postures against the planning scene, and the shelf is only in the planning scene
while the grasp controller has put it there.

### One artefact to know about

The first x in each sweep reports a large miss (+80 mm in x, −330 mm in z) and the second
does not. The pattern is identical at all four heights, which no real reach limit would
be, and it is almost certainly the arm still travelling when the first sample is taken.
Treat the first row of a sweep as a warm-up, or pass the x of interest twice.

### So why did the row-4 servo fail?

Not sag. On that run the servo reported "-3 mm depth, -8 mm sideways, -40 mm height after
40 s, closest it came was 36 mm" with **zero IK solves rejected**, and the arm had already
arrived at the pre-grasp +15 mm high. An arm that tracks free-air targets to 4 mm does not
miss by 40 mm unless something is in the way, and by then the grasp has deliberately taken
the shelf OUT of the planning scene so that the reach can be planned at all. Nothing is
checking the final approach against the shelf it is reaching into.

That is the open question for the bottom row, and it is a different question from the one
that was being asked.

---

## Why the jaws close on air: a position-controlled gripper cannot close on an object (2026-09-08)

Three clamps in twenty runs, every one on nothing, with the book never moving from
x = 2.900 and the finger joint settling at 0.0010 -- a span of 28.7 mm around a book
30.0 mm thick. The jaws did not stop at the book. They went through it.

That is not a bug in this solution. It is what a position-commanded gripper does in
Gazebo, and it is well documented.

### The mechanism, from the robot description

    gripper_left_finger_joint    prismatic, command=['position'], effort=10.0
    gripper_left_fingertip_*     revolute,  mimic of the above x 8.28, effort=0.1
    gripper_left_inner_finger_*  revolute,  mimic of the above x -8.28, effort=0.1

Only the prismatic joint is actuated. Every link that actually touches the book is a
**mimic** joint following it through a passive four-bar. Under dartsim those mimics are
not enforced by the physics at all — the engine refuses to create the constraints and says
so at every startup (§1) — so the fingers are moved by `gz_ros2_control` writing their
positions. A book between the pads has nothing to push back against: the driven joint goes
where it is told and the linkage follows.

`gz_ros2_control` turns a position command into a velocity command --
`target_vel = -position_proportional_gain * error`, see the comment in
`config/gazebo_controller_manager_cfg.yaml` -- which does not change the conclusion.

### It is a known limitation, not a local mistake

The ROS answers archive and the gazebo-pkgs issue tracker both describe it in the same
terms: objects cannot be grasped through a position hardware interface, because the
fingers move to the commanded position regardless of what is between them. The two
remedies in general use are

  1. **effort control on the gripper joints**, with PID tuning, so the fingers push
     rather than teleport; and
  2. **a grasp fix**: detect contact between the pads and the object, then create a
     fixed joint between them and stop closing. In Gazebo Harmonic the mechanism is the
     `DetachableJoint` system; in Gazebo Classic it was `gazebo_grasp_plugin`.

  3. **switch to bullet-featherstone**, which is the only Gazebo Harmonic engine that
     implements mimic constraints — measured in §1b, and the only one of the three that
     addresses the cause rather than working around it.

References:
  - https://answers.ros.org/question/352107/  (object slips from gripper in Gazebo)
  - https://github.com/JenniferBuehler/gazebo-pkgs/issues/9  (grasp fix, why it exists)
  - http://docs.ros.org/en/indigo/api/gazebo_grasp_plugin/html/classgazebo_1_1GazeboGraspFix.html
  - https://github.com/ros-controls/gz_ros2_control/issues/340  (the same mimic error)
  - https://github.com/gazebosim/gz-physics/pull/517  (mimic constraints, bullet-featherstone only)
  - https://github.com/ros-controls/gz_ros2_control/issues/440  (controllers under bullet-featherstone)

### What this means for the measurements already taken

The grip check added on 2026-09-08 -- fail the grasp when the closed span is more than
2 mm under the book thickness -- is correct and will now fail EVERY grasp, because a
position-controlled joint always reaches full closure. That is the honest reading and it
should stay until the gripper can actually hold something.

It also retires a line of investigation. The pads were being tracked to the millimetre
against the book to find out why they missed, and on the last clamp they were 53 mm into
a 160 mm book with the jaws open to 65 mm around a 30 mm spine. They were not missing.
