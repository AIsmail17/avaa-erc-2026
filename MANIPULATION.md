# Manipulation — measured characteristics

Everything here is measured against the running simulation. Feeds report section 4
(Manipulation).

---

## 1. The gripper works, despite the startup error ✅

`erc_bringup` logs this at every startup, in red:

```
[Err] [Physics.cc:1906] Attempting to create a mimic constraint for joint
[gripper_left_inner_finger_left_joint] but the chosen physics engine does not
support mimic constraints, so no constraint will be created.
```

The gripper is a linkage: one actuated joint (`gripper_left_finger_joint`) with the rest
of the finger joints declared as mimics. DART does not implement mimic constraints, so the
warning is real — and it looked like it might make grasping impossible.

**It does not.** Measured fingertip separation (TF distance between
`gripper_left_fingertip_left_link` and `gripper_left_fingertip_right_link`):

| `gripper_left_finger_joint` | Fingertip span |
|---|---|
| 0.000 | **0.0280 m** |
| 0.020 | 0.0444 m |
| 0.040 | 0.0605 m |

Close to linear: `span ≈ 0.028 + 0.82 × joint`.

**A book is 30 mm thick and the closed span is 28 mm**, so the gripper closes past the book
and can clamp it. Open at 0.040 gives 60.5 mm — twice the book thickness, ample clearance
for approaching.

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
