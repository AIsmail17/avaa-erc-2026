# Questions for the organising committee

Team AVAA. Everything below is reproducible from the supplied simulator with the
tools in `tools/`, and each item says what was measured and how.

We are not asking for the rules to change. We are asking whether these behaviours
are intended, because several of them are properties of the supplied robot and
world rather than of any team's code, and we would rather ask than quietly work
around something that was meant to work.

**Answered, 2026-09-02.** The committee's position is that teams solve these
themselves. This document is kept as the record of what was measured and how,
because the answers shaped the solution and belong in the report; item 1 in
particular has since been partly solved in our own code, and is corrected below
rather than deleted.

---

## 1. The base will not stay still, and no team code can make it

> **Correction, 2026-09-02.** The figures first written here, and the claim that
> commanding zero velocity does not help, were both wrong. They were measured per
> second of **wall clock**, and this simulator does not run at a fixed speed: a
> freshly launched instance manages a real-time factor of 0.59 and the same
> instance after twenty minutes of experiments manages 0.013. Per wall second at
> RTF 0.013 almost no simulated time passes in a measurement window, so every
> condition looks identical -- which is exactly what we saw, and exactly the wrong
> conclusion. The numbers below are per **simulated** second, with the RTF recorded
> beside each one (`tools/drift.py`, `tools/rtf.py`).

**What happens.** With nothing commanding the base it drifts continuously and does
not decay. Measured against Gazebo per simulated second, in thirty-second windows
at RTF 0.41 to 0.57:

| | translation | yaw |
|---|---|---|
| arm still, nothing commanded | 6.8 mm/s | 0.81 deg/s |
| arm still, zero twist at 20 Hz | 3.1 mm/s | 0.42 deg/s |
| arm swinging, nothing commanded | 9.8 mm/s | 0.88 deg/s |
| arm swinging, zero twist at 20 Hz | 2.1 mm/s | 0.43 deg/s |

and on a base still carrying the momentum of a fresh spawn, 58 mm/s uncommanded
against 1.8 mm/s held.

**So a zero twist does help, by a factor of three to thirty**, and our own code now
publishes one at 20 Hz whenever it is not driving. That is the fix, it is ours, and
it turned a reach that landed 38 mm to one side of the book into one that starts
from the pose it was aimed from. What it does not do is abolish the drift: 2 mm/s
and 0.4 deg/s still accumulate over a reach that takes tens of seconds, which is
why the last centimetres are closed by a perception servo rather than by a plan.

**What we checked.** The four mecanum wheels are modelled

```
<mu>0.80</mu>  <mu2>0.0</mu2>  <fdir1 gz:expressed_in="base_footprint">1 -1 0</fdir1>
```

with the `fdir1` vectors correctly alternated, so four locked wheels should pin
the base in the plane. `fdir1` is an ODE parameter. `erc_world.sdf` declares
`<physics type="ode">`, but gz-sim selects its engine independently and the
console shows dartsim in use, which ignores `fdir1`. What is left is `mu2 = 0` in
some direction of DART's choosing, the same for every wheel.

We tried raising `mu2`. Controlled A/B, identical protocol, fresh simulator each
time: 0.00 gave 212 mm and -25.6 deg over one idle minute, 0.12 gave 172 mm and
-25.4 deg. No meaningful difference, so we reverted to the supplied value. At
`mu2 = 0.80` the drift falls away but the base can no longer turn at all -- asked
for 0.20 rad/s it managed one degree in six seconds -- so that is not a fix
either.

**Questions.**
- Is the world intended to run on ODE, so that `fdir1` is honoured?
- If dartsim is intended, is a residual drift of about 2 mm/s and 0.4 deg/s under
  a held base the expected behaviour? There is no brake in the `MecanumDrive`
  plugin; a zero twist holds the wheels at zero velocity and the base still slides
  across them, just far more slowly than when nothing is published at all.

**A second observation, about the simulator rather than the robot.** The real-time
factor collapses when the robot gets into a jammed contact state and a teleport
clears it: measured in one sitting, 0.033 with the robot at the shelf, 0.365 after
teleporting it to open floor, and 0.433 after teleporting it straight back to the
shelf. The position is not what matters; resetting the contact state is. We mention
it because the trial is timed, and a run whose arm brushes the shelf early will be
measured against a simulator running ten times slower for the rest of it.

---

## 2. Wheel odometry does not see the base move

`/odom` comes from the `MecanumDrive` plugin and is integrated from wheel
rotation. Because the base slides across locked wheels, odom is blind to it: in
one measurement the base travelled 103 mm in 30 seconds while odom reported
2.9 mm. The reverse is worse -- holding station by driving the wheels made odom
accumulate 813 mm of travel that never happened, while the robot was genuinely
held to within 17 mm.

This matters for the navigation phase, which has no map and no absolute
reference. Is `/odom` intended to be usable for anything beyond very short
distances?

---

## 3. The physics engine does not create the gripper's mimic constraints

On every start:

```
[Err] [Physics.cc:1906] Attempting to create a mimic constraint for joint
[gripper_left_inner_finger_left_joint] but the chosen physics engine does not
support mimic constraints, so no constraint will be created.
```

`gz_ros2_control` drives the seven linkage joints itself, and measured through
TF the pads do move with the finger joint, so the gripper is usable. We would
like to know whether the linkage was intended to be physically coupled, since
that changes how it behaves against a contact.

---

## 4. MoveIt is described as configured, but no SRDF ships for this robot

The Phase 1 document states that MoveIt 2 is configured. The image has MoveIt
installed but contains no SRDF for `tiago_pro`, so `move_group` cannot start.
We wrote one, with its collision matrix, and it works -- but if an official SRDF
exists we would rather use it than diverge from every other team.

---

## 5. Nav2's local planner stops short of its goal

DWB drives to roughly 0.3 m from a goal and stops, with the progress checker
aborting at "Failed to make progress" 0.235 m out. We loosened the goal checker
and wrote a separate controller for the final approach, which is reasonable for
manipulation anyway. Is stopping short expected with the supplied parameters?

---

## 6. The arm cannot follow anything at the default controller gain

`gz_ros2_control` turns a position error into a velocity command as

    target_vel = -position_proportional_gain * error

and its default is 0.1, so a joint one radian from its target is driven at
0.1 rad/s. Commanded to a posture and left alone, the arm went from 0.71 rad of
total error to 0.22 over fifty-five seconds and was still creeping. Every symptom
we chased for days came from this: trajectories reported complete while the arm
was still travelling, arrivals tens of millimetres out however good the plan, and
joints reading saturated effort while barely moving.

We set it to 5.0, which is the one change we have made to a supplied file. It is
a controller tuning value rather than a property of the robot -- it does not
alter reach, joint limits, or the 26 Nm and 43 Nm effort ratings, which are all
left as supplied.

Is 0.1 intended? If so we would like to understand how the arm is meant to reach
a commanded posture within a trial. If not, the default is worth correcting for
everyone, because a team that does not find it will conclude their kinematics are
wrong when they are not.

---

## Changes we have made to supplied files

Exactly one, verified against upstream `origin/main`:

    src/erc_bringup/config/gazebo_controller_manager_cfg.yaml   +23 lines

which sets `position_proportional_gain: 5.0` as described in item 6. Everything
else under `src/erc_description` and `src/erc_bringup` is byte-identical to what
was supplied. Our solution lives entirely in `src/avaa_solution`.

Two things we changed while investigating and then reverted, mentioned so the git
history is not misread: the wheel `mu2` was briefly raised to 0.12 for item 1,
and reverted after the controlled comparison showed no benefit. The book friction
needed no change from us -- the fix was already made upstream in `f3da578`.
