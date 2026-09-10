#!/usr/bin/env python3
"""Stand a book back up on its shelf, so the next attempt starts from the same place.

    tools/in-sim putback.py book_col_3_row_4_blue
    tools/in-sim putback.py --all
    tools/in-sim putback.py --all --jitter

A failed grasp leaves the book on its side, or on the floor, and the next run then
measures something different from the last one. The row and column are in the model name
and the shelf geometry is fixed, so where it belongs is known exactly: the column's
nominal y, the row's settled height, and the 90 degree pitch every book is spawned with.

By default the book goes back to its column centre, which is the repeatable choice and
the right one when the thing being measured is the arm. The exact sideways offset the
randomiser gave it is gone once the book has moved, and inventing a new one changes the
experiment between runs.

--jitter puts it back with a fresh offset drawn the way simulation.launch.py draws one,
up to a quarter of the column width either side. That is the layout perception has to
cope with: a column whose books do not line up is what makes grouping them under a marker
hard, and a shelf of perfectly stacked books is an easier problem than the real one.
"""
import math
import random
import subprocess
import sys
import time

# simulation.launch.py: five columns a metre apart about the shelf centre at y = 0, the
# stocked rows spawned at 1.1 + 0.825 - i * 0.33 for i in 1..4, settling 18 mm lower.
COLUMN_Y = [((5 - 1) / 2 - col) * 1.0 for col in range(5)]
ROW_Z = [1.1 + 0.825 - i * 0.33 - 0.018 for i in (1, 2, 3, 4)]
SHELF_X = 2.900
PITCH = math.pi / 2.0
# COLUMN_JITTER_RANGE in simulation.launch.py: a quarter of the 1 m column width.
JITTER = 0.25


def release_whatever_is_held():
    """Tell the simulation grasp aid to let go before anything is teleported.

    sim_grasp_fix holds a picked-up book by setting its pose to follow the gripper, at
    5 Hz, until it sees an OPEN gripper command. It is launched once with the simulator
    and outlives every run, so after a successful pick the book stays welded to the arm
    across runs -- and a teleport into it is overwritten a fifth of a second later.

    That is exactly what happened on the first run after the first successful pick: this
    tool reported the book placed, and the next grasp then failed in its posture search
    because the book it was aiming at was 1.6 m away, riding on the gripper.

    The aid takes anything above 0.040 as opening. It listens on both the public topic
    and the raw one, so publishing to either is enough; publishing to both costs nothing
    and does not depend on which one the solution happens to use this week.
    """
    try:
        import rclpy
        from builtin_interfaces.msg import Duration
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    except ImportError:
        return
    started = False
    try:
        rclpy.init()
        started = True
        node = rclpy.create_node("putback_release")
        pubs = [node.create_publisher(JointTrajectory, topic, 10) for topic in (
            "/gripper_left_controller/joint_trajectory",
            "/gripper_left_controller_raw/joint_trajectory")]
        traj = JointTrajectory()
        traj.joint_names = ["gripper_left_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.052]
        point.time_from_start = Duration(sec=1)
        traj.points = [point]
        for _ in range(12):
            for pub in pubs:
                pub.publish(traj)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.05)
        node.destroy_node()
        print("asked the grasp aid to let go of anything it is holding")
    except Exception as exc:  # noqa: BLE001 - a fixture must not fail on its tidy-up
        print("could not send the release (%s)" % exc)
    finally:
        if started:
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def books():
    for _ in range(8):
        names = sorted(l.strip(" -") for l in gz("model", "--list").splitlines()
                       if "book_col" in l)
        if names:
            return names
        time.sleep(0.5)
    return []


def belongs(name, jitter=False):
    """Where a book called this belongs: (y, z), or None if the name is not a shelf slot."""
    try:
        column = int(name.split("book_col_")[1].split("_")[0])
        row = int(name.split("_row_")[1].split("_")[0])
    except (IndexError, ValueError):
        return None
    if not 1 <= column <= 5 or not 2 <= row <= 5:
        return None
    y = COLUMN_Y[column - 1]
    if jitter:
        y += random.uniform(-JITTER, JITTER)
    return y, ROW_Z[row - 2]


def place(name, y, z):
    # Pitch 90 degrees about y, which is how every book is spawned.
    request = ('name: "%s", position: {x: %f, y: %f, z: %f}, '
               'orientation: {x: 0, y: %f, z: 0, w: %f}'
               % (name, SHELF_X, y, z,
                  math.sin(PITCH / 2.0), math.cos(PITCH / 2.0)))
    out = gz("service", "-s", "/world/erc_world/set_pose", "--reqtype", "gz.msgs.Pose",
             "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", request)
    ok = "true" in out.lower()
    print("  %-28s -> (%.3f, %+.3f, %.3f)  %s"
          % (name, SHELF_X, y, z, "ok" if ok else out.strip() or "no reply"))
    return ok


def main():
    arguments = sys.argv[1:]
    jitter = "--jitter" in arguments
    wanted = [a for a in arguments if a != "--jitter"] or ["--all"]
    names = books()
    if not names:
        print("no books; is the simulator up?")
        return 1
    if wanted == ["--all"]:
        targets = names
    else:
        targets = [n for n in names if any(w == n or w in n for w in wanted)]
    if not targets:
        print("nothing matching %s" % " ".join(wanted))
        return 1
    plan = {}
    for name in targets:
        where = belongs(name, jitter)
        if where is None:
            print("  %s: not a stocked shelf position" % name)
            continue
        plan[name] = where
    if not plan:
        return 1

    release_whatever_is_held()
    time.sleep(1.0)

    print("standing %d book(s) back up%s"
          % (len(plan), ", with fresh sideways jitter" if jitter else ""))
    # Two passes over the SAME plan. Gazebo drops set_pose requests when it is busy, and
    # a book teleported next to one that has not been moved yet can end up leaning on it.
    # Drawing the jitter once and placing twice fixes both without moving the target
    # between the two attempts.
    done = 0
    for attempt in (1, 2):
        done = sum(1 for name, (y, z) in plan.items() if place(name, y, z))
        if done == len(plan):
            break
        print("  pass %d placed %d of %d; trying again" % (attempt, done, len(plan)))
    print("%d of %d placed" % (done, len(plan)))
    return 0 if done == len(plan) else 1


if __name__ == "__main__":
    sys.exit(main())
