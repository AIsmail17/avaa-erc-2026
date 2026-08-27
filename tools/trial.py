#!/usr/bin/env python3
"""Run one trial and judge it against Gazebo, not against our own logs.

    python3 trial.py <shelf_column_number> <book_colour>

Every failure in this project so far has reported success. The grasp sequence ran all its
states and announced "book grasped and stowed" twice while the book sat untouched on the
shelf; the approach announced completion while the target was out of frame; a row was
latched from a column that had not been resolved and drove the arm a metre off. In each
case the logs were clean and only the simulator's ground truth showed otherwise.

So this records every book's true pose before the run, runs the trial, and records them
again. A book that moved is the only evidence that counts.

Output is deliberately blunt: MOVED or NOT MOVED, which book, and how far.
"""
import json
import subprocess
import sys
import time

MOVE_THRESHOLD_M = 0.02   # beyond settling jitter


def gz(*args, timeout=25):
    return subprocess.run(["gz", *args], capture_output=True, text=True,
                          timeout=timeout).stdout


def book_models():
    out = gz("model", "--list")
    return sorted(l.strip(" -") for l in out.splitlines() if "book_col" in l)


def pose(model):
    out = gz("model", "-m", model, "-p")
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return [float(v) for v in line.strip("[]").split()]
            except ValueError:
                return None
    return None


def snapshot(models):
    return {m: pose(m) for m in models}


def distance(a, b):
    if a is None or b is None:
        return None
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def run_node(executable, extra=(), log="/tmp/trial_node.log"):
    cmd = (f"source /opt/erc_ws/install/setup.bash && "
           f"exec python3 -u /opt/erc_ws/install/avaa_solution/lib/avaa_solution/"
           f"{executable} --ros-args -p use_sim_time:=true "
           + " ".join(extra) + f" > {log} 2>&1")
    return subprocess.Popen(["/entrypoint.sh", "bash", "-c", cmd])


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    column, colour = sys.argv[1], sys.argv[2]

    models = book_models()
    if not models:
        print("no books found; is the simulation running?")
        sys.exit(1)
    print(f"tracking {len(models)} books")

    before = snapshot(models)
    robot_before = pose("tiago_pro")

    perception = run_node(
        "perception",
        [f"-p shelf_column_number:={column}", f"-p book_colour:={colour}",
         "-p save_images:=false"],
        "/tmp/trial_perception.log")
    time.sleep(5)
    approach = run_node("approach", log="/tmp/trial_approach.log")

    print(f"trial running: column {column}, {colour} book", flush=True)
    deadline = time.time() + 260
    grasp = None
    while time.time() < deadline:
        time.sleep(10)
        if grasp is None:
            try:
                log = open("/tmp/trial_approach.log", errors="ignore").read()
            except OSError:
                log = ""
            if "verifying -> done" in log:
                print("approach done; starting grasp", flush=True)
                grasp = run_node("grasp", log="/tmp/trial_grasp.log")
            elif "-> failed" in log:
                print("approach FAILED", flush=True)
                break
        else:
            try:
                log = open("/tmp/trial_grasp.log", errors="ignore").read()
            except OSError:
                log = ""
            if "stowing -> done" in log or "-> failed" in log:
                break

    for proc in (perception, approach, grasp):
        if proc is not None:
            proc.terminate()
    subprocess.run(["pkill", "-f", "avaa_solution/lib"], capture_output=True)
    time.sleep(3)

    after = snapshot(models)
    robot_after = pose("tiago_pro")

    print("\n=== judged against Gazebo ===")
    moved = []
    for model in models:
        d = distance(before.get(model), after.get(model))
        if d is not None and d > MOVE_THRESHOLD_M:
            moved.append((model, d))

    if not moved:
        print("RESULT: NOT MOVED — no book shifted more than "
              f"{MOVE_THRESHOLD_M:.02f} m. The grasp did not pick anything up.")
    else:
        print("RESULT: MOVED")
        for model, d in sorted(moved, key=lambda t: -t[1]):
            print(f"  {model}: {d:.3f} m")
            print(f"    before {before[model]}")
            print(f"    after  {after[model]}")

    if robot_before and robot_after:
        print(f"\nrobot moved {distance(robot_before, robot_after):.2f} m; "
              f"final position {[round(v, 3) for v in robot_after[:2]]}")

    print(json.dumps({
        "column": column, "colour": colour,
        "moved": [m for m, _ in moved],
    }))


if __name__ == "__main__":
    main()
