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

Output is deliberately blunt: PICKED UP, DISTURBED, or NOT MOVED, which book, and
how far. Displacement alone is not success -- a book swept onto its side travelled
0.14 m with the fingers closed on air -- so the verdict checks that it is still the
way up it started.
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
    """Position and orientation. The orientation is what separates a pick from a sweep."""
    out = gz("model", "-m", model, "-p")
    lines = [l.strip() for l in out.splitlines()]
    for i, line in enumerate(lines):
        if line.startswith("[") and i + 1 < len(lines) and lines[i + 1].startswith("["):
            try:
                return ([float(v) for v in line.strip("[]").split()],
                        [float(v) for v in lines[i + 1].strip("[]").split()])
            except ValueError:
                return None
    return None


def snapshot(models):
    return {m: pose(m) for m in models}


UPRIGHT_TOL = 0.35      # radians of roll/pitch change that still counts as upright
CARRIED_M = 0.05        # how far a book must travel before it counts as taken


def classify(before, after):
    """Say what actually happened to a book, not merely that something did.

    Displacement alone is not a pick. One run swept a book over on the shelf -- it
    travelled 0.14 m and ended lying on its side with the fingers fully closed on air --
    and a harness that only measured distance called that a success. In the competition
    that is a dropped book and a penalty, not a score.

    So a pick has to be a book that moved AND is still the way up it started.
    """
    if before is None or after is None:
        return "unknown", 0.0
    moved = sum((a - b) ** 2 for a, b in zip(before[0][:3], after[0][:3])) ** 0.5
    if moved <= MOVE_THRESHOLD_M:
        return "still", moved
    tipped = max(abs(_wrap(a - b)) for a, b in zip(before[1][:2], after[1][:2]))
    if tipped > UPRIGHT_TOL:
        return "knocked over", moved
    if moved < CARRIED_M:
        return "nudged", moved
    return "taken", moved


def _wrap(angle):
    while angle > 3.14159265:
        angle -= 2 * 3.14159265
    while angle < -3.14159265:
        angle += 2 * 3.14159265
    return angle



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
    outcomes = []
    for model in models:
        verdict, d = classify(before.get(model), after.get(model))
        if verdict != "still":
            outcomes.append((model, verdict, d))

    taken = [o for o in outcomes if o[1] == "taken"]
    spoiled = [o for o in outcomes if o[1] in ("knocked over", "nudged")]

    if taken:
        print("RESULT: PICKED UP")
    elif spoiled:
        print("RESULT: DISTURBED BUT NOT PICKED UP")
    else:
        print("RESULT: NOT MOVED — no book shifted more than "
              f"{MOVE_THRESHOLD_M:.02f} m. The grasp did not pick anything up.")

    for model, verdict, d in sorted(outcomes, key=lambda t: -t[2]):
        print(f"  {model}: {verdict}, {d:.3f} m")
        print(f"    before {before[model][0]} rpy {before[model][1]}")
        print(f"    after  {after[model][0]} rpy {after[model][1]}")
    if spoiled:
        print(f"  ({len(spoiled)} book(s) disturbed — each one is a penalty in the run)")

    if robot_before and robot_after:
        travelled = sum((a - b) ** 2 for a, b in
                        zip(robot_before[0][:2], robot_after[0][:2])) ** 0.5
        print(f"\nrobot moved {travelled:.2f} m; "
              f"final position {[round(v, 3) for v in robot_after[0][:2]]}")

    print(json.dumps({
        "column": column, "colour": colour,
        "taken": [m for m, v, _ in outcomes if v == "taken"],
        "disturbed": [m for m, v, _ in outcomes if v != "taken"],
    }))


if __name__ == "__main__":
    main()
