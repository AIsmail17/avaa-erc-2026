#!/usr/bin/env python3
"""Stand a book back up on its shelf, so the next attempt starts from the same place.

    tools/in-sim putback.py book_col_3_row_4_blue
    tools/in-sim putback.py --all

A failed grasp leaves the book on its side, or on the floor, and the next run then
measures something different from the last one. The row and column are in the model name
and the shelf geometry is fixed, so where it belongs is known exactly: the column's
nominal y, the row's settled height, and the 90 degree pitch every book is spawned with.

Sideways jitter is not restored -- the layout randomiser puts each book up to 250 mm off
its column centre and that number is gone once the book has moved. The column centre is
inside that range and squarely in front of the marker, which is the honest choice.
"""
import math
import subprocess
import sys
import time

# simulation.launch.py: five columns a metre apart about the shelf centre at y = 0, the
# stocked rows spawned at 1.1 + 0.825 - i * 0.33 for i in 1..4, settling 18 mm lower.
COLUMN_Y = [((5 - 1) / 2 - col) * 1.0 for col in range(5)]
ROW_Z = [1.1 + 0.825 - i * 0.33 - 0.018 for i in (1, 2, 3, 4)]
SHELF_X = 2.900
PITCH = math.pi / 2.0


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


def place(name):
    try:
        column = int(name.split("book_col_")[1].split("_")[0])
        row = int(name.split("_row_")[1].split("_")[0])
    except (IndexError, ValueError):
        print("  %s: cannot read a column and row out of the name" % name)
        return False
    if not 1 <= column <= 5 or not 2 <= row <= 5:
        print("  %s: column %d row %d is not a stocked position" % (name, column, row))
        return False

    y = COLUMN_Y[column - 1]
    z = ROW_Z[row - 2]
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
    wanted = sys.argv[1:] or ["--all"]
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
    print("standing %d book(s) back up" % len(targets))
    done = sum(1 for name in targets if place(name))
    print("%d of %d placed" % (done, len(targets)))
    return 0 if done == len(targets) else 1


if __name__ == "__main__":
    sys.exit(main())
