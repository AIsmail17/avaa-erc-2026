#!/usr/bin/env python3
"""Reduce a run log to the handful of facts a trial is scored on.

    tools/runsummary.py ~/erc/runs/run-*.log
    tools/runsummary.py --table ~/erc/runs/*.log

The report has to give scores across five trials, detection and grasping success rates,
and an average trial time, and a thirteen-minute run produces several thousand lines of
log to get those six numbers out of. This reads them.

Times are the mission clock, which counts SIMULATED seconds from the phase the trial
starts in -- the only clock a run can be compared against another run with, since the
real-time factor has been measured anywhere between 0.013 and 0.60.
"""
import glob
import re
import sys

STRIP = re.compile(r"\x1b\[[0-9;]*m")
STAMP = re.compile(r"\[(\d+\.\d+)\]")


def parse(path):
    """Pull the milestones out of one log."""
    out = {
        "path": path, "column": None, "column_at": None,
        "row": None, "row_at": None, "approach": None, "grasp": None,
        "clamped": False, "lifted": False, "placed": False,
        "servo": None, "started": None, "last": None, "failures": [],
        "camera_stall": False,
    }
    with open(path, errors="replace") as handle:
        for raw in handle:
            line = STRIP.sub("", raw)
            stamp = STAMP.search(line)
            if stamp:
                t = float(stamp.group(1))
                if out["started"] is None:
                    out["started"] = t
                out["last"] = t

            m = re.search(r"column identified as (\d+) after .* \(([\d.]+) s in\)", line)
            if m:
                out["column"], out["column_at"] = int(m.group(1)), float(m.group(2))
            m = re.search(r"row identified as (\d+) after .* \(([\d.]+) s in\)", line)
            if m:
                out["row"], out["row_at"] = int(m.group(1)), float(m.group(2))

            if "the approach gave up" in line:
                out["approach"] = "failed"
            if re.search(r"phase approach -> grasp", line):
                out["approach"] = "ok"
            m = re.search(r"servo is on the book \(([^)]*)\)", line)
            if m:
                out["servo"] = m.group(1)
            if "clamping -> lifting" in line:
                out["clamped"] = out["lifted"] = True
            if "lifting -> withdrawing" in line:
                out["lifted"] = True
            if re.search(r"phase deliver(y)? -> done|book placed|placed in the bin", line):
                out["placed"] = True
            if "no camera images arrived at all" in line:
                out["camera_stall"] = True
            m = re.search(r"\[ERROR\].*?: (.*)", line)
            if m and len(out["failures"]) < 6:
                text = m.group(1).strip()
                if text not in out["failures"]:
                    out["failures"].append(text)

    if out["approach"] is None and (out["clamped"] or out["servo"]):
        out["approach"] = "ok"
    out["grasp"] = ("ok" if out["clamped"] else
                    "reached" if out["servo"] else "not reached")
    return out


def duration(row):
    if row["started"] is None or row["last"] is None:
        return 0.0
    return row["last"] - row["started"]


def main(argv):
    table = "--table" in argv
    paths = []
    for arg in argv:
        if arg.startswith("--"):
            continue
        paths.extend(sorted(glob.glob(arg)))
    if not paths:
        print(__doc__)
        return 1

    rows = [parse(p) for p in paths]

    if table:
        print("%-34s %7s %6s %5s %9s %7s %8s"
              % ("run", "column", "row", "appr", "grasp", "placed", "wall s"))
        for r in rows:
            print("%-34s %7s %6s %5s %9s %7s %8.0f"
                  % (r["path"].rsplit("/", 1)[-1][:34],
                     r["column"] if r["column"] else "-",
                     r["row"] if r["row"] else "-",
                     r["approach"] or "-", r["grasp"],
                     "yes" if r["placed"] else "no", duration(r)))
        got = [r for r in rows if r["column"]]
        print()
        print("  identified a column   %d of %d" % (len(got), len(rows)))
        print("  identified a row      %d of %d"
              % (len([r for r in rows if r["row"]]), len(rows)))
        print("  approach completed    %d of %d"
              % (len([r for r in rows if r["approach"] == "ok"]), len(rows)))
        print("  book clamped          %d of %d"
              % (len([r for r in rows if r["clamped"]]), len(rows)))
        print("  book placed           %d of %d"
              % (len([r for r in rows if r["placed"]]), len(rows)))
        stalled = [r for r in rows if r["camera_stall"]]
        if stalled:
            print("  CAMERA STALLED IN     %d of %d -- those runs prove nothing"
                  % (len(stalled), len(rows)))
        return 0

    for r in rows:
        print("=" * 72)
        print(r["path"])
        print("  column     %s%s" % (
            r["column"] or "never identified",
            "" if r["column_at"] is None else "  at %.1f s" % r["column_at"]))
        print("  row        %s%s" % (
            r["row"] or "never identified",
            "" if r["row_at"] is None else "  at %.1f s" % r["row_at"]))
        print("  approach   %s" % (r["approach"] or "did not finish"))
        print("  grasp      %s%s" % (
            r["grasp"], "" if not r["servo"] else "   servo: %s" % r["servo"]))
        print("  placed     %s" % ("yes" if r["placed"] else "no"))
        if r["camera_stall"]:
            print("  WARNING    the camera stream stalled during this run; every")
            print("             perception answer after that point is about a stored frame")
        for f in r["failures"]:
            print("  error      %s" % f[:100])
    return 0


sys.exit(main(sys.argv[1:]))
