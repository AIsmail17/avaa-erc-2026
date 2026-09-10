"""Where the collection bin is, from the legs of the table it stands on.

Perception finds the bin by its colour and measures the middle of the red it can see, and from
an angle that middle is the corner nearest the camera rather than the bin. The twentieth full
run placed the book over it and the book fell off the rim, 65 mm outside the bin's side wall.

The table is the better reference. Its four legs are small, square and alone in the laser plane
near the bin, and the front laser picks them out: measured against Gazebo's own poses from 1.8
to 3.3 m and at an angle, all four legs came back as clusters within 2 to 3 cm of their true
centres. And the bin stands at a fixed place on the table.

Geometry, from the arena's meshes, in the world: the table top spans x -1.40..-0.60 and
y -0.70..+0.70 with a 70 mm leg at each corner, so leg centres are 1.33 m apart along the table
and 0.73 m apart across it. The bin's centre is at (-0.977, +0.001): 0.342 m behind the middle
of the pair of legs that faces the shelf, and on the table's centre line.
"""
import math

# Leg centres: this far apart along the table, and across it.
LEG_PAIR_LONG = 1.33
LEG_PAIR_SHORT = 0.73
PAIR_TOLERANCE = 0.08

# From the middle of a pair of legs to the bin's centre, along the table's inward normal. For
# a pair across the table the bin is also 23 mm off that line, which is left out.
BIN_BEHIND_LONG_PAIR = 0.342
BIN_BEHIND_SHORT_PAIR = 0.665

# The laser sees the near faces of a leg, so a cluster's centroid falls short of the leg's
# centre by about this much, towards the laser: 1.8 to 2.9 cm measured on four legs.
LEG_FACE_BIAS = 0.025

# A leg cluster: returns within CLUSTER_GAP of the previous one, at least LEG_MIN_RETURNS of
# them, spanning less than LEG_MAX_EXTENT.
CLUSTER_GAP = 0.06
LEG_MIN_RETURNS = 2
LEG_MAX_EXTENT = 0.15


def leg_candidates(points, max_range=4.0):
    """Return the centres of leg-sized clusters of laser returns, as (x, y) in base_link.

    Points are taken in scan order, which is what makes neighbours in the list neighbours on
    the ground.
    """
    clusters = []
    for x, y in points:
        if math.hypot(x, y) > max_range:
            continue
        if clusters and math.hypot(x - clusters[-1][-1][0], y - clusters[-1][-1][1]) < CLUSTER_GAP:
            clusters[-1].append((x, y))
        else:
            clusters.append([(x, y)])
    legs = []
    for cluster in clusters:
        if len(cluster) < LEG_MIN_RETURNS:
            continue
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) >= LEG_MAX_EXTENT:
            continue
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        reach = math.hypot(cx, cy)
        if reach > 0.0:
            cx += LEG_FACE_BIAS * cx / reach
            cy += LEG_FACE_BIAS * cy / reach
        legs.append((cx, cy))
    return legs


def bin_from_legs(legs, near=None, near_limit=0.6):
    """Return (bin centre, inward normal) from the nearest matching pair of legs, or None.

    A pair matches when its spacing is one side of the table. The normal is the table's,
    pointing into it and away from the robot. ``near`` -- perception's sighting of the bin --
    rules out any pair that would put the bin further than ``near_limit`` from it, so a pair
    made of a leg and something else that happens to stand 1.33 m away is not believed.
    """
    best = None
    for i in range(len(legs)):
        for j in range(i + 1, len(legs)):
            a, b = legs[i], legs[j]
            spacing = math.hypot(b[0] - a[0], b[1] - a[1])
            if abs(spacing - LEG_PAIR_LONG) <= PAIR_TOLERANCE:
                behind = BIN_BEHIND_LONG_PAIR
            elif abs(spacing - LEG_PAIR_SHORT) <= PAIR_TOLERANCE:
                behind = BIN_BEHIND_SHORT_PAIR
            else:
                continue
            mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
            nx, ny = (b[1] - a[1]) / spacing, -(b[0] - a[0]) / spacing
            if nx * mid[0] + ny * mid[1] < 0.0:
                nx, ny = -nx, -ny
            centre = (mid[0] + behind * nx, mid[1] + behind * ny)
            if near is not None and math.hypot(centre[0] - near[0],
                                               centre[1] - near[1]) > near_limit:
                continue
            reach = math.hypot(mid[0], mid[1])
            if best is None or reach < best[0]:
                best = (reach, centre, (nx, ny))
    if best is None:
        return None
    return best[1], best[2]


def standoff_goal(centre, normal, distance):
    """Return the point ``distance`` short of the bin centre along the normal, and the heading.

    Standing there facing along the normal puts the bin's centre straight ahead at
    ``distance``, square to the table, with both legs of the pair out beside the base.
    """
    goal = (centre[0] - distance * normal[0], centre[1] - distance * normal[1])
    return goal, math.atan2(normal[1], normal[0])
