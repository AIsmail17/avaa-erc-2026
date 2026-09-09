"""Where the arena is, in the frame the robot plans in.

One constant joins the two languages this project speaks. Gazebo, the rules and every
ground-truth check speak WORLD, measured from the floor. The planner, the perception
fixes and every target speak BASE_LINK, measured from a frame that rides on the robot.
Everything else here is derived from that one number, in one place, on purpose.

    base_link stands 0.0762 m above the floor.

It is not a guess and it is not 0.186, which is what this project used everywhere until
2026-09-09 and which was wrong by 110 mm.

  * The URDF says so outright: base_footprint_joint has origin xyz="0 0 0.0762", and
    base_link sits at exactly the wheel axle height, which is where TIAGo puts it.
  * TF says so: base_footprint to base_link measures +0.0762.
  * Gazebo agrees with TF about everything else, so the chain is sound. Asked for
    torso_lift_link, the simulator says world z = 0.9424 and TF says 0.9423 from
    base_footprint -- the same number to a tenth of a millimetre, which also settles
    that base_footprint really is the floor plane (tools/worldtf.py).
  * The camera agrees. Every book fix, deprojected from depth and carried through TF
    into base_link, came back 115 mm above ground-truth-minus-0.186 and within 5 mm of
    ground-truth-minus-0.0762 -- at four rows, three head tilts and ranges from 0.65 to
    1.62 m, with a spread of 13 mm (tools/bookheight.py).

Why it went unnoticed for so long is worth keeping, because it is the general shape of
the trap: the tools that measured the miss converted ground truth into base_link with
the SAME wrong constant as the code that aimed the arm. Both sides moved together, the
difference came out as zero, and a reach that was 110 mm low reported as perfect. A
constant that appears on both sides of a comparison is never tested by that comparison.

What it cost, before it was found:

  * every row was aimed 110 mm low, and the grasp aims a further 45 mm below the book
    centre, so the gripper was sent 155 mm below the centre of a book that is 250 mm
    tall -- 30 mm below its bottom edge, into the board it stands on
  * the bottom row read as unreachable, because 0.401 in base_link is 110 mm lower than
    the bottom row actually is and the forearm meets the base down there
  * the shelf boards went into the planning scene 110 mm low, so the planner was
    avoiding boards that were not there and driving through ones that were
  * the bin rim was 110 mm low, so a book was released short of the rim
  * DEPTH_HEIGHT_BIAS grew to 0.152 m to absorb it, which then made a real height
    measurement look far too noisy to identify a row with
"""

# base_footprint_joint, tiago_pro.urdf. The floor to base_link.
BASE_LINK_Z = 0.0762

# The four stocked rows, in world z, as the books actually settle.
#
# simulation.launch.py spawns them at 1.1 + 0.825 - i * 0.33 for i in 1..4, which is
# 1.595 down to 0.605, and they drop 18 mm onto the board before coming to rest. These
# are the settled heights, read back from Gazebo, because that is where the book is when
# the gripper arrives.
ROW_HEIGHTS_WORLD = [1.577, 1.247, 0.917, 0.587]

# The same four rows in base_link, top row first. This is what a grasp aims at.
ROW_HEIGHTS_BASE = [round(z - BASE_LINK_Z, 4) for z in ROW_HEIGHTS_WORLD]

# The rim of the collection bin.
#
# The rules put the bin on a table: table 140 x 80 x 73 cm, bin 50 x 31 x 21 cm, so the
# rim stands 0.94 m above the floor. Gazebo has the bin body centred at 0.845 and it is
# 0.21 tall, which puts the rim at 0.950 -- 10 mm from the arithmetic.
BIN_RIM_WORLD_Z = 0.950
BIN_RIM_BASE_Z = round(BIN_RIM_WORLD_Z - BASE_LINK_Z, 4)

# Rows are this far apart, which is the margin any height check has to work inside.
ROW_SPACING = 0.330

# A book: erc_book.sdf is a 0.25 x 0.03 x 0.16 box pitched 90 degrees, so it stands
# 250 mm tall, is 30 mm across the spine and 160 mm deep.
BOOK_HEIGHT = 0.25
BOOK_WIDTH = 0.03
BOOK_DEPTH = 0.16
