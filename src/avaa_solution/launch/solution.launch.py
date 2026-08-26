"""Team AVAA — ERC 2026 solution entry point.

The organisers invoke this file, and only this file:

    ros2 launch avaa_solution solution.launch.py shelf_column_number:=2 book_colour:=red

Both argument names are fixed by the competition specification. A submission that does
not accept them exactly is not evaluated, so they are validated here and the launch is
aborted with a clear message rather than starting a run that cannot score.

Note that both arguments are *semantic*, not spatial. The column marker digits are
randomised on every simulation load, and the colour->row assignment is randomised
vertically, so `shelf_column_number:=2` refers to whichever physical column happens to
carry the marker "2" on this run. Nothing about position may be hardcoded.

Nodes are kept separate rather than combined into one process: the Phase 1 rubric marks
modularity explicitly, and it means perception can be run and debugged on its own.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VALID_COLUMNS = ["1", "2", "3", "4", "5"]
VALID_COLOURS = ["red", "blue", "green", "yellow"]


def generate_launch_description() -> LaunchDescription:
    shelf_column_number = LaunchConfiguration("shelf_column_number")
    book_colour = LaunchConfiguration("book_colour")
    save_images = LaunchConfiguration("save_images")

    declare_column = DeclareLaunchArgument(
        "shelf_column_number",
        description=(
            "Overhead marker digit of the target shelf column, 1-5. "
            "Marker placement is randomised each run."
        ),
        choices=VALID_COLUMNS,
    )

    declare_colour = DeclareLaunchArgument(
        "book_colour",
        description=(
            "Colour of the target book. The row it sits on is randomised each run "
            "and must be determined from the camera."
        ),
        choices=VALID_COLOURS,
    )

    declare_save_images = DeclareLaunchArgument(
        "save_images",
        default_value="true",
        description=(
            "Write timestamped annotated frames to erc_images/ during the trial. "
            "Leave on for scored runs; the images are worth +2 per identification."
        ),
    )

    perception = Node(
        package="avaa_solution",
        executable="perception",
        name="avaa_perception",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "book_colour": book_colour,
            "save_images": save_images,
        }],
    )

    mission = Node(
        package="avaa_solution",
        executable="mission",
        name="avaa_mission",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "shelf_column_number": shelf_column_number,
            "book_colour": book_colour,
        }],
    )

    return LaunchDescription([
        declare_column,
        declare_colour,
        declare_save_images,
        LogInfo(msg=[
            "[AVAA] target column marker=", shelf_column_number,
            "  book colour=", book_colour,
        ]),
        perception,
        mission,
    ])
