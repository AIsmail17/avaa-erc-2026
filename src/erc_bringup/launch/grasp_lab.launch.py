"""The grasp on a bench: the robot, one book, and nothing else.

    ros2 launch erc_bringup grasp_lab.launch.py

or, from outside the container, tools/lab up.

Why this exists
---------------
Every grasp experiment so far has cost a full run: drive out, search for a marker,
identify a column, close in, centre, and only then reach. Thirteen minutes to exercise
thirty seconds of arm motion, with two chances in three of failing somewhere earlier and
never reaching the part under test. This launch drops the shelf, the markers, the walls,
the collection bin, nineteen of the twenty books, both cameras, the depth cloud,
perception, approach and navigation, and stands the robot in front of a single book on a
post.

What that buys, beyond the wait: the real-time factor. The arena world spends most of a
loaded NUC's two cores on collision geometry and camera rendering that a grasp test does
not use.

Environment
-----------
ERC_PHYSICS   dart (default) or bullet -- which physics engine to run.

              This is the reason the bench was built. The gripper's four-bar is seven
              mimic joints following gripper_left_finger_joint, and Gazebo only honours
              mimic constraints under bullet-featherstone. Under dartsim it drops them
              and says so on every launch:

                  [Err] [Physics.cc:1906] Attempting to create a mimic constraint for
                  joint [gripper_left_inner_finger_left_joint] but the chosen physics
                  engine does not support mimic constraints, so no constraint will be
                  created.

              So the linkage has never been simulated at all, which is why the fingers
              pass through each other and why a closing jaw has never pushed a book.
              Switching engines is not free -- gz_ros2_control issue #440 reports
              controllers failing to activate under bullet-featherstone -- which is
              exactly why it gets tested here and not in the arena.

ERC_LAB_X     where the book stands, metres in front of base_link (default 0.75)
ERC_LAB_Y     lateral offset (default 0.0)
ERC_LAB_Z     height of the book's centre above the floor (default 1.00)
ERC_LAB_COLOUR   book colour (default yellow)
ERC_GRASP_FIX    1 to also run the pose-following grasp aid

Launch arguments
----------------
moveit:=true     bring up move_group as well, so the real grasp controller can be run
                 here instead of at the end of a thirteen minute arena run.

                 There is no shelf in this world, which is the point: if a reach that
                 stops short in the arena completes here, the shelf collision geometry is
                 what stops it, and that is a different bug from the arm not reaching.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BOOK_COLOURS = {
    'red': '1 0 0 1',
    'green': '0 1 0 1',
    'blue': '0 0 1 1',
    'yellow': '1 1 0 1',
}

BOOK_TALL = 0.25          # the 0.25 m side, stood on end by the quarter turn in pitch
QUARTER_TURN = 1.5707963267948966

ENGINES = {
    'dart': None,         # the default; no <engine> block at all
    'bullet': 'gz-physics-bullet-featherstone-plugin',
    'bullet-featherstone': 'gz-physics-bullet-featherstone-plugin',
}


def generate_launch_description():
    bringup_dir = get_package_share_directory('erc_bringup')
    arena_dir = get_package_share_directory('erc_description')
    controller_params = os.path.join(bringup_dir, 'config', 'controller_params.yaml')

    book_x = float(os.environ.get('ERC_LAB_X', '0.75'))
    book_y = float(os.environ.get('ERC_LAB_Y', '0.0'))
    book_z = float(os.environ.get('ERC_LAB_Z', '1.00'))
    colour = os.environ.get('ERC_LAB_COLOUR', 'yellow')
    if colour not in BOOK_COLOURS:
        raise ValueError('ERC_LAB_COLOUR must be one of %s' % sorted(BOOK_COLOURS))

    engine_name = os.environ.get('ERC_PHYSICS', 'dart').lower()
    if engine_name not in ENGINES:
        raise ValueError('ERC_PHYSICS must be one of %s' % sorted(ENGINES))
    engine_lib = ENGINES[engine_name]
    engine_xml = ('' if engine_lib is None else
                  '      <engine><filename>%s</filename></engine>' % engine_lib)

    # The post reaches from the floor to the underside of the book.
    stand_top = book_z - BOOK_TALL / 2.0
    if stand_top <= 0.05:
        raise ValueError('ERC_LAB_Z=%.3f leaves no room for the post' % book_z)

    with open(os.path.join(arena_dir, 'worlds', 'grasp_lab.sdf')) as f:
        world = f.read()
    world = (world
             .replace('PHYSICS_ENGINE_PLACEHOLDER', engine_xml)
             .replace('BOOK_STAND_HALF_Z', '%.4f' % (stand_top / 2.0))
             .replace('BOOK_STAND_X', '%.4f' % book_x)
             .replace('BOOK_STAND_Y', '%.4f' % book_y)
             .replace('BOOK_STAND_Z', '%.4f' % stand_top))
    world_file = '/tmp/erc_grasp_lab.sdf'
    with open(world_file, 'w') as f:
        f.write(world)

    urdf_path = os.path.join(arena_dir, 'urdf', 'tiago_pro.urdf')
    with open(urdf_path) as f:
        robot_description = f.read()

    book_sdf_path = os.path.join(arena_dir, 'models', 'book', 'sdf', 'erc_book.sdf')
    with open(book_sdf_path) as f:
        book_sdf = f.read().replace('BOOK_COLOUR_PLACEHOLDER', BOOK_COLOURS[colour])
    book_file = '/tmp/erc_lab_book_%s.sdf' % colour
    with open(book_file, 'w') as f:
        f.write(book_sdf)

    print('[grasp_lab] %s physics, one %s book at (%.2f, %.2f, %.2f) on a %.2f m post'
          % (engine_name, colour, book_x, book_y, book_z, stand_top))

    headless = LaunchConfiguration('headless')
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        condition=UnlessCondition(headless), output='screen')
    _nv_icd = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'
    gazebo_headless = ExecuteProcess(
        cmd=['bash', '-c',
             'unset DISPLAY; '
             f'[ -f {_nv_icd} ] && export __EGL_VENDOR_LIBRARY_FILENAMES={_nv_icd}; '
             f'exec gz sim -r -s "{world_file}"'],
        condition=IfCondition(headless), output='screen')

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': True}],
               output='screen')

    # Only the clock and the contacts. No cameras: perception is not in this test, and
    # rendering two of them is most of what the arena world spends its time on.
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}], output='screen')
    contact_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
        parameters=[{'use_sim_time': True}], output='screen')

    # Facing +x, so world x is straight ahead of the robot and the book's pose reads the
    # same as the arm's reach. The arena spawns it turned a quarter turn; nothing here
    # needs that and it makes every number harder to check by eye.
    spawn_robot = TimerAction(period=3.0, actions=[
        Node(package='ros_gz_sim', executable='create',
             parameters=[{'use_sim_time': True}],
             arguments=['-name', 'tiago_pro', '-topic', 'robot_description',
                        '-x', '0', '-y', '0', '-z', '0.15',
                        '-R', '0', '-P', '0', '-Y', '0'],
             output='screen'),
    ])

    spawn_book = TimerAction(period=5.0, actions=[
        Node(package='ros_gz_sim', executable='create',
             arguments=['-file', book_file, '-name', 'book_lab_%s' % colour,
                        '-x', '%.4f' % book_x, '-y', '%.4f' % book_y,
                        '-z', '%.4f' % book_z,
                        '-R', '0', '-P', str(QUARTER_TURN), '-Y', '0'],
             output='screen'),
    ])

    def spawner(name, params=None):
        args = [name, '-c', '/controller_manager',
                '--controller-manager-timeout', '120',
                '--service-call-timeout', '60']
        if params:
            args += ['--param-file', params]
        return Node(package='controller_manager', executable='spawner',
                    arguments=args, parameters=[{'use_sim_time': True}],
                    output='screen')

    # The right arm, both grippers' partners and the head are all still spawned: the
    # controllers are declared in one config and a missing one is a failure to start,
    # not a saving worth chasing.
    controllers = TimerAction(period=8.0, actions=[
        spawner('joint_state_broadcaster'),
        spawner('arm_left_controller', controller_params),
        spawner('arm_right_controller', controller_params),
        spawner('head_controller', controller_params),
        spawner('torso_controller', controller_params),
        spawner('gripper_left_controller_raw', controller_params),
        spawner('gripper_right_controller_raw', controller_params),
    ])

    gripper_clamp = TimerAction(period=9.0, actions=[
        Node(package='erc_bringup', executable='gripper_command_clamp.py',
             parameters=[{'use_sim_time': True}], output='screen'),
    ])

    # move_group, from the solution's own launch so the bench plans exactly the way a
    # real run does -- same kinematics, same joint limits, same controller mapping.
    moveit = []
    try:
        moveit_launch = os.path.join(
            get_package_share_directory('avaa_solution'), 'launch', 'moveit.launch.py')
        moveit = [TimerAction(period=12.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                condition=IfCondition(LaunchConfiguration('moveit')),
            )])]
    except Exception as exc:  # noqa: BLE001
        print('[grasp_lab] move_group NOT available (%s); the bench can still test the '
              'jaws, but not a planned reach' % exc)

    grasp_fix_wanted = os.environ.get('ERC_GRASP_FIX', '').lower() in ('1', 'true', 'yes')
    grasp_fix = TimerAction(period=6.0, actions=([
        Node(package='erc_bringup', executable='sim_grasp_fix.py',
             parameters=[{'use_sim_time': True}], output='screen'),
    ] if grasp_fix_wanted else []))

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('moveit', default_value='false',
                              description='bring up move_group so a planned reach can be '
                                          'tested here'),
        gazebo_gui, gazebo_headless,
        rsp, bridge, contact_bridge,
        spawn_robot, spawn_book, controllers, gripper_clamp, grasp_fix,
        *moveit,
    ])
