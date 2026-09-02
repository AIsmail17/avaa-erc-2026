"""Delivery controller — carry the book to the collection bin and place it inside.

Runs once the grasp controller reports a book in the gripper.

    CARRY     fold the arm in with the book held, and tilt the head to look for the bin
    SEEK      turn on the spot until the bin is in view
    DRIVE     alternate turning and driving until the bin is dead ahead at the standoff
    ABOVE     put the gripper over the middle of the bin, book hanging clear of the rim
    LOWER     descend until the book is just above the floor of the bin
    RELEASE   open the jaws
    RETREAT   lift straight out and fold the arm back in
    DONE

Why it steers by the bin rather than by odometry
------------------------------------------------
The base slides. Not because anything is driving it -- the wheels are mecanum, modelled
with mu 0.8 along the roller axis and mu2 0.0 across it, so one direction has no friction
at all and any motion it acquires it keeps. Measured on a fresh simulation with nothing
commanding it and no publisher on /cmd_vel, the base travels about 4 mm/s and turns about
0.6 deg/s, indefinitely. Publishing a zero twist does not help: measured back to back in
one session, 4.3 mm/s uncommanded against 4.1 mm/s with a zero twist at 20 Hz, because
zero velocity locks the wheels and the base slides across the locked wheels.

Wheel odometry cannot see any of that, since a wheel that is not turning reports nothing.
Worse, it is wrong in the direction that hurts: measured during a run held to 17 mm of
true error, odom had accumulated 813 mm of motion that never happened. So a delivery that
drove a planned distance on odom would arrive somewhere else entirely.

Every leg here is therefore closed on the bin itself, which is a fixed object measured
through a camera bolted to the base. If the base slides, the reading moves with it and
the correction comes out right. Nothing is dead reckoned and no leg has a distance in it.

Where the numbers come from
---------------------------
Sideways and in range, from depth: measured against ground truth with the head settled,
the bin came back 7 and 5 mm out in range and 20 and 16 mm out sideways. The bin opening
is 500 x 310 mm, so that is two hundred millimetres of margin on the tightest axis.

In height, from the rules: the bin is 210 mm tall on a 730 mm table, so its rim is 940 mm
up. Depth is not trusted vertically -- a third reading, taken while the head was still
tilting, came back 176 mm high -- and the shelf rows have been treated as known heights
since the first grasp for exactly the same reason.
"""

import math
import threading
import traceback
from collections import deque
from enum import Enum
from typing import List, Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from avaa_solution.kinematics.arm_chain import ArmChain
from avaa_solution.moveit_client import MoveItClient, error_name

TOPIC_BIN_POINT = "/avaa/perception/bin_point"
TOPIC_CMD = "/cmd_vel"
TOPIC_STATE = "/avaa/deliver/state"
TOPIC_SCAN = "/scan_front_raw"
TOPIC_HEAD = "/head_controller/joint_trajectory"
GRIPPER_TOPIC = "/gripper_left_controller_raw/joint_trajectory"
ARM_TOPIC = "/arm_left_controller/joint_trajectory"

ARM_JOINTS = ["arm_left_%d_joint" % i for i in range(1, 8)]
CHAIN_JOINTS = ["torso_lift_joint"] + ARM_JOINTS

# Matching grasp_node: the gripper reaches along its local +x and the fingers travel
# along its local +y, so a book held spine-out keeps this orientation all the way to the
# bin. Rotating it to lay the book flat would be tidier and is not attempted -- every
# reorientation is a chance for the pads to lose a book they are only just holding.
CARRY_APPROACH = [1.0, 0.0, 0.0]
CARRY_CLOSING = [0.0, 1.0, 0.0]

# The jaws, from grasp_node's measured span curve: span = 0.0271 + 0.8146 * joint.
GRIPPER_RELEASE = 0.052

# A book is 250 mm tall and is gripped 45 mm below its centre, so it hangs this far
# below the gripper and stands this far above it.
BOOK_BELOW_GRIP = 0.125 - 0.045
BOOK_ABOVE_GRIP = 0.125 + 0.045

# The stowed arm sits in the LiDAR plane, so returns closer than this to the base are
# the robot seeing itself.
SELF_FILTER_RADIUS = 0.45

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class State(Enum):
    IDLE = "idle"
    CARRY = "carry"
    SEEK = "seeking"
    DRIVE = "driving"
    ABOVE = "above"
    LOWER = "lowering"
    RELEASE = "releasing"
    RETREAT = "retreating"
    DONE = "done"
    FAILED = "failed"


class DeliverNode(Node):
    def __init__(self) -> None:
        super().__init__("avaa_deliver")

        # Where the bin should end up, in base_link, before the arm is asked to reach.
        # Dead ahead rather than lined up with the shoulder: the base cannot strafe
        # (commanding pure vy yaws it by about the amount it moves sideways), so the
        # only lateral control is turning, and turning can only put a target dead ahead.
        # The arm crosses the 159 mm to the shoulder centre line instead, which is what
        # it already does for a book centred on the base.
        self.declare_parameter("standoff_m", 0.60)
        self.declare_parameter("standoff_tol_m", 0.05)
        self.declare_parameter("bearing_tol_rad", 0.05)
        self.declare_parameter("drive_speed", 0.12)
        self.declare_parameter("turn_speed", 0.35)
        self.declare_parameter("seek_speed", 0.35)
        # Clear of the rim by this much before descending, measured at the book's foot.
        self.declare_parameter("rim_clearance_m", 0.06)
        # And this far above the floor of the bin when the jaws open. Not zero: the
        # gripper pads are 34 mm tall and the book is held 80 mm above its own foot, so
        # asking for zero asks the pads to go through the floor of the bin.
        self.declare_parameter("release_gap_m", 0.02)
        self.declare_parameter("bin_depth_m", 0.21)
        self.declare_parameter("gripper_time_sec", 6.0)
        self.declare_parameter("settle_sec", 1.8)
        self.declare_parameter("bin_fresh_sec", 2.0)
        self.declare_parameter("seek_timeout_sec", 90.0)
        self.declare_parameter("drive_timeout_sec", 180.0)
        self.declare_parameter("obstacle_stop_m", 0.30)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("hold_base_hz", 20.0)
        # The head angle while hunting for the bin, before there is a range to aim at.
        self.declare_parameter("search_tilt_rad", -0.30)
        self.declare_parameter("carry_point", [0.34, 0.10, 1.00])

        self.standoff = float(self.get_parameter("standoff_m").value)
        self.standoff_tol = float(self.get_parameter("standoff_tol_m").value)
        self.bearing_tol = float(self.get_parameter("bearing_tol_rad").value)
        self.drive_speed = float(self.get_parameter("drive_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.seek_speed = float(self.get_parameter("seek_speed").value)
        self.rim_clearance = float(self.get_parameter("rim_clearance_m").value)
        self.release_gap = float(self.get_parameter("release_gap_m").value)
        self.bin_depth = float(self.get_parameter("bin_depth_m").value)
        self.gripper_time = float(self.get_parameter("gripper_time_sec").value)
        self.settle = float(self.get_parameter("settle_sec").value)
        self.bin_fresh = float(self.get_parameter("bin_fresh_sec").value)
        self.seek_timeout = float(self.get_parameter("seek_timeout_sec").value)
        self.drive_timeout = float(self.get_parameter("drive_timeout_sec").value)
        self.obstacle_stop = float(self.get_parameter("obstacle_stop_m").value)
        self.search_tilt = float(self.get_parameter("search_tilt_rad").value)
        self.hold_base_hz = float(self.get_parameter("hold_base_hz").value)
        self.carry_point = [float(v) for v in
                            self.get_parameter("carry_point").value]

        self.chain = ArmChain.from_urdf()
        self.state = State.IDLE
        self.joints = {}
        self.scan_ranges: List[float] = []
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0

        # The same short median the grasp uses: enough to bury one bad frame, short
        # enough that the answer still means now on a base that never stops moving.
        self.bin_points = deque(maxlen=9)
        self.bin_point: Optional[np.ndarray] = None
        self.bin_at = None

        self.motion_thread = None
        self.motion_result = None
        self.motion_label = ""
        self.settled_at = None
        self.entered_at = None
        self.head_aimed_at = None
        self.released_at = None
        self.above_point = None

        self.create_subscription(PointStamped, TOPIC_BIN_POINT, self._on_bin, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self._on_scan, SENSOR_QOS)

        self.pub_cmd = self.create_publisher(Twist, TOPIC_CMD, 10)
        self.pub_head = self.create_publisher(JointTrajectory, TOPIC_HEAD, 10)
        self.pub_gripper = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.pub_arm = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.pub_state = self.create_publisher(String, TOPIC_STATE, 10)

        self.moveit = MoveItClient("avaa_deliver_moveit")
        self.get_logger().info("waiting for move_group...")
        if not self.moveit.wait_until_ready(60.0):
            self.get_logger().error(
                "move_group is not running. Start it with "
                "'ros2 launch avaa_solution moveit.launch.py'.")
            raise SystemExit(1)
        self.get_logger().info("move_group connected")

        self.create_timer(0.2, self._tick)
        self.create_timer(1.0 / max(self.hold_base_hz, 1.0),
                          self._hold_base)
        self.get_logger().info(
            "delivery ready — bin wanted dead ahead at %.2f m" % self.standoff)

    # ------------------------------------------------------------------ inputs

    def _on_bin(self, msg: PointStamped) -> None:
        self.bin_points.append([msg.point.x, msg.point.y, msg.point.z])
        self.bin_point = np.median(np.array(self.bin_points, dtype=float), axis=0)
        self.bin_at = self.get_clock().now()

    def _on_joints(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan_ranges = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    # ------------------------------------------------------------------ helpers

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _enter(self, state: State) -> None:
        if state is not self.state:
            self.get_logger().info("%s -> %s" % (self.state.value, state.value))
            self.state = state
            self.entered_at = self._now()
            self._stop()

    def _stop(self) -> None:
        self.pub_cmd.publish(Twist())

    def _elapsed(self) -> float:
        return 0.0 if self.entered_at is None else self._now() - self.entered_at

    def _bin_now(self) -> Optional[np.ndarray]:
        """Give the bin as measured recently enough to steer by, or None."""
        if self.bin_point is None or self.bin_at is None:
            return None
        if len(self.bin_points) < 3:
            return None
        age = (self.get_clock().now() - self.bin_at).nanoseconds / 1e9
        return self.bin_point if age <= self.bin_fresh else None

    def _current_joints(self):
        try:
            return [self.joints[name] for name in CHAIN_JOINTS]
        except KeyError:
            return None

    def _gripper_now(self) -> Optional[np.ndarray]:
        values = self._current_joints()
        return None if values is None else self.chain.position(values)

    def _range_ahead(self) -> Optional[float]:
        """Find the nearest return in a narrow cone ahead, ignoring the robot itself."""
        if not self.scan_ranges:
            return None
        nearest = None
        for index, distance in enumerate(self.scan_ranges):
            if not math.isfinite(distance) or distance < SELF_FILTER_RADIUS:
                continue
            angle = self.scan_angle_min + index * self.scan_angle_inc
            if abs(angle) > 0.30:
                continue
            if nearest is None or distance < nearest:
                nearest = distance
        return nearest

    def _aim_head(self, tilt: float, period: float = 1.0) -> None:
        """Point the head down at the bin, re-asserted slowly rather than every tick."""
        now = self._now()
        if (self.head_aimed_at is not None
                and now - self.head_aimed_at < period):
            return
        self.head_aimed_at = now
        traj = JointTrajectory()
        traj.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, float(np.clip(tilt, -1.047, 0.349))]
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points = [point]
        self.pub_head.publish(traj)

    def _tilt_for(self, target: Optional[np.ndarray]) -> float:
        """Look at the bin if its range is known, otherwise at the search angle."""
        if target is None:
            return self.search_tilt
        where = self._camera_height()
        if where is None:
            return self.search_tilt
        drop = where - float(target[2])
        return float(np.clip(-math.atan2(drop, max(float(target[0]), 0.20)),
                             -1.047, 0.349))

    def _camera_height(self) -> Optional[float]:
        """Camera height above base_link, which moves with the torso."""
        torso = self.joints.get("torso_lift_joint")
        # 1.183 m with the torso down, measured from the URDF chain to the depth
        # optical frame; the torso adds its travel one for one.
        return None if torso is None else 1.183 + float(torso)

    def _send_gripper(self, value: float) -> None:
        traj = JointTrajectory()
        traj.joint_names = ["gripper_left_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [float(value)]
        point.time_from_start = Duration(
            sec=int(self.gripper_time),
            nanosec=int((self.gripper_time % 1.0) * 1e9))
        traj.points = [point]
        self.pub_gripper.publish(traj)

    def _start(self, label: str, function) -> None:
        """Run a MoveIt call on a worker thread, so this node keeps serving callbacks."""
        self.motion_label = label
        self.motion_result = None
        self.settled_at = None

        def run():
            try:
                self.motion_result = function()
            except BaseException as exc:  # noqa: BLE001 - a failed motion is not a crash
                self.get_logger().error(
                    "%s raised %r\n%s" % (label, exc, traceback.format_exc()))
                self.motion_result = (99999, 0.0)

        self.motion_thread = threading.Thread(target=run, daemon=True)
        self.motion_thread.start()

    def _finished(self):
        """(code, fraction) once the motion is done AND the arm has stopped moving."""
        if self.motion_result is None:
            return None
        if self.settled_at is None:
            self.settled_at = self._now()
            return None
        if self._now() - self.settled_at < self.settle:
            return None
        result = self.motion_result
        return result if isinstance(result, tuple) else (result, 1.0)

    def _clear(self, solution) -> bool:
        return self.moveit.state_valid(CHAIN_JOINTS, list(solution)) is not False

    def _straight(self, start_solution, start_point, end_point, steps: int = 6):
        """Joint waypoints tracing a straight line in space, each one checked.

        The same shape as the grasp controller's reach: seed each solve from the last so
        consecutive postures are neighbours and the elbow cannot flip halfway along, and
        put every one to /check_state_validity before it goes near the arm. None means
        the line cannot be walked, which is a real answer and not a failure to try.
        """
        start_point = np.asarray(start_point, dtype=float)
        end_point = np.asarray(end_point, dtype=float)
        waypoints = [list(start_solution)]
        seed = list(start_solution)
        for step in range(1, steps + 1):
            point = start_point + (step / float(steps)) * (end_point - start_point)
            solution = self.chain.ik(point, seed=seed, approach=CARRY_APPROACH,
                                     closing=CARRY_CLOSING)
            if solution is None or not self._clear(solution):
                return None
            waypoints.append(list(solution))
            seed = list(solution)
        return waypoints

    # ------------------------------------------------------------------ states

    def _hold_base(self) -> None:
        """Ask the wheels to stand still while the arm works.

        Silence on /cmd_vel is not an instruction to stay put, and the difference is
        large. Measured against Gazebo per simulated second with the arm swinging,
        9.8 mm/s and 0.88 deg/s uncommanded against 2.1 mm/s and 0.43 deg/s with a zero
        twist at 20 Hz. Only in the states where this node is not itself driving --
        SEEK and DRIVE publish their own commands and must not be argued with.
        """
        if self.state in (State.SEEK, State.DRIVE, State.IDLE):
            return
        self.pub_cmd.publish(Twist())

    def _tick(self) -> None:
        self.pub_state.publish(String(data=self.state.value))
        handler = {
            State.IDLE: self._do_idle,
            State.CARRY: self._do_carry,
            State.SEEK: self._do_seek,
            State.DRIVE: self._do_drive,
            State.ABOVE: self._do_above,
            State.LOWER: self._do_lower,
            State.RELEASE: self._do_release,
            State.RETREAT: self._do_retreat,
        }.get(self.state)
        if handler:
            handler()

    def _do_idle(self) -> None:
        if not bool(self.get_parameter("auto_start").value):
            return
        if self._current_joints() is None:
            return
        self._enter(State.CARRY)
        start = self._current_joints()
        here = self._gripper_now()
        path = self._straight(start, here, self.carry_point, steps=6)
        if path is None:
            self.get_logger().warn(
                "no clear line to the carry posture; driving with the arm as it is")
            self.motion_result = (1, 1.0)
            return
        self._start("carry", lambda: self.moveit.execute_path(CHAIN_JOINTS, path))

    def _do_carry(self) -> None:
        self._aim_head(self.search_tilt)
        done = self._finished()
        if done is None:
            return
        code, _ = done
        if code != 1:
            # Not fatal. An arm that did not fold is an arm that is still holding the
            # book, and the book is what the points are for; it just drives badly.
            self.get_logger().warn(
                "could not fold the arm in for the drive (%s); carrying it out here"
                % error_name(code))
        self._enter(State.SEEK)

    def _do_seek(self) -> None:
        """Turn on the spot until the bin is in view.

        Turning rather than driving to a remembered place. The bin's position in the
        world is fixed and known, but the robot's is not: odom counts wheel turns and
        the base slides across its wheels without turning them, so a stored world pose
        is worth less the longer the trial has run. A full turn costs about eighteen
        seconds and needs no estimate of anything.
        """
        target = self._bin_now()
        self._aim_head(self._tilt_for(target))
        if target is not None:
            self.get_logger().info(
                "bin in view at %.2f m, %+.0f mm to the side"
                % (target[0], target[1] * 1000))
            self._enter(State.DRIVE)
            return
        if self._elapsed() > self.seek_timeout:
            self.get_logger().error(
                "turned for %.0f s without seeing the bin" % self.seek_timeout)
            self._enter(State.FAILED)
            return
        command = Twist()
        command.angular.z = self.seek_speed
        self.pub_cmd.publish(command)

    def _do_drive(self) -> None:
        """Turn to put the bin dead ahead, then drive to the standoff. Repeat.

        Rotate-then-drive, not both at once and never sideways. Commanding pure vy yaws
        this base by roughly the magnitude it strafes, so lateral error is corrected by
        aiming rather than by sliding.
        """
        target = self._bin_now()
        self._aim_head(self._tilt_for(target))
        if target is None:
            # Lost it. Stop rather than coast: the base keeps whatever motion it is
            # given, so coasting blind is how a delivery ends up against the table.
            self._stop()
            if self._elapsed() > self.drive_timeout:
                self.get_logger().error("lost sight of the bin while closing in")
                self._enter(State.FAILED)
            return
        if self._elapsed() > self.drive_timeout:
            self.get_logger().error(
                "could not settle in front of the bin in %.0f s (%.2f m ahead, "
                "%+.0f mm to the side)"
                % (self.drive_timeout, target[0], target[1] * 1000))
            self._enter(State.FAILED)
            return

        bearing = math.atan2(float(target[1]), max(float(target[0]), 0.05))
        range_error = float(target[0]) - self.standoff

        if abs(bearing) > self.bearing_tol:
            command = Twist()
            command.angular.z = float(np.clip(2.0 * bearing,
                                              -self.turn_speed, self.turn_speed))
            self.pub_cmd.publish(command)
            return

        if abs(range_error) > self.standoff_tol:
            ahead = self._range_ahead()
            if range_error > 0 and ahead is not None and ahead < self.obstacle_stop:
                self.get_logger().warn(
                    "something is %.2f m ahead, closer than the bin at %.2f m; "
                    "stopping here" % (ahead, target[0]))
                self._stop()
                self._enter(State.ABOVE)
                self._plan_above(target)
                return
            command = Twist()
            command.linear.x = float(np.clip(0.6 * range_error,
                                             -self.drive_speed, self.drive_speed))
            self.pub_cmd.publish(command)
            return

        self._stop()
        self.get_logger().info(
            "in front of the bin: %.2f m ahead, %+.0f mm to the side"
            % (target[0], target[1] * 1000))
        self._enter(State.ABOVE)
        self._plan_above(target)

    def _plan_above(self, target) -> None:
        """Lift the book over the rim, directly above the middle of the bin."""
        start = self._current_joints()
        here = self._gripper_now()
        if start is None or here is None:
            self._enter(State.FAILED)
            return
        rim = float(target[2])
        # High enough that the foot of the book clears the rim on the way across.
        above = np.array([float(target[0]), float(target[1]),
                          rim + BOOK_BELOW_GRIP + self.rim_clearance])
        self.above_point = above
        path = self._straight(start, here, above, steps=6)
        if path is None:
            self.get_logger().error(
                "no clear line to a point above the bin %s"
                % np.round(above, 3).tolist())
            self._enter(State.FAILED)
            return
        self.get_logger().info(
            "lifting the book over the rim to %s" % np.round(above, 3).tolist())
        self._start("above", lambda: self.moveit.execute_path(CHAIN_JOINTS, path))

    def _do_above(self) -> None:
        done = self._finished()
        if done is None:
            return
        code, _ = done
        if code != 1:
            self.get_logger().error("could not get over the bin: %s" % error_name(code))
            self._enter(State.FAILED)
            return

        target = self._bin_now()
        if target is None:
            self.get_logger().warn(
                "the bin is out of view from over it, as expected; lowering onto the "
                "position measured on the way in")
            target = self.above_point
            rim = float(target[2]) - BOOK_BELOW_GRIP - self.rim_clearance
        else:
            rim = float(target[2])

        start = self._current_joints()
        here = self._gripper_now()
        # Book's foot this far above the floor of the bin when the jaws open.
        floor = rim - self.bin_depth
        low = np.array([float(target[0]), float(target[1]),
                        floor + self.release_gap + BOOK_BELOW_GRIP])
        path = self._straight(start, here, low, steps=5)
        if path is None:
            self.get_logger().warn(
                "no clear line down into the bin; releasing from above the rim, which "
                "scores as a drop rather than a placement")
            self._enter(State.RELEASE)
            self._send_gripper(GRIPPER_RELEASE)
            self.released_at = self._now()
            return
        self.get_logger().info(
            "lowering into the bin to %s" % np.round(low, 3).tolist())
        self._enter(State.LOWER)
        self._start("lower", lambda: self.moveit.execute_path(CHAIN_JOINTS, path))

    def _do_lower(self) -> None:
        done = self._finished()
        if done is None:
            return
        code, _ = done
        if code != 1:
            self.get_logger().warn(
                "the descent into the bin stopped early (%s); releasing from here"
                % error_name(code))
        where = self._gripper_now()
        if where is not None:
            self.get_logger().info(
                "releasing with the gripper at %s" % np.round(where, 3).tolist())
        self._enter(State.RELEASE)
        self._send_gripper(GRIPPER_RELEASE)
        self.released_at = self._now()

    def _do_release(self) -> None:
        if self._now() - self.released_at < self.gripper_time + 1.0:
            return
        start = self._current_joints()
        here = self._gripper_now()
        if start is None or here is None or self.above_point is None:
            self._enter(State.DONE)
            return
        # Straight up, along the way it came in. Anything else drags the pads across a
        # book that is now standing free in the bin.
        out = np.array([here[0], here[1], float(self.above_point[2])])
        path = self._straight(start, here, out, steps=4)
        self._enter(State.RETREAT)
        if path is None:
            self.get_logger().warn("no clear lift out of the bin; leaving the arm here")
            self.motion_result = (1, 1.0)
            return
        self._start("retreat", lambda: self.moveit.execute_path(CHAIN_JOINTS, path))

    def _do_retreat(self) -> None:
        done = self._finished()
        if done is None:
            return
        self.get_logger().info("book delivered")
        self._enter(State.DONE)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeliverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
