"""Grasp controller — take the identified book off the shelf.

Runs once the approach controller has the base in front of the target column.

    PREPARE   torso to height, arm to a pre-grasp pose held back from the shelf
    OPEN      gripper wide
    ADVANCE   arm forward to the book
    CLAMP     close on the spine
    LIFT      small rise to take the weight off the shelf
    WITHDRAW  straight back out, book held
    STOW      return to the driving posture
    DONE

Where the target comes from
---------------------------
Height comes from the **identified row**, not from depth. Distance and lateral offset come
from **depth**. Each is used where it is trustworthy: measured against ground truth, the
depth estimate is good to 15-35 mm in x and y but carries an unexplained systematic bias of
about +0.108 m in z (see PERCEPTION.md section 9). Rows are 0.33 m apart, so that vertical
error would be a third of the gap between shelves; the row identification, which is
reliable, pins the height instead.

Joint targets come from ``kinematics.arm_chain``, built from the URDF. MoveIt is installed
in the competition image but has no SRDF for this robot, so there is no configured planning
group to ask.
"""

from enum import Enum
from typing import List, Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from avaa_solution.kinematics.arm_chain import ArmChain

TOPIC_TARGET_ROW = "/avaa/perception/target_row"
TOPIC_BOOK_POINT = "/avaa/perception/target_book_point"
TOPIC_STATE = "/avaa/grasp/state"
ARM_TOPIC = "/arm_left_controller/joint_trajectory"
TORSO_TOPIC = "/torso_controller/joint_trajectory"
GRIPPER_TOPIC = "/gripper_left_controller_raw/joint_trajectory"

ARM_JOINTS = [f"arm_left_{i}_joint" for i in range(1, 8)]

# Gripper command values, from the measured span curve (span ~= 0.028 + 0.82 * joint):
# a book is 0.03 m thick, and the fully closed span of 0.028 m closes past it.
GRIPPER_OPEN = 0.040     # 60.5 mm, twice the book thickness
GRIPPER_CLAMP = 0.000    # 28 mm, closes onto the spine

# Row heights as gripper z in base_link, rows 1..4. base_link sits 0.186 m above the
# floor, so these are the world shelf heights less that offset.
DEFAULT_ROW_HEIGHTS = [1.391, 1.061, 0.731, 0.401]

# The torso settles 2.5-3 cm short of whatever it is commanded, repeatably and in both
# directions of travel (see MANIPULATION.md). Command past the target to land on it.
TORSO_BIAS = 0.028

# Which way the hand must be held to take a book off a shelf, in base_link, for an
# arm whose seven joints leave four degrees of freedom spare. Without these the
# solver picks a wrist at random: it reached the right point with the approach axis
# 78 degrees out and closed the fingers past the corner of the book. See ArmChain.ik.
GRASP_APPROACH = [1.0, 0.0, 0.0]   # reach straight into the shelf
GRASP_CLOSING = [0.0, 1.0, 0.0]    # fingers close across the spine

# Driving posture, measured to sit inside the base footprint.
TUCK_POSE = [-0.5, -2.4, 0.0, -2.4, 0.0, 0.0, 0.0]


def row_to_height(row: int, heights: List[float], top_down: bool = True) -> Optional[float]:
    """Gripper z in base_link for a shelf row, honouring the numbering direction.

    ``heights`` is ordered top shelf first. With ``top_down`` the competition's row 1 is
    the topmost stocked shelf; otherwise it is the bottom one.

    The direction is a parameter because the rules number the stocked rows 1-4 without
    saying which end row 1 is, while the simulation names them top-down. Getting it
    backwards costs the row identification points and sends the arm to the wrong shelf, so
    it is worth being able to flip with a launch argument rather than a code change.

    Returns None for a row outside the range rather than clamping: a bad row number should
    surface, not quietly aim at the nearest shelf.
    """
    if not heights or not 1 <= row <= len(heights):
        return None
    index = row - 1 if top_down else len(heights) - row
    return heights[index]


class State(Enum):
    IDLE = "idle"
    PREPARE = "preparing"
    OPEN = "opening"
    ADVANCE = "advancing"
    CLAMP = "clamping"
    LIFT = "lifting"
    WITHDRAW = "withdrawing"
    STOW = "stowing"
    DONE = "done"
    FAILED = "failed"


class GraspNode(Node):
    def __init__(self) -> None:
        super().__init__("avaa_grasp")

        self.declare_parameter("row_heights", DEFAULT_ROW_HEIGHTS)
        # Which end of the shelf row 1 refers to. The simulation numbers the stocked rows
        # top-down, and the competition numbers them 1-4 without stating the direction, so
        # this is a parameter rather than an assumption baked into the table. If the
        # organisers clarify otherwise it is a launch argument, not a rewrite.
        self.declare_parameter("rows_top_down", True)
        self.declare_parameter("standoff_m", 0.12)      # pre-grasp gap from the spine
        # How far past the measured face to close the gripper.
        #
        # The depth measurement is good. An earlier note here claimed a 0.114 m near bias
        # at grasping range; that was wrong, and the error was mine. Depth sees a book's
        # FRONT FACE, while the Gazebo pose is its CENTRE, and the book is 0.16 m deep
        # along x -- so the face sits 0.08 m in front of the coordinate I was comparing
        # against. Measured properly, sampled depth was 2.738 m against a true
        # camera-to-face distance of 2.755 m: accurate to 17 mm.
        #
        # So this only has to carry the gripper from the face into the book far enough to
        # grip the spine. 0.05 m is comfortably inside the book's 0.16 m depth and clear
        # of the shelf behind it. The 0.11 m that briefly stood here would have driven the
        # gripper 8 cm into the book.
        self.declare_parameter("grasp_depth_m", 0.05)
        self.declare_parameter("lift_m", 0.03)
        self.declare_parameter("move_time_sec", 4.0)
        self.declare_parameter("gripper_time_sec", 2.0)
        self.declare_parameter("settle_sec", 1.5)
        self.declare_parameter("state_timeout_sec", 40.0)
        self.declare_parameter("auto_start", True)

        self.row_heights: List[float] = list(
            self.get_parameter("row_heights").value or DEFAULT_ROW_HEIGHTS)
        self.rows_top_down = bool(self.get_parameter("rows_top_down").value)
        self.standoff = float(self.get_parameter("standoff_m").value)
        self.grasp_depth = float(self.get_parameter("grasp_depth_m").value)
        self.lift = float(self.get_parameter("lift_m").value)
        self.move_time = float(self.get_parameter("move_time_sec").value)
        self.gripper_time = float(self.get_parameter("gripper_time_sec").value)
        self.settle = float(self.get_parameter("settle_sec").value)
        self.timeout = float(self.get_parameter("state_timeout_sec").value)

        self.chain = ArmChain.from_urdf()
        self.state = State.IDLE
        self.state_since = self._now()
        self.row: Optional[int] = None
        self.book: Optional[np.ndarray] = None
        self.joints = {}
        self.grasp_solution: Optional[List[float]] = None
        self.pre_solution: Optional[List[float]] = None

        self.create_subscription(Int32, TOPIC_TARGET_ROW, self._on_row, 10)
        self.create_subscription(PointStamped, TOPIC_BOOK_POINT, self._on_book, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.pub_arm = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.pub_torso = self.create_publisher(JointTrajectory, TORSO_TOPIC, 10)
        self.pub_gripper = self.create_publisher(JointTrajectory, GRIPPER_TOPIC, 10)
        self.pub_state = self.create_publisher(String, TOPIC_STATE, 10)

        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            f"grasp ready — rows {'top-down' if self.rows_top_down else 'bottom-up'}, "
            f"heights {[round(h, 3) for h in self.row_heights]}"
        )

    # ------------------------------------------------------------------ inputs

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_row(self, msg: Int32) -> None:
        self.row = int(msg.data)

    def _on_book(self, msg: PointStamped) -> None:
        self.book = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _on_joints(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position

    # ------------------------------------------------------------------ helpers

    def _enter(self, state: State) -> None:
        if state is not self.state:
            self.get_logger().info(f"{self.state.value} -> {state.value}")
            self.state = state
            self.state_since = self._now()

    def _elapsed(self) -> float:
        return self._now() - self.state_since

    def _send(self, pub, names, values, seconds) -> None:
        traj = JointTrajectory()
        traj.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in values]
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
        traj.points = [point]
        pub.publish(traj)

    def row_height(self, row: int) -> Optional[float]:
        return row_to_height(row, self.row_heights, self.rows_top_down)

    def _solve(self, target: np.ndarray, seed: Optional[List[float]] = None):
        """Every solve in this node holds the same wrist, not just the grasp itself.

        The lift and the withdraw run through here too, and they matter as much: a
        wrist left free to rotate while the fingers are clamped twists the book back
        out of the shelf instead of drawing it clear.
        """
        return self.chain.ik(target, seed=seed,
                             approach=GRASP_APPROACH, closing=GRASP_CLOSING)

    def _plan(self) -> bool:
        """Work out the pre-grasp and grasp joint targets. False if unreachable."""
        if self.row is None or self.book is None:
            return False
        height = self.row_height(self.row)
        if height is None:
            self.get_logger().error(f"row {self.row} is outside 1..{len(self.row_heights)}")
            return False

        # x and y from depth, z from the row. See the module docstring.
        face_x = float(self.book[0])
        y = float(self.book[1])

        grasp = np.array([face_x + self.grasp_depth, y, height])
        pre = np.array([face_x - self.standoff, y, height])

        pre_solution = self._solve(pre)
        if pre_solution is None:
            self.get_logger().error(
                f"no IK for pre-grasp {np.round(pre, 3).tolist()}")
            return False
        # Seed the grasp solve from the pre-grasp so the two are near neighbours and the
        # advance is a short, straight-ish motion rather than a re-orientation.
        grasp_solution = self._solve(grasp, seed=pre_solution)
        if grasp_solution is None:
            self.get_logger().error(f"no IK for grasp {np.round(grasp, 3).tolist()}")
            return False

        self.pre_solution = pre_solution
        self.grasp_solution = grasp_solution
        self.get_logger().info(
            f"row {self.row} at z={height:.3f}; book at "
            f"x={face_x:.3f} y={y:.3f}; torso {grasp_solution[0]:.3f}"
        )
        return True

    def _command(self, solution: List[float], seconds: float) -> None:
        """Send a full-chain solution, compensating the torso's known undershoot."""
        torso = min(0.35, max(0.0, solution[0] + TORSO_BIAS))
        self._send(self.pub_torso, ["torso_lift_joint"], [torso], seconds)
        self._send(self.pub_arm, ARM_JOINTS, solution[1:], seconds)

    def _offset_solution(self, solution: List[float], dz: float) -> Optional[List[float]]:
        target = self.chain.position(solution) + np.array([0.0, 0.0, dz])
        return self._solve(target, seed=solution)

    # ------------------------------------------------------------------ states

    def _tick(self) -> None:
        self.pub_state.publish(String(data=self.state.value))

        if self.state in (State.DONE, State.FAILED):
            return

        if self.state is not State.IDLE and self._elapsed() > self.timeout:
            self.get_logger().error(f"timed out in {self.state.value}")
            self._enter(State.FAILED)
            return

        handler = {
            State.IDLE: self._do_idle,
            State.PREPARE: self._do_prepare,
            State.OPEN: self._do_open,
            State.ADVANCE: self._do_advance,
            State.CLAMP: self._do_clamp,
            State.LIFT: self._do_lift,
            State.WITHDRAW: self._do_withdraw,
            State.STOW: self._do_stow,
        }.get(self.state)
        if handler:
            handler()

    def _do_idle(self) -> None:
        if not bool(self.get_parameter("auto_start").value):
            return
        if self.row is None or self.book is None:
            return
        if not self._plan():
            self._enter(State.FAILED)
            return
        self._command(self.pre_solution, self.move_time)
        self._enter(State.PREPARE)

    def _do_prepare(self) -> None:
        # The torso is slow (0.035 m/s) and may have further to travel than the arm.
        if self._elapsed() >= self.move_time + self.settle:
            self._send(self.pub_gripper, ["gripper_left_finger_joint"],
                       [GRIPPER_OPEN], self.gripper_time)
            self._enter(State.OPEN)

    def _do_open(self) -> None:
        if self._elapsed() >= self.gripper_time + 0.5:
            self._command(self.grasp_solution, self.move_time)
            self._enter(State.ADVANCE)

    def _do_advance(self) -> None:
        if self._elapsed() >= self.move_time + self.settle:
            self._send(self.pub_gripper, ["gripper_left_finger_joint"],
                       [GRIPPER_CLAMP], self.gripper_time)
            self._enter(State.CLAMP)

    def _do_clamp(self) -> None:
        if self._elapsed() >= self.gripper_time + 0.5:
            lifted = self._offset_solution(self.grasp_solution, self.lift)
            if lifted is None:
                self.get_logger().warn("no IK for the lift; withdrawing without it")
                self._enter(State.WITHDRAW)
                return
            self._command(lifted, 2.0)
            self._enter(State.LIFT)

    def _do_lift(self) -> None:
        if self._elapsed() >= 2.0 + self.settle:
            self._enter(State.WITHDRAW)

    def _do_withdraw(self) -> None:
        if self._elapsed() < 0.2:
            # Straight back out along the shelf normal, so the book clears the shelf
            # before any joint starts re-orienting.
            self._command(self.pre_solution, self.move_time)
            return
        if self._elapsed() >= self.move_time + self.settle:
            self._send(self.pub_arm, ARM_JOINTS, TUCK_POSE, self.move_time)
            self._enter(State.STOW)

    def _do_stow(self) -> None:
        if self._elapsed() >= self.move_time + self.settle:
            self.get_logger().info("book grasped and stowed")
            self._enter(State.DONE)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspNode()
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
