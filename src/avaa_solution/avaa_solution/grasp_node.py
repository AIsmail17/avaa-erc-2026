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

from collections import deque

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
# Measured from TF, not from the span model: at a commanded 0.000 the joint settles
# at 0.0026 with the fingertips 30.4 mm apart, and the book is 30.0 mm thick. The
# jaws were closing around the book with 0.2 mm to spare on each side and gripping
# nothing, which is why a perfectly aimed grasp -- gripper arrived 8 mm from plan,
# jaws centred on the spine and 20 mm inside the front face -- still lifted nothing.
#
# The joint goes to -0.001, its lower limit, which the span curve puts at about
# 27.7 mm: 2.3 mm narrower than the book, so it actually squeezes.
GRIPPER_CLAMP = -0.001

# Row heights as gripper z in base_link, rows 1..4. base_link sits 0.186 m above the
# floor, so these are the world shelf heights less that offset.
DEFAULT_ROW_HEIGHTS = [1.391, 1.061, 0.731, 0.401]

# The torso settles 2.5-3 cm short of whatever it is commanded, repeatably and in both
# directions of travel (see MANIPULATION.md). Command past the target to land on it.
# Measured, not assumed: commanded 0.15/0.20/0.25/0.30 all settle at exactly the
# commanded height, error 0.0000 m. This was 0.028 to compensate an undershoot seen
# earlier, which turns out to have been orphaned processes holding the simulation at
# a third of real time so trajectories never finished. With those cleaned up the
# torso tracks exactly, and the compensation was placing the gripper 28 mm above the
# book -- enough on its own to close the fingers over the top corner of a 30 mm book.
TORSO_BIAS = 0.0

# What the joints can actually manage, used to give each trajectory a duration it can be
# followed within. A joint_trajectory_controller aborts a trajectory it cannot keep up
# with, and an aborted trajectory looks exactly like a command that was never sent: the
# arm does not move at all, and nothing says why. Measured runs sat still at 394 mm and
# at 1066 mm from their targets until the state timed out.
#
# Deliberately conservative. Overestimating the speed asks for a trajectory that gets
# rejected; underestimating only makes the move take longer than it needs to.
ARM_SPEED = 0.25          # rad/s
TORSO_SPEED = 0.035       # m/s, as documented for this lift

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
    STAGE = "staging"
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
        # How far in front of the book face the pre-grasp sits. At 0.12 the hand is
        # already in the shelf opening, and the arm was jamming there rather than at
        # the grasp: stuck 86 mm out with arm_left_3 held against the shelf. Standing
        # off further keeps the whole hand clear until the advance, which is a short
        # straight push along the shelf normal and the one motion that should be
        # inside the opening.
        self.declare_parameter("standoff_m", 0.25)
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
        # How far past the book face the GRASPING FRAME is driven, which is not where
        # the jaws are: they sit 29.7 mm behind that frame, measured from TF. At 0.05
        # a perfect arrival put the jaws 20 mm inside the face, and a run that arrived
        # 29 mm short in depth -- comfortably inside tolerance, and dead on sideways
        # and in height -- closed them 9 mm in FRONT of the book.
        #
        # 0.11 puts the jaws at the middle of a 160 mm deep book when the arm arrives,
        # and still 51 mm inside the face when it is 29 mm short.
        self.declare_parameter("grasp_depth_m", 0.11)
        self.declare_parameter("lift_m", 0.03)
        # Measured: this arm needs about three times the trajectory duration it is
        # given. States wait on arrival rather than on this, so it is a pace, not
        # a deadline.
        self.declare_parameter("move_time_sec", 6.0)
        self.declare_parameter("gripper_time_sec", 2.0)
        self.declare_parameter("settle_sec", 1.5)
        # Long enough to outlast a full-reach move. A 40 s limit was aborting
        # states while the arm was still travelling toward them.
        self.declare_parameter("state_timeout_sec", 120.0)
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
        self.grasp_target = None
        self.pre_target = None
        self.stage_solution = None
        self.stage_target = None
        # Held close to the body: far enough forward to be a real posture, nowhere near
        # the shelf face.
        self.stage_x = 0.45
        # Outer-loop correction for a posture the arm cannot quite hold. Reaching into
        # the shelf, arm_left_4 settles against its lower stop 0.203 rad short of the
        # commanded value while every other joint sits exactly on target, which leaves
        # the gripper a steady 95 mm from the plan. The offset is repeatable, so it can
        # be aimed off: re-solve for a target displaced by the miss and command that.
        self.corrections = 0
        self.max_corrections = 3
        self.settled_error = None
        self.settled_since = 0.0
        # How close the gripper must actually get before the fingers are allowed to
        # close, and how long to keep trying before calling it a failure.
        #
        # The timeout is generous on purpose. tools/arm_probe.py commanded three poses
        # with time_from_start of 4 s: each was reached exactly, and each took between
        # 12 and 16 s to get there. Every timing in this controller had been written as
        # though 4 s meant 4 s, so each state moved on while the arm was still in
        # transit, and the fingers closed somewhere along the way.
        # Per axis, because the axes are not remotely equivalent. Sideways is the tight
        # one: the jaws open to 60.5 mm around a 30 mm book, so more than about 15 mm off
        # centre and a finger meets the front of the book instead of passing it. Depth
        # and height are forgiving, the book being 160 mm deep and 250 mm tall. A single
        # spherical tolerance has to be set to the tightest axis, and a grasp that was
        # 17 mm out almost entirely in the forgiving directions never got to close.
        self.arrival_tol_lateral = 0.012
        # Tightened from 0.040: the jaw offset means depth error eats into the margin
        # twice over, once for the offset and once for the miss.
        self.arrival_tol_depth = 0.030
        self.arrival_tol_height = 0.040
        # The pre-grasp point only has to be close enough to advance from in a straight
        # line, so it does not need the tolerance the grasp itself does.
        self.pre_tol = 0.03
        # Whether the current state has managed to get its command out. Re-sending one
        # that did arrive is actively harmful: each new trajectory restarts the motion
        # from wherever the arm currently is, so repeating every 6 s left it creeping
        # 5 mm in 55 s. Send once, when something is listening.
        self.command_sent = False
        # Measured end to end: the arm sat near 590 mm for 25 s and then converged
        # to 37 mm by 38 s. A 30 s limit cut it off just before it arrived.
        self.arrival_timeout = 75.0
        # Roughly a second of frames at 15 Hz. Long enough to bury an outlier,
        # short enough that the target is still current when the arm commits.
        self.book_points = deque(maxlen=15)
        self.book_points_min = 8
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
        """Hold the median of recent sightings, not the latest one.

        Measured against Gazebo at grasping range, the published point is accurate
        sideways -- 3 mm of bias, 7 mm of spread, worst case 14 mm, all inside the 15 mm
        of clearance the jaws leave around a 30 mm book. In depth it is not: 76 mm of
        bias with 149 mm of spread over the same samples.

        Planning from whichever sample happened to arrive last therefore puts the hand
        anywhere within a hand-width of the shelf. Too shallow and the fingers close in
        front of the spine; too deep and they drive into it, which is what shoved a book
        0.193 m along the shelf and left it flat while the controller reported success.

        A median over a second of frames rejects the outliers that a mean would carry.
        """
        self.book_points.append([msg.point.x, msg.point.y, msg.point.z])
        if len(self.book_points) < self.book_points_min:
            return
        self.book = np.median(np.array(self.book_points), axis=0)

    def _on_joints(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = position

    # ------------------------------------------------------------------ helpers

    def _enter(self, state: State) -> None:
        if state is not self.state:
            self.command_sent = False
            self.get_logger().info(f"{self.state.value} -> {state.value}")
            self.state = state
            self.state_since = self._now()

    def _elapsed(self) -> float:
        return self._now() - self.state_since

    def _send_path(self, pub, names, waypoints, seconds) -> bool:
        """Send a trajectory through several waypoints rather than one endpoint.

        The controller interpolates in joint space. Between two postures that are 0.36 m
        apart in a straight line, that interpolation bows the arm sideways, and at the
        shelf the bow goes through it: the advance jammed with arm_left_3 held 0.159 rad
        off and the gripper 133 mm short, having reached the pre-grasp point cleanly.

        Handing the controller the intermediate postures keeps the gripper on the straight
        line it is supposed to follow, which for a reach into a shelf is the only path
        that fits.
        """
        if pub.get_subscription_count() == 0:
            self.get_logger().warn(
                f"nothing is listening on {pub.topic_name} yet; holding the path",
                throttle_duration_sec=2.0)
            return False
        traj = JointTrajectory()
        traj.joint_names = list(names)
        traj.points = []
        step = seconds / max(1, len(waypoints))
        for index, values in enumerate(waypoints, start=1):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in values]
            moment = step * index
            point.time_from_start = Duration(
                sec=int(moment), nanosec=int((moment % 1.0) * 1e9))
            traj.points.append(point)
        pub.publish(traj)
        return True

    def _straight_line(self, start_solution, start_point, end_point, steps=6):
        """Joint waypoints tracing a straight line in space between two points."""
        face_x = float(self.book[0]) if self.book is not None else None
        waypoints = []
        seed = list(start_solution)
        start_point = np.asarray(start_point, dtype=float)
        end_point = np.asarray(end_point, dtype=float)
        for index in range(1, steps + 1):
            fraction = index / float(steps)
            point = start_point + fraction * (end_point - start_point)
            solution = self._solve(point, seed=seed, face_x=face_x, near=seed)
            if solution is None:
                self.get_logger().warn(
                    "no IK %.0f%% of the way along the reach; pushing straight instead"
                    % (100.0 * fraction))
                return None
            waypoints.append(solution)
            seed = solution
        return waypoints

    def _send(self, pub, names, values, seconds) -> bool:
        traj = JointTrajectory()
        traj.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in values]
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
        traj.points = [point]
        if pub.get_subscription_count() == 0:
            # Publishing now would go nowhere. A trajectory sent before the controller
            # has matched the subscription is dropped in silence and the arm simply never
            # moves: seen in tools/arm_probe.py, where the first of three commanded poses
            # was ignored for 16 s while the other two were reached exactly, and in a
            # grasp that sat 1066 mm from its pre-grasp point without twitching.
            self.get_logger().warn(
                f"nothing is listening on {pub.topic_name} yet; holding the command",
                throttle_duration_sec=2.0)
            return False
        pub.publish(traj)
        return True

    def row_height(self, row: int) -> Optional[float]:
        return row_to_height(row, self.row_heights, self.rows_top_down)

    def _solve(self, target: np.ndarray, seed: Optional[List[float]] = None,
               face_x: Optional[float] = None, near=None):
        """Every solve in this node holds the same wrist, and keeps the elbow outside.

        The lift and the withdraw run through here too, and they matter as much: a
        wrist left free to rotate while the fingers are clamped twists the book back
        out of the shelf instead of drawing it clear.

        Reaching the right point with the right wrist still leaves four degrees of
        freedom, and the solver spent them on postures that put the arm through the
        shelf. Commanded at the shelf, one such solution ended 5.733 rad from its target
        with arm_left_1 pushed backwards; the identical command in open floor settled
        within 0.998 and still closing. Nothing in the logs said "blocked" -- the joints
        simply never arrived.
        """
        prefer = None if face_x is None else self._clearance_cost(face_x, near=near)
        return self.chain.ik(target, seed=seed, approach=GRASP_APPROACH,
                             closing=GRASP_CLOSING, prefer=prefer)

    def _clearance_cost(self, face_x: float, near=None):
        """Score postures: out of the shelf, near where the arm already is, off the stops.

        Scoring on intrusion alone is not enough, and scoring on it alone was worse than
        not scoring at all. Every candidate for a reachable book scores zero intrusion --
        no joint origin goes past the shelf face, only the hand does -- so the minimum was
        picked arbitrarily from among ties, and what came back was whichever random
        restart happened to land first: shoulder wound round to 4.5 of a possible 4.7 rad
        and the torso pinned at its 0.35 limit. The arm then drifted away from it for a
        minute and a half.

        So the cost orders the ties too. Travel keeps the posture near the one the arm is
        already in, which also makes the move short; the limit term keeps it off the stops,
        where a joint has no room to correct and the controller struggles to hold it.
        """
        # Measure travel from a given posture when one is supplied, otherwise from where
        # the arm is now. The grasp is solved against the pre-grasp posture: the two are
        # 230 mm apart along one axis and ought to be neighbours in joint space, but the
        # solver does not know that. Left to choose freely it returned a posture unrelated
        # to the pre-grasp one, and the advance became a re-orientation instead of a push.
        # The jaws straddled the book correctly at the pre-grasp -- tips at y 0.129 and
        # 0.189 around a book spanning 0.144 to 0.174 -- and then closed on nothing.
        if near is not None:
            current = list(near)
            weight = 2.0
        else:
            try:
                current = [self.joints[name]
                           for name in ["torso_lift_joint"] + ARM_JOINTS]
            except KeyError:
                current = None
            weight = 0.10
        limits = self.chain.limits

        def cost(values: List[float]) -> float:
            origins = self.chain.joint_origins(values)
            # The last two frames are the wrist and the gripper; they have to go in.
            intrusion = sum(max(0.0, float(p[0]) - face_x) for p in origins[:-2])
            travel = (0.0 if current is None
                      else sum(abs(a - b) for a, b in zip(values, current)))
            # Weighted heavily and measured generously, because a joint near its stop
            # is the failure that has cost the most here. arm_left_4 was chosen 0.203 rad
            # from its lower limit, sagged onto the stop under the weight of an extended
            # arm, and held the gripper 95 mm from the book while every other joint sat
            # exactly on target.
            crowding = sum(max(0.0, 0.45 - min(v - lo, hi - v)) ** 2
                           for v, (lo, hi) in zip(values, limits))
            return 10.0 * intrusion + weight * travel + 20.0 * crowding

        return cost

    def _travel_time(self, solution, torso: float) -> float:
        """How long this move needs, from how far each joint has to travel.

        Asking for less time than the joints can deliver does not make them faster; it
        makes the controller give up on the trajectory.
        """
        try:
            arm_now = [self.joints[name] for name in ARM_JOINTS]
            torso_now = self.joints["torso_lift_joint"]
        except KeyError:
            return 0.0     # nothing known yet; leave the caller default alone
        arm_time = max(abs(a - b) for a, b in zip(arm_now, solution[1:])) / ARM_SPEED
        torso_time = abs(torso_now - torso) / TORSO_SPEED
        return max(arm_time, torso_time)

    def _tracking(self, solution) -> str:
        """Per-joint gap between a commanded solution and where the joints actually are.

        Distinguishes the two things a stalled position error can mean. If the joints are
        at the commanded values then the arm did what it was told and the target is wrong;
        if they are not, something is stopping it or the command never took.
        """
        if solution is None:
            return ""
        names = ["torso_lift_joint"] + ARM_JOINTS
        try:
            actual = [self.joints[name] for name in names]
        except KeyError:
            return " [joints unknown]"
        gaps = [a - c for a, c in zip(actual, solution)]
        worst = max(range(len(gaps)), key=lambda i: abs(gaps[i]))
        return (" [sent=%s worst joint %s off %+.3f, sum |gap| %.3f]"
                % (self.command_sent, names[worst], gaps[worst],
                   sum(abs(g) for g in gaps)))

    def _ensure_commanded(self, solution) -> None:
        """Get the command out if the first attempt found nobody listening."""
        if not self.command_sent:
            self._command(solution, self.move_time)

    def _reach_offset(self, target) -> Optional[np.ndarray]:
        """Signed miss along each axis, in base_link, rather than one distance."""
        achieved = self._achieved()
        if achieved is None or target is None:
            return None
        return np.asarray(achieved) - np.asarray(target)

    def _within_tolerance(self, offset) -> bool:
        return (abs(float(offset[0])) <= self.arrival_tol_depth
                and abs(float(offset[1])) <= self.arrival_tol_lateral
                and abs(float(offset[2])) <= self.arrival_tol_height)

    def _achieved(self) -> Optional[np.ndarray]:
        """Where the gripper actually is, from the real joint positions."""
        try:
            values = [self.joints[name]
                      for name in ["torso_lift_joint"] + ARM_JOINTS]
        except KeyError:
            return None
        return self.chain.position(values)

    def _unused_correct_for_sag(self) -> bool:
        """Kept as a record of something that did not work.

        The idea was to aim off by the measured shortfall: the gripper is short by some
        vector, so ask for a target displaced by that vector. It is a reasonable move
        against a fixed offset, and this is not one. Reaching further needs more
        extension, extension is what the arm was failing to hold, and the miss went from
        95 mm to 168 mm with the elbow gap growing from 0.20 to 0.59 rad. Left here so
        the next person does not spend an afternoon rediscovering it.
        """
        achieved = self._achieved()
        if achieved is None or self.grasp_target is None:
            return False
        miss = self.grasp_target - achieved
        aim = self.grasp_target + miss
        solution = self._solve(aim, seed=self.grasp_solution,
                               face_x=float(self.book[0]) if self.book is not None
                               else None)
        if solution is None:
            self.get_logger().warn(
                "no IK for the corrected target %s" % np.round(aim, 3).tolist())
            return False
        self.corrections += 1
        self.get_logger().info(
            "arm is %.0f mm short and holding; aiming %.0f mm past the target "
            "(correction %d of %d)"
            % (np.linalg.norm(miss) * 1000, np.linalg.norm(miss) * 1000,
               self.corrections, self.max_corrections))
        self.grasp_solution = solution
        self._command(solution, self.move_time)
        return True

    def _reach_error(self, target) -> Optional[float]:
        """How far the gripper actually is from a planned point.

        Reads the real joint positions rather than assuming the trajectory was followed.
        Measured on this arm, a trajectory asking for 4 s takes 12 to 16 s to finish, so
        assuming it was followed is assuming a great deal.
        """
        if target is None:
            return None
        try:
            values = [self.joints[name]
                      for name in ["torso_lift_joint"] + ARM_JOINTS]
        except KeyError:
            return None      # not all joints reported yet
        return float(np.linalg.norm(self.chain.position(values) - target))

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

        pre_solution = self._solve(pre, face_x=face_x)
        if pre_solution is None:
            self.get_logger().error(
                f"no IK for pre-grasp {np.round(pre, 3).tolist()}")
            return False
        # Seed the grasp solve from the pre-grasp so the two are near neighbours and the
        # advance is a short, straight-ish motion rather than a re-orientation.
        # Solved as a neighbour of the pre-grasp posture, so the advance is the short
        # straight push it is meant to be rather than a fresh choice of arm shape.
        grasp_solution = self._solve(grasp, seed=pre_solution, face_x=face_x,
                                     near=pre_solution)
        if grasp_solution is None:
            self.get_logger().error(f"no IK for grasp {np.round(grasp, 3).tolist()}")
            return False

        # Rise to the row before reaching for it.
        #
        # The controller interpolates in joint space, so a single command from the tucked
        # pose to a point in the shelf sweeps the arm along whatever path the joints
        # happen to take between them. Measured with the link contact sensors, that path
        # put arm_left_6 inside the shelf at z=0.44, below the lowest shelf surface at
        # 0.587, while the book it was reaching for was at z=1.247. Four links ended up
        # against base_link_shelf_collision and the arm stopped moving.
        #
        # Staging near the body at the target height turns one long diagonal sweep into a
        # lift and then a reach, neither of which crosses the shelf.
        stage = np.array([self.stage_x, y, height])
        stage_solution = self._solve(stage, face_x=face_x)
        if stage_solution is None:
            self.get_logger().warn(
                f"no IK for the staging point {np.round(stage, 3).tolist()}; "
                "reaching directly")

        self.stage_solution = stage_solution
        self.stage_target = stage
        self.pre_solution = pre_solution
        self.grasp_solution = grasp_solution
        self.grasp_target = grasp
        self.pre_target = pre
        self.get_logger().info(
            f"row {self.row} at z={height:.3f}; book at "
            f"x={face_x:.3f} y={y:.3f}; torso {grasp_solution[0]:.3f}"
        )
        return True

    def _command(self, solution: List[float], seconds: float) -> bool:
        """Send a full-chain solution, compensating the torso's known undershoot."""
        torso = min(0.35, max(0.0, solution[0] + TORSO_BIAS))
        seconds = max(seconds, self._travel_time(solution, torso))
        self.get_logger().info(
            "commanding over %.1fs: torso %.3f, arm %s"
            % (seconds, torso, [round(v, 2) for v in solution[1:]]))
        sent_torso = self._send(self.pub_torso, ["torso_lift_joint"], [torso], seconds)
        sent_arm = self._send(self.pub_arm, ARM_JOINTS, solution[1:], seconds)
        self.command_sent = sent_torso and sent_arm
        return self.command_sent

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
            State.STAGE: self._do_stage,
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
        # _enter clears command_sent, so it has to come first or the command is wiped
        # and sent a second time by the state that is waiting on it.
        if self.stage_solution is not None:
            self._enter(State.STAGE)
            self._command(self.stage_solution, self.move_time)
        else:
            self._enter(State.PREPARE)
            self._command(self.pre_solution, self.move_time)

    def _do_stage(self) -> None:
        """Get to the target height near the body before reaching into the shelf."""
        if self.stage_solution is None:
            self._enter(State.PREPARE)
            self._command(self.pre_solution, self.move_time)
            return
        error = self._reach_error(self.stage_target)
        if error is not None and error <= self.pre_tol:
            self.get_logger().info(
                f"staged at the row, {error * 1000:.0f} mm out; reaching in")
            self._enter(State.PREPARE)
            self._command(self.pre_solution, self.move_time)
            return
        if self._elapsed() >= self.arrival_timeout:
            self.get_logger().error(
                "arm stalled on the way to the staging point; it is probably against "
                "the shelf")
            self._enter(State.FAILED)
            return
        self._ensure_commanded(self.stage_solution)
        if error is not None:
            self.get_logger().info(
                f"staging: {error * 1000:.0f} mm to go", throttle_duration_sec=3.0)

    def _do_prepare(self) -> None:
        """Reach the pre-grasp point, and wait until the arm is actually there.

        The torso is slow (0.035 m/s) and may have further to travel than the arm, and
        the arm itself takes about three times the trajectory duration it is given. Moving
        on after move_time + settle meant the hand was still swinging out when the fingers
        were told to open and again when the advance was commanded, so the advance started
        from somewhere unplanned. In one run the distance to the grasp point was still
        growing when the advance began: 389, 398, 408, 423, 441 mm.
        """
        error = self._reach_error(self.pre_target)
        if error is None:
            if self._elapsed() >= self.move_time + self.settle:
                self._open_fingers()
            return
        if error <= self.pre_tol:
            self.get_logger().info(
                f"at the pre-grasp point, {error * 1000:.0f} mm out")
            self._open_fingers()
            return
        if self._elapsed() >= self.arrival_timeout:
            self.get_logger().error(
                f"arm stalled {error * 1000:.0f} mm from the pre-grasp point after "
                f"{self._elapsed():.1f}s")
            self._enter(State.FAILED)
            return
        self._ensure_commanded(self.pre_solution)
        self.get_logger().info(
            f"preparing: {error * 1000:.0f} mm to go{self._tracking(self.pre_solution)}",
            throttle_duration_sec=3.0)

    def _open_fingers(self) -> None:
        self._send(self.pub_gripper, ["gripper_left_finger_joint"],
                   [GRIPPER_OPEN], self.gripper_time)
        self._enter(State.OPEN)

    def _do_open(self) -> None:
        if self._elapsed() < self.gripper_time + 0.5:
            return
        # The reach into the shelf is the one motion that has to stay on its line.
        waypoints = self._straight_line(
            self.pre_solution, self.pre_target, self.grasp_target)
        self._enter(State.ADVANCE)
        if waypoints is None:
            self._command(self.grasp_solution, self.move_time)
            return
        self.grasp_solution = waypoints[-1]
        seconds = max(self.move_time, self._travel_time(
            waypoints[-1], min(0.35, max(0.0, waypoints[-1][0] + TORSO_BIAS))))
        torso_path = [[min(0.35, max(0.0, w[0] + TORSO_BIAS))] for w in waypoints]
        sent_torso = self._send_path(
            self.pub_torso, ["torso_lift_joint"], torso_path, seconds)
        sent_arm = self._send_path(
            self.pub_arm, ARM_JOINTS, [w[1:] for w in waypoints], seconds)
        self.command_sent = sent_torso and sent_arm
        self.get_logger().info(
            "advancing along %d waypoints over %.1fs" % (len(waypoints), seconds))

    def _do_advance(self) -> None:
        """Close the fingers when the gripper has arrived, not when a timer says so.

        This used to clamp after move_time + settle regardless of where the arm actually
        was. Measured against the plan, it was still 60 mm short and still moving when
        that timer expired: the grasping frame reached x=0.805 against a commanded 0.869.
        The failures looked like clean misses because they were clean misses, of a target
        the arm had not reached yet.

        The jaws sit 29.7 mm behind gripper_left_grasping_link when open, measured from
        TF, so where they close is not the point the IK aims for. That offset is small
        next to the errors above and is folded into grasp_depth_m rather than modelled.

        Waiting on the measured error also removes the guesswork from move_time. A move
        that needs longer simply takes longer, and one that never arrives is reported
        rather than clamped anyway.
        """
        error = self._reach_error(self.grasp_target)
        offset = self._reach_offset(self.grasp_target)
        if error is None:
            if self._elapsed() >= self.move_time + self.settle:
                self.get_logger().warn(
                    "cannot measure the gripper position; clamping on the timer")
                self._clamp()
            return

        if offset is not None and self._within_tolerance(offset):
            self.get_logger().info(
                "gripper arrived: %+.0f mm depth, %+.0f mm sideways, %+.0f mm height"
                % (offset[0] * 1000, offset[1] * 1000, offset[2] * 1000))
            self._clamp()
            return

        if self._elapsed() >= self.arrival_timeout:
            self.get_logger().error(
                f"gripper stalled {error * 1000:.0f} mm from the grasp point after "
                f"{self._elapsed():.1f}s; not closing on empty air")
            self._enter(State.FAILED)
            return

        # Aiming off by the shortfall was tried here and made things worse: asking for a
        # target 95 mm further in demands more extension, which is what the arm was
        # already failing to hold, and the miss grew to 168 mm with the elbow gap
        # tripling. The shortfall is not an offset to cancel, it is a posture that cannot
        # be held, so it is dealt with when the posture is chosen rather than here.

        self._ensure_commanded(self.grasp_solution)
        self.get_logger().info(
            f"advancing: {error * 1000:.0f} mm to go"
            f"{self._tracking(self.grasp_solution)}",
            throttle_duration_sec=2.0)

    def _clamp(self) -> None:
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
