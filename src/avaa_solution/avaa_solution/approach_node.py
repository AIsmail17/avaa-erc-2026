"""Fine approach — close the last stretch to the target column.

Nav2 gets the robot to within about 0.3 m of a goal and then stops dead, so it is used for
gross navigation only (see config/nav2_params.yaml). This node closes the remainder using
the camera and the front LiDAR, which is what mobile manipulation needs anyway: the arm has
to be placed relative to the book, not to an odometry coordinate that was only ever an
estimate of where the book is.

Sequence:

    CENTRE   rotate until the target column sits in the middle of the image
    APPROACH drive forward, holding the column centred, until the shelf face is at standoff
    SQUARE   rotate to sit perpendicular to the shelf face, fitted from the LiDAR
    DONE

Lateral motion is never commanded. Measured on this base, a pure vy command yaws the robot
by roughly the same magnitude as it strafes, so the approach is built from the two motions
that behave: forward drive and rotation. Rotating to face the column and then driving
straight at it arrives laterally centred by construction.
"""

import math
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32, Int32, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

BASE_FRAME = "base_footprint"
CAMERA_FRAME = "head_front_camera_depth_optical_frame"

# base_link sits this far above base_footprint. Row heights are quoted in base_link.
BASE_LINK_Z = 0.186

# head_2_joint: negative looks down, roughly one-for-one in radians. Limits from the URDF.
HEAD_TILT_MIN = -1.047   # about 60 degrees down
HEAD_TILT_MAX = 0.349    # about 20 degrees up

# Gripper z in base_link for rows 1..4, top shelf first.
DEFAULT_ROW_HEIGHTS = [1.391, 1.061, 0.731, 0.401]

# Scan returns inside this radius of base_footprint are the robot itself, not obstacles.
# The base is 0.717 x 0.497 m, so its circumscribed radius is 0.437 m; this sits just
# outside that.
SELF_FILTER_RADIUS = 0.45

TOPIC_TARGET_COLUMN_X = "/avaa/perception/target_column_x"
TOPIC_TARGET_ROW = "/avaa/perception/target_row"
TOPIC_BOOK_POINT = "/avaa/perception/target_book_point"
TOPIC_ARM_LEFT = "/arm_left_controller/joint_trajectory"
TOPIC_ARM_RIGHT = "/arm_right_controller/joint_trajectory"
TOPIC_SCAN = "/scan_front_raw"
TOPIC_STATE = "/avaa/approach/state"
TOPIC_CMD = "/cmd_vel"

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


# Arm posture for driving, measured rather than guessed (see try_tuck.py).
#
# The arms spawn with every joint at zero, which for this arm is fully extended: the
# gripper reaches 0.838 m forward, 0.478 m beyond the front of the base. Driving at the
# shelf in that posture wedges the arm against it -- observed as six simultaneous contacts
# between both grippers, both arm_6 links and erc_shelf, with the base unable to advance
# while the LiDAR still read 0.94 m of clear space ahead. Each contact event costs half a
# point.
#
# This pose measured 0.319 m forward and 0.174 m lateral, both inside the base footprint
# (0.36 m half-length, 0.249 m half-width), with no contacts. Joint 2 does most of the
# work; the elbow pulls the forearm in laterally, and joint 1 finishes the job.
TUCK_POSE = [-0.5, -2.4, 0.0, -2.4, 0.0, 0.0, 0.0]


class State(Enum):
    WAITING = "waiting"
    TUCK = "tucking"
    SEARCH = "searching"
    CENTRE = "centring"
    ACQUIRE = "acquiring"
    APPROACH = "approaching"
    SQUARE = "squaring"
    DONE = "done"
    FAILED = "failed"


class ApproachNode(Node):
    def __init__(self) -> None:
        super().__init__("avaa_approach")

        # Distance to stop at, measured from base_footprint -- not from the laser, which
        # sits 0.275 m further forward. The base is 0.717 m long, so 0.75 m from the
        # origin leaves about 0.39 m of clearance ahead of the bumper.
        #
        # Provisional. The right value is whatever puts the books inside the arm's working
        # envelope, which cannot be settled until grasping exists.
        self.declare_parameter("standoff_m", 0.75)
        self.declare_parameter("centre_tolerance_px", 12.0)
        self.declare_parameter("standoff_tolerance_m", 0.05)
        self.declare_parameter("square_tolerance_rad", 0.05)
        self.declare_parameter("max_yaw_rate", 0.45)
        self.declare_parameter("max_forward", 0.22)
        self.declare_parameter("max_lateral", 0.10)
        # Refuse to drive closer than this whatever the standoff says, so a bad reading
        # cannot push the base into the shelf. A collision costs 0.5 points each time.
        # Also measured from base_footprint: the bumper is 0.36 m out, so 0.55 m leaves
        # roughly 0.19 m of margin.
        self.declare_parameter("min_safe_range_m", 0.55)
        self.declare_parameter("state_timeout_sec", 45.0)
        self.declare_parameter("search_rate", 0.35)
        self.declare_parameter("row_heights", DEFAULT_ROW_HEIGHTS)
        # Where to pause and confirm the book before committing to the final drive.
        #
        # Far enough back that the whole shelf column is still comfortably in frame, so
        # the book can be found and centred without a race. Driving straight to grasping
        # range instead means handing over from marker-steering to book-steering while
        # moving, and if the book is not acquired in time the robot arrives beside its
        # column with nothing recognisable in view.
        self.declare_parameter("acquire_range_m", 1.50)
        self.declare_parameter("acquire_tolerance_px", 25.0)
        # Searching gets its own budget: a full turn at 0.35 rad/s is about 18 s of
        # simulation time, which at a real-time factor near 0.5 is well over half a minute
        # of wall clock. The ordinary state timeout would abort mid-sweep.
        self.declare_parameter("search_timeout_sec", 150.0)
        self.declare_parameter("image_width_px", 640)
        self.declare_parameter("tuck_time_sec", 5.0)

        self.standoff = float(self.get_parameter("standoff_m").value)
        self.centre_tol = float(self.get_parameter("centre_tolerance_px").value)
        self.standoff_tol = float(self.get_parameter("standoff_tolerance_m").value)
        self.square_tol = float(self.get_parameter("square_tolerance_rad").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rate").value)
        self.max_fwd = float(self.get_parameter("max_forward").value)
        self.max_lateral = float(self.get_parameter("max_lateral").value)
        self.min_safe = float(self.get_parameter("min_safe_range_m").value)
        self.timeout = float(self.get_parameter("state_timeout_sec").value)
        self.search_rate = float(self.get_parameter("search_rate").value)
        self.search_timeout = float(self.get_parameter("search_timeout_sec").value)
        self.image_width = int(self.get_parameter("image_width_px").value)
        self.tuck_time = float(self.get_parameter("tuck_time_sec").value)

        self.column_cx: Optional[float] = None
        self.column_cx_at: Optional[float] = None
        self.scan: Optional[LaserScan] = None
        self.state = State.WAITING
        self.state_since = self._now()

        # Servo on the target column's image position, not on a column index. The index
        # perception publishes is frame-relative -- it counts the columns currently in
        # view, so it changes as markers enter and leave the frame, and anything treating
        # it as the column's identity tracks a different column each frame.
        # Scan points must be transformed into the robot frame, not read as if the laser
        # were aligned with it -- it is mounted rotated. See _scan_points_base.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # The row has to be identified before closing in, not after. Resolving it needs
        # all four books of the column in frame, and at grasping range the column no
        # longer fits, so driving first and identifying later never succeeds.
        self.row_seen = False
        self.target_row: Optional[int] = None
        self.head_tilt: Optional[float] = None
        self.acquire_range = float(self.get_parameter("acquire_range_m").value)
        self.acquire_tol = float(self.get_parameter("acquire_tolerance_px").value)
        # Set once the book has actually been located in 3D, which is the signal that it
        # is genuinely visible rather than merely expected to be.
        self.book_point_at: Optional[float] = None
        self.approach_target = self.acquire_range
        self.create_subscription(
            PointStamped, TOPIC_BOOK_POINT, self._on_book_point, 10)
        self.row_heights = list(
            self.get_parameter("row_heights").value or DEFAULT_ROW_HEIGHTS)
        self.pub_head = self.create_publisher(
            JointTrajectory, "/head_controller/joint_trajectory", 10)
        self.create_subscription(Int32, TOPIC_TARGET_ROW, self._on_row, 10)
        self.create_subscription(Float32, TOPIC_TARGET_COLUMN_X, self._on_column_x, 10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self._on_scan, SENSOR_QOS)
        self.pub_cmd = self.create_publisher(Twist, TOPIC_CMD, 10)
        self.pub_state = self.create_publisher(String, TOPIC_STATE, 10)
        self.pub_arm_left = self.create_publisher(JointTrajectory, TOPIC_ARM_LEFT, 10)
        self.pub_arm_right = self.create_publisher(JointTrajectory, TOPIC_ARM_RIGHT, 10)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"approach up — standoff {self.standoff:.2f} m, "
            f"safety floor {self.min_safe:.2f} m"
        )

    # ------------------------------------------------------------------ inputs

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_column_x(self, msg: Float32) -> None:
        self.column_cx = float(msg.data)
        self.column_cx_at = self._now()

    def _column_cx_fresh(self, max_age: float = 1.5) -> Optional[float]:
        """The target column's image x, or None if perception has gone quiet.

        Acting on a stale bearing steers toward where the column used to be, so the
        controller stops rather than guessing.
        """
        if self.column_cx is None or self.column_cx_at is None:
            return None
        if (self._now() - self.column_cx_at) > max_age:
            return None
        return self.column_cx

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    # ------------------------------------------------------------------ geometry

    def _scan_points_base(self) -> List[Tuple[float, float]]:
        """All scan returns as (x, y) in base_footprint.

        The scan frame is NOT aligned with the robot. The front laser is mounted at
        roll -180 deg, yaw -45 deg relative to base_footprint, so scan angle zero points
        45 degrees off to the side and the roll mirrors the direction of increasing angle.
        Treating scan angles as robot-relative bearings therefore measures a cone pointing
        somewhere else entirely -- which is why the shelf-squaring fit reported the face
        2.5 degrees off while the robot was actually sitting 35 degrees away from square.
        """
        if self.scan is None:
            return []
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, self.scan.header.frame_id, rclpy.time.Time()
            )
        except Exception:  # noqa: BLE001 - transform may not be available yet
            return []

        q = tf.transform.rotation
        t = tf.transform.translation
        # Rotation matrix from the quaternion; only the rows producing x and y are needed.
        xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
        xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
        wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
        r00, r01, r02 = 1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)
        r10, r11, r12 = 2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)

        points = []
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or not (self.scan.range_min < r < self.scan.range_max):
                continue
            angle = self.scan.angle_min + i * self.scan.angle_increment
            lx, ly, lz = r * math.cos(angle), r * math.sin(angle), 0.0
            bx = r00 * lx + r01 * ly + r02 * lz + t.x
            by = r10 * lx + r11 * ly + r12 * lz + t.y
            # Discard the robot's own body. The laser plane sits at z = 0.209 m and the
            # tucked arm reaches 0.319 m forward, so the stowed arm is unavoidably inside
            # the scan -- it read as an obstacle 0.35 m ahead and tripped the safety stop
            # the instant driving began. Nothing external can be this close without the
            # robot already being in contact, so anything inside the footprint radius is
            # the robot seeing itself.
            if math.hypot(bx, by) < SELF_FILTER_RADIUS:
                continue
            points.append((bx, by))
        return points

    def _forward_points(self, half_angle: float = 0.30) -> List[Tuple[float, float]]:
        """Returns within +/- half_angle of straight ahead, as (x, y) in base_footprint."""
        return [(x, y) for x, y in self._scan_points_base()
                if x > 0.0 and abs(math.atan2(y, x)) <= half_angle]

    def _range_ahead(self) -> Optional[float]:
        points = self._forward_points(half_angle=0.12)
        if not points:
            return None
        # Median, not minimum: a single spurious short return would otherwise stop the
        # approach short of the shelf.
        return float(np.median([x for x, _ in points]))

    def _min_range_ahead(self) -> Optional[float]:
        points = self._forward_points(half_angle=0.35)
        if not points:
            return None
        return float(min(math.hypot(x, y) for x, y in points))

    def _shelf_angle(self) -> Optional[float]:
        """Yaw error against the shelf face: 0 when square on, positive when yawed CCW.

        Fits a line to the forward returns. The shelf front is flat and 5.25 m wide, so
        within a narrow cone it is a clean straight edge. For a robot yawed by theta, a
        surface of constant world x appears in the robot frame with dx/dy = tan(theta), so
        the fitted slope is the yaw error directly.

        Returns None when the fit is not credible, rather than squaring up to whatever
        happens to be in front. Without that guard a robot that has turned away from the
        shelf will happily square itself to the far wall.
        """
        points = self._forward_points(half_angle=0.45)
        if len(points) < 12:
            return None
        xs = np.array([p[0] for p in points])
        ys = np.array([p[1] for p in points])
        # x as a function of y: the face is roughly parallel to the robot's y axis, so
        # this avoids the vertical-line singularity of fitting y = f(x).
        slope, intercept = np.polyfit(ys, xs, 1)

        # A flat face fits tightly. A scattered cloud -- books at different depths, or two
        # surfaces at once -- does not, and squaring to it would be meaningless.
        residual = float(np.std(xs - (slope * ys + intercept)))
        if residual > 0.08:
            return None
        return float(math.atan(slope))

    # ------------------------------------------------------------------ control

    def _enter(self, state: State) -> None:
        if state is not self.state:
            self.get_logger().info(f"{self.state.value} -> {state.value}")
            self.state = state
            self.state_since = self._now()

    def _publish_state(self) -> None:
        self.pub_state.publish(String(data=self.state.value))

    def _elapsed(self) -> float:
        return self._now() - self.state_since

    def _stop(self) -> None:
        self.pub_cmd.publish(Twist())

    def _tick(self) -> None:
        self._publish_state()

        if self.state in (State.DONE, State.FAILED):
            self._stop()
            return

        if self.state is State.WAITING:
            # Stow the arms before anything moves, then go looking for the marker. The
            # robot's start pose is not guaranteed to face the shelves -- in practice it
            # spawns facing a wall -- so searching is part of the task, not a fallback.
            self._send_tuck()
            self._enter(State.TUCK)
            return

        budget = self.search_timeout if self.state is State.SEARCH else self.timeout
        if self._elapsed() > budget:
            self.get_logger().error(f"timed out in {self.state.value}")
            self._stop()
            self._enter(State.FAILED)
            return

        # Safety floor applies in every moving state.
        nearest = self._min_range_ahead()
        if nearest is not None and nearest < self.min_safe and self.state is State.APPROACH:
            self.get_logger().warn(
                f"safety stop: obstacle at {nearest:.2f} m < {self.min_safe:.2f} m"
            )
            self._stop()
            self._enter(State.SQUARE)
            return

        if self.state is State.TUCK:
            self._do_tuck()
        elif self.state is State.SEARCH:
            self._do_search()
        elif self.state is State.ACQUIRE:
            self._do_acquire()
        elif self.state is State.CENTRE:
            self._do_centre()
        elif self.state is State.APPROACH:
            self._do_approach()
        elif self.state is State.SQUARE:
            self._do_square()

    def _send_tuck(self) -> None:
        """Command both arms to the driving posture."""
        for pub, side in ((self.pub_arm_left, "left"), (self.pub_arm_right, "right")):
            traj = JointTrajectory()
            traj.joint_names = [f"arm_{side}_{i}_joint" for i in range(1, 8)]
            point = JointTrajectoryPoint()
            pose = list(TUCK_POSE)
            if side == "right":
                # Mirror the shoulder pan and upper-arm roll.
                pose[0] = -pose[0]
                pose[2] = -pose[2]
            point.positions = [float(v) for v in pose]
            point.time_from_start = Duration(sec=int(self.tuck_time), nanosec=0)
            traj.points = [point]
            pub.publish(traj)
        self.get_logger().info("stowing arms for driving")

    def _do_tuck(self) -> None:
        self._stop()  # no driving until the arms are in
        elapsed = self._now() - self.state_since

        # Repeat only during the first moment, to cover the controller not yet being
        # subscribed. Repeating later is actively harmful: each JointTrajectory replaces
        # the one in progress and restarts its time_from_start, so a trajectory re-sent
        # every two seconds never finishes. That left the arm permanently mid-sweep, and
        # since the tuck path crosses the LiDAR plane the moving arm registered as an
        # obstacle 0.08 m ahead the instant driving began.
        if elapsed < 0.6:
            self._send_tuck()

        # Wait out the full trajectory plus settling before allowing any motion.
        if elapsed >= self.tuck_time + 2.0:
            self._enter(State.SEARCH)

    def _do_search(self) -> None:
        """Rotate on the spot until the target column's marker comes into view.

        The robot spawns facing a wall and the marker digits are randomised per run, so
        the target may be anywhere around it. Rotating in place is the cheapest way to
        cover the full circle without risking a collision, and with the arms stowed the
        base turns cleanly on the spot.
        """
        if self._column_cx_fresh() is not None:
            self._stop()
            self.get_logger().info("target marker found")
            self._enter(State.CENTRE)
            return

        cmd = Twist()
        cmd.angular.z = self.search_rate
        self.pub_cmd.publish(cmd)
        self.get_logger().info(
            f"searching for marker... ({self._elapsed():.0f}s)",
            throttle_duration_sec=5.0,
        )

    def _on_row(self, msg: Int32) -> None:
        self.row_seen = True
        self.target_row = int(msg.data)

    def _aim_head(self) -> None:
        """Tilt the head to keep the target row in frame as the robot closes in.

        Approaching with the head level loses the books out of the bottom of the image
        well before grasping range, which is what stopped the book point being published
        just when the grasp controller needed it.

        head_2_joint is negative for down, essentially one-for-one in radians (measured:
        -0.40 gives 22.9 degrees down, -0.80 gives 45.8), with 60 degrees of downward
        travel available.
        """
        if self.target_row is None:
            return
        if not 1 <= self.target_row <= len(self.row_heights):
            return
        target_z = self.row_heights[self.target_row - 1]

        distance = self._range_ahead()
        if distance is None or distance < 0.05:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, CAMERA_FRAME, rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return
        camera_z = tf.transform.translation.z

        # base_link sits above base_footprint, and row heights are quoted in base_link,
        # so put both in the same frame before taking the difference.
        drop = (camera_z - BASE_LINK_Z) - target_z
        desired = -math.atan2(drop, distance)
        desired = max(HEAD_TILT_MIN, min(HEAD_TILT_MAX, desired))

        # Only re-send on a meaningful change. Every JointTrajectory replaces the one in
        # progress and restarts its time_from_start, so a trajectory re-sent every tick
        # never finishes and the head never actually arrives.
        if self.head_tilt is not None and abs(desired - self.head_tilt) < 0.05:
            return
        self.head_tilt = desired

        traj = JointTrajectory()
        traj.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, float(desired)]
        point.time_from_start = Duration(sec=1, nanosec=0)
        traj.points = [point]
        self.pub_head.publish(traj)
        self.get_logger().info(
            f"head tilt -> {math.degrees(-desired):+.0f} deg down "
            f"(row {self.target_row} at {distance:.2f} m)"
        )

    def _on_book_point(self, msg: PointStamped) -> None:
        self.book_point_at = self._now()

    def _book_located(self, max_age: float = 1.5) -> bool:
        """Whether the book is currently being located in 3D, not merely expected."""
        if self.book_point_at is None:
            return False
        return (self._now() - self.book_point_at) <= max_age

    def _do_acquire(self) -> None:
        """Hold at a working distance until the book is seen and centred.

        This is a checkpoint, not a drive. Everything after it depends on the book being
        genuinely in view, so it is worth a few seconds here rather than discovering at
        grasping range that the target was lost on the way in.

        The head is aimed at the row first: the books are well below the markers, and
        without the tilt the target may not be in frame at all from here.
        """
        self._stop()
        self._aim_head()

        bearing = self._column_cx_fresh()
        located = self._book_located()

        if bearing is None:
            self.get_logger().warn(
                "acquiring: no bearing", throttle_duration_sec=3.0)
            return

        error_px = bearing - self.image_width / 2.0
        if abs(error_px) > self.acquire_tol:
            cmd = Twist()
            cmd.angular.z = -math.copysign(
                min(self.max_yaw, 0.003 * abs(error_px) + 0.06), error_px)
            self.pub_cmd.publish(cmd)
            self.get_logger().info(
                f"acquiring: centring, error {error_px:+.0f}px",
                throttle_duration_sec=3.0)
            return

        if not located:
            self.get_logger().warn(
                "acquiring: centred but the book is not located yet",
                throttle_duration_sec=3.0)
            return

        self.get_logger().info(
            f"book acquired at {self._range_ahead() or float('nan'):.2f} m; closing in")
        self.approach_target = self.standoff
        self._enter(State.APPROACH)

    def _do_centre(self) -> None:
        column_cx = self._column_cx_fresh()
        if column_cx is None:
            self._stop()
            return
        error_px = column_cx - self.image_width / 2.0
        if abs(error_px) <= self.centre_tol:
            self._stop()
            # Hold here until the row has been read. This is the last point at which the
            # whole column is in frame; drive closer and the chance is gone.
            if not self.row_seen:
                self.get_logger().info(
                    "centred; waiting for the row before closing in",
                    throttle_duration_sec=5.0,
                )
                return
            # Drive to the acquire checkpoint first, not straight to grasping range.
            self.approach_target = self.acquire_range
            self._enter(State.APPROACH)
            return
        # Positive error means the column is right of centre, so turn clockwise.
        cmd = Twist()
        cmd.angular.z = -math.copysign(
            min(self.max_yaw, 0.004 * abs(error_px) + 0.08), error_px
        )
        self.pub_cmd.publish(cmd)

    def _do_approach(self) -> None:
        ahead = self._range_ahead()
        if ahead is None:
            self.get_logger().warn("no forward LiDAR returns; holding", throttle_duration_sec=3.0)
            self._stop()
            return
        remaining = ahead - self.approach_target
        if remaining <= self.standoff_tol:
            self._stop()
            # Two-stage: pause at the acquire checkpoint, then commit to the final drive.
            if self.approach_target > self.standoff:
                self._enter(State.ACQUIRE)
            else:
                self._enter(State.SQUARE)
            return

        # Keep the target row in frame as the gap closes.
        self._aim_head()

        cmd = Twist()
        cmd.linear.x = min(self.max_fwd, max(0.05, 0.5 * remaining))

        # Correct sideways, not by turning.
        #
        # Turning to chase the bearing while driving converts a small angular error into a
        # large lateral excursion: the robot yaws a little, then drives along the new
        # heading. One run ended at y = -2.25, past the last column and off the end of the
        # shelf unit, having started centred.
        #
        # The base strafes cleanly once the arms are stowed (measured: vy = +0.20 gives
        # dy = +0.233 with dyaw = 0.000), so lateral error can be taken out directly while
        # the heading stays square to the shelf. The earlier belief that strafing yaws the
        # base was an artefact of measuring with the arms extended.
        column_cx = self._column_cx_fresh()
        bearing = "stale"
        if column_cx is not None:
            error_px = column_cx - self.image_width / 2.0
            bearing = f"{error_px:+6.1f}px"
            if abs(error_px) > self.centre_tol:
                # Image x grows to the right; +y is to the robot's left.
                cmd.linear.y = -math.copysign(
                    min(self.max_lateral, 0.0012 * abs(error_px)), error_px)
        self.pub_cmd.publish(cmd)

        # Log what was commanded alongside what the range is doing. Range alone cannot
        # distinguish "commanding zero" from "commanding motion and not getting it", and
        # those have opposite fixes.
        self.get_logger().info(
            f"ahead={ahead:.2f} remaining={remaining:+.2f} "
            f"cmd vx={cmd.linear.x:.3f} wz={cmd.angular.z:+.3f} bearing={bearing}",
            throttle_duration_sec=2.0,
        )

    def _do_square(self) -> None:
        angle = self._shelf_angle()
        if angle is None:
            # No credible flat surface ahead. Stop where we are rather than turning
            # towards whatever else is in view.
            self._stop()
            self.get_logger().warn("no flat face to square against; finishing as-is")
            self._enter(State.DONE)
            return
        if abs(angle) <= self.square_tol:
            self._stop()
            ahead = self._range_ahead()
            self.get_logger().info(
                f"approach complete — shelf at {ahead:.2f} m, "
                f"face angle {math.degrees(angle):+.1f} deg"
                if ahead is not None else "approach complete"
            )
            self._enter(State.DONE)
            return

        # Rotate AGAINST the error. The fitted angle is the yaw error itself, so turning
        # by +angle drives further off square, not towards it -- which sent the robot
        # 155 degrees around until the shelf left the cone and it squared to a wall.
        cmd = Twist()
        cmd.angular.z = -math.copysign(min(self.max_yaw, 0.8 * abs(angle) + 0.08), angle)
        self.pub_cmd.publish(cmd)
        self.get_logger().info(
            f"squaring: face angle {math.degrees(angle):+.1f} deg, "
            f"wz={cmd.angular.z:+.3f}",
            throttle_duration_sec=2.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApproachNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())  # never leave the base driving
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
