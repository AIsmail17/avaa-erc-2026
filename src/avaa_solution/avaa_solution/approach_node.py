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
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

TOPIC_TARGET_COLUMN_X = "/avaa/perception/target_column_x"
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
    CENTRE = "centring"
    APPROACH = "approaching"
    SQUARE = "squaring"
    DONE = "done"
    FAILED = "failed"


class ApproachNode(Node):
    def __init__(self) -> None:
        super().__init__("avaa_approach")

        # Distance from the shelf face to stop at. The books sit at X=2.90 and the shelf
        # unit is centred at X=3.0 and 0.30 m deep, so the face is at about 2.85.
        self.declare_parameter("standoff_m", 0.60)
        self.declare_parameter("centre_tolerance_px", 12.0)
        self.declare_parameter("standoff_tolerance_m", 0.05)
        self.declare_parameter("square_tolerance_rad", 0.05)
        self.declare_parameter("max_yaw_rate", 0.45)
        self.declare_parameter("max_forward", 0.22)
        # Refuse to drive closer than this whatever the standoff says, so a bad reading
        # cannot push the base into the shelf. A collision costs 0.5 points each time.
        self.declare_parameter("min_safe_range_m", 0.40)
        self.declare_parameter("state_timeout_sec", 45.0)
        self.declare_parameter("image_width_px", 640)
        self.declare_parameter("tuck_time_sec", 5.0)

        self.standoff = float(self.get_parameter("standoff_m").value)
        self.centre_tol = float(self.get_parameter("centre_tolerance_px").value)
        self.standoff_tol = float(self.get_parameter("standoff_tolerance_m").value)
        self.square_tol = float(self.get_parameter("square_tolerance_rad").value)
        self.max_yaw = float(self.get_parameter("max_yaw_rate").value)
        self.max_fwd = float(self.get_parameter("max_forward").value)
        self.min_safe = float(self.get_parameter("min_safe_range_m").value)
        self.timeout = float(self.get_parameter("state_timeout_sec").value)
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
        if self.state is State.WAITING:
            # Stow the arms before any driving happens.
            self._send_tuck()
            self._enter(State.TUCK)

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

    def _forward_points(self, half_angle: float = 0.30) -> List[Tuple[float, float]]:
        """LiDAR returns within +/- half_angle of straight ahead, as (x, y) in the scan frame."""
        if self.scan is None:
            return []
        points = []
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r) or not (self.scan.range_min < r < self.scan.range_max):
                continue
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if abs(angle) > half_angle:
                continue
            points.append((r * math.cos(angle), r * math.sin(angle)))
        return points

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
        """Angle of the shelf face relative to the robot, 0 when square on.

        Fits a line to the forward returns. The shelf front is flat and 5.25 m wide, so
        within a narrow cone it is a clean straight edge.
        """
        points = self._forward_points(half_angle=0.45)
        if len(points) < 12:
            return None
        xs = np.array([p[0] for p in points])
        ys = np.array([p[1] for p in points])
        # x as a function of y: the face is roughly parallel to the robot's y axis, so
        # this avoids the vertical-line singularity of fitting y = f(x).
        slope, _intercept = np.polyfit(ys, xs, 1)
        return float(math.atan(slope))

    # ------------------------------------------------------------------ control

    def _enter(self, state: State) -> None:
        if state is not self.state:
            self.get_logger().info(f"{self.state.value} -> {state.value}")
            self.state = state
            self.state_since = self._now()

    def _publish_state(self) -> None:
        self.pub_state.publish(String(data=self.state.value))

    def _stop(self) -> None:
        self.pub_cmd.publish(Twist())

    def _tick(self) -> None:
        self._publish_state()

        if self.state in (State.DONE, State.FAILED, State.WAITING):
            if self.state is not State.WAITING:
                self._stop()
            return

        if (self._now() - self.state_since) > self.timeout:
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
            self._enter(State.CENTRE)

    def _do_centre(self) -> None:
        column_cx = self._column_cx_fresh()
        if column_cx is None:
            self._stop()
            return
        error_px = column_cx - self.image_width / 2.0
        if abs(error_px) <= self.centre_tol:
            self._stop()
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
        remaining = ahead - self.standoff
        self.get_logger().info(
            f"ahead={ahead:.2f} m  remaining={remaining:+.2f} m",
            throttle_duration_sec=3.0,
        )
        if remaining <= self.standoff_tol:
            self._stop()
            self._enter(State.SQUARE)
            return

        cmd = Twist()
        cmd.linear.x = min(self.max_fwd, max(0.05, 0.5 * remaining))
        # Keep the column centred while driving; small correction only.
        column_cx = self._column_cx_fresh()
        if column_cx is not None:
            error_px = column_cx - self.image_width / 2.0
            if abs(error_px) > self.centre_tol:
                cmd.angular.z = -math.copysign(min(0.15, 0.002 * abs(error_px)), error_px)
        self.pub_cmd.publish(cmd)

    def _do_square(self) -> None:
        angle = self._shelf_angle()
        if angle is None:
            self._stop()
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
        cmd = Twist()
        cmd.angular.z = math.copysign(min(self.max_yaw, 0.8 * abs(angle) + 0.08), angle)
        self.pub_cmd.publish(cmd)


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
