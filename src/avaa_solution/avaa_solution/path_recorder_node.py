"""Record the planned and executed navigation paths, for RViz.

Neither leg of the trip hands a global planner a goal. The approach and the delivery each
drive towards a standoff point they re-measure from the camera every frame, so the plan
worth drawing is that standoff point, as the robot currently believes it, joined to where
the leg began. The executed path is where the base went.

    /avaa/nav/planned_approach   nav_msgs/Path   odom   leg start -> book standoff
    /avaa/nav/planned_delivery   nav_msgs/Path   odom   leg start -> bin standoff
    /avaa/nav/executed_path      nav_msgs/Path   odom   base_link, every 2 cm

It only listens and publishes paths. Nothing here moves the robot, so leaving it out of a
run changes nothing. ``solution.launch.py rviz:=true`` starts it with rviz/nav_paths.rviz.

The executed path is wheel odometry through TF, which cannot see the base sliding across
its mecanum rollers (see deliver_node.py). It is drawn as the robot believed it.
"""

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

TOPIC_APPROACH_STATE = "/avaa/approach/state"
TOPIC_DELIVER_STATE = "/avaa/deliver/state"
TOPIC_BOOK_POINT = "/avaa/perception/target_book_point"
TOPIC_BIN_POINT = "/avaa/perception/bin_point"
TOPIC_PLANNED_APPROACH = "/avaa/nav/planned_approach"
TOPIC_PLANNED_DELIVERY = "/avaa/nav/planned_delivery"
TOPIC_EXECUTED = "/avaa/nav/executed_path"

# Controller states in which the base is driving towards its target. States where only the
# arm moves are left out, so a leg's plan stops changing once the base has arrived.
APPROACH_DRIVING = ("searching", "centring", "acquiring", "approaching", "squaring",
                    "retreating")
DELIVERY_DRIVING = ("seeking", "driving", "squaring")

# How far short of its target each leg means to stop: the approach node's standoff_m, and
# the delivery node's table_standoff_m from the bin's centre.
BOOK_STANDOFF_M = 0.65
BIN_STANDOFF_M = 0.82

EXECUTED_STEP_M = 0.02
PLANNED_STEP_M = 0.05

Point2 = Tuple[float, float]


def rotate(q, v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Rotate the vector v by the unit quaternion q."""
    tx = 2.0 * (q.y * v[2] - q.z * v[1])
    ty = 2.0 * (q.z * v[0] - q.x * v[2])
    tz = 2.0 * (q.x * v[1] - q.y * v[0])
    return (v[0] + q.w * tx + (q.y * tz - q.z * ty),
            v[1] + q.w * ty + (q.z * tx - q.x * tz),
            v[2] + q.w * tz + (q.x * ty - q.y * tx))


def straight_line(start: Point2, goal: Point2, step: float) -> List[Point2]:
    """Return points from start to goal no more than step apart, both ends included."""
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    count = max(1, int(math.ceil(math.hypot(dx, dy) / step)))
    return [(start[0] + dx * i / count, start[1] + dy * i / count) for i in range(count + 1)]


def standoff_point(robot: Point2, target: Point2, standoff: float) -> Point2:
    """Return the point standoff metres short of target, on the line from robot."""
    dx, dy = target[0] - robot[0], target[1] - robot[1]
    distance = math.hypot(dx, dy)
    if distance <= standoff:
        return robot
    scale = (distance - standoff) / distance
    return (robot[0] + dx * scale, robot[1] + dy * scale)


class Leg:
    """One drive towards a target: where it began, and where it currently means to stop."""

    def __init__(self, standoff: float) -> None:
        self.standoff = standoff
        self.start: Optional[Point2] = None
        self.goal: Optional[Point2] = None
        self.active = False


class PathRecorder(Node):
    """Publish the planned and executed paths of the approach and the delivery."""

    def __init__(self) -> None:
        super().__init__("avaa_path_recorder")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.approach = Leg(BOOK_STANDOFF_M)
        self.delivery = Leg(BIN_STANDOFF_M)
        self.executed: List[Point2] = []

        self.pub_approach = self.create_publisher(Path, TOPIC_PLANNED_APPROACH, 10)
        self.pub_delivery = self.create_publisher(Path, TOPIC_PLANNED_DELIVERY, 10)
        self.pub_executed = self.create_publisher(Path, TOPIC_EXECUTED, 10)

        self.create_subscription(
            String, TOPIC_APPROACH_STATE,
            lambda msg: self._on_state(self.approach, msg.data, APPROACH_DRIVING), 10)
        self.create_subscription(
            String, TOPIC_DELIVER_STATE,
            lambda msg: self._on_state(self.delivery, msg.data, DELIVERY_DRIVING), 10)
        self.create_subscription(
            PointStamped, TOPIC_BOOK_POINT,
            lambda msg: self._on_target(self.approach, msg), 10)
        self.create_subscription(
            PointStamped, TOPIC_BIN_POINT,
            lambda msg: self._on_target(self.delivery, msg), 10)
        self.create_timer(0.2, self._tick)
        self.get_logger().info("recording planned and executed paths in %s" % ODOM_FRAME)

    def _robot_xy(self) -> Optional[Point2]:
        """Return the base's position in odom, or None while TF does not have it."""
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, BASE_FRAME, Time())
        except TransformException:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _on_state(self, leg: Leg, state: str, driving: Tuple[str, ...]) -> None:
        """Start a leg the first time its controller drives, and pause it when it stops."""
        leg.active = state in driving
        if leg.active and leg.start is None:
            leg.start = self._robot_xy()

    def _on_target(self, leg: Leg, msg: PointStamped) -> None:
        """Re-measure a driving leg's goal from a fresh sighting of its target."""
        if not leg.active or leg.start is None:
            return
        robot = self._robot_xy()
        try:
            tf = self.tf_buffer.lookup_transform(ODOM_FRAME, msg.header.frame_id, Time())
        except TransformException:
            return
        if robot is None:
            return
        x, y, _ = rotate(tf.transform.rotation, (msg.point.x, msg.point.y, msg.point.z))
        target = (x + tf.transform.translation.x, y + tf.transform.translation.y)
        leg.goal = standoff_point(robot, target, leg.standoff)

    def _tick(self) -> None:
        """Extend the executed path and republish all three paths."""
        robot = self._robot_xy()
        if robot is not None and (
                not self.executed
                or math.hypot(robot[0] - self.executed[-1][0],
                              robot[1] - self.executed[-1][1]) >= EXECUTED_STEP_M):
            self.executed.append(robot)
        stamp = self.get_clock().now().to_msg()
        self.pub_executed.publish(self._path(self.executed, stamp))
        for leg, pub in ((self.approach, self.pub_approach),
                         (self.delivery, self.pub_delivery)):
            if leg.start is not None and leg.goal is not None:
                pub.publish(self._path(straight_line(leg.start, leg.goal, PLANNED_STEP_M),
                                       stamp))

    @staticmethod
    def _path(points: List[Point2], stamp) -> Path:
        """Build a Path in odom through the given points."""
        path = Path()
        path.header.frame_id = ODOM_FRAME
        path.header.stamp = stamp
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path


def main(args=None) -> None:
    """Run the path recorder until interrupted."""
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
