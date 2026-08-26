"""Perception node — finds books in the head camera feed.

Kept separate from the mission state machine so perception, planning and execution stay
distinct processes, which the Phase 1 rubric explicitly rewards.

Publishes every detection as a ``vision_msgs/Detection2DArray`` on
``/avaa/perception/books``, with the colour in ``class_id``. When the target column is
known it also reports the target book's row on ``/avaa/perception/target_row`` and saves
a timestamped annotated frame for the judges.

It deliberately does **not** publish to ``/erc/shelf_row_identification``. The mission
node owns the scoring topics, so there is exactly one place responsible for what gets
reported to the committee.
"""

import os
from datetime import datetime
from typing import List, Optional

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from avaa_solution.vision import book_detector as bd

TOPIC_RGB = "/head_front_camera/head_front_camera/color/image_raw"
TOPIC_DETECTIONS = "/avaa/perception/books"
TOPIC_TARGET_ROW = "/avaa/perception/target_row"

# The camera publishes best-effort; a reliable subscriber would receive nothing.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("avaa_perception")

        self.declare_parameter("book_colour", "")
        # 0-based index into the columns visible in frame, left to right. -1 means the
        # target column is not known yet -- the marker detector will supply it.
        self.declare_parameter("target_column_index", -1)
        self.declare_parameter("save_images", True)
        # Only src/ is bind-mounted into the container, so this is the deepest path that
        # still lands inside the git repository on the host. See PERCEPTION.md.
        self.declare_parameter(
            "image_dir", "/opt/erc_ws/src/avaa_solution/erc_images"
        )
        self.declare_parameter("min_save_interval_sec", 2.0)
        self.declare_parameter("detect_period_sec", 0.2)

        self.book_colour = str(self.get_parameter("book_colour").value).lower()
        self.image_dir = str(self.get_parameter("image_dir").value)
        self.save_images = bool(self.get_parameter("save_images").value)
        self.min_save_interval = float(self.get_parameter("min_save_interval_sec").value)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_header = None
        self.last_saved_at: Optional[float] = None
        self.last_reported_row: Optional[int] = None

        if self.save_images:
            os.makedirs(self.image_dir, exist_ok=True)

        self.create_subscription(Image, TOPIC_RGB, self._on_image, SENSOR_QOS)
        self.pub_detections = self.create_publisher(Detection2DArray, TOPIC_DETECTIONS, 10)
        self.pub_target_row = self.create_publisher(Int32, TOPIC_TARGET_ROW, 10)

        period = float(self.get_parameter("detect_period_sec").value)
        self.create_timer(period, self._process)

        if self.book_colour and self.book_colour not in bd.COLOURS:
            self.get_logger().error(
                f"book_colour {self.book_colour!r} is not one of {bd.COLOURS}"
            )
        self.get_logger().info(
            f"perception up — target colour {self.book_colour or '(unset)'}, "
            f"images -> {self.image_dir if self.save_images else 'disabled'}"
        )

    # ------------------------------------------------------------------ callbacks

    def _on_image(self, msg: Image) -> None:
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_header = msg.header
        except Exception as exc:  # noqa: BLE001 - never let a bad frame kill the node
            self.get_logger().warn(f"could not convert frame: {exc}")

    def _process(self) -> None:
        if self.latest_frame is None:
            return
        frame = self.latest_frame

        try:
            books = bd.detect_books(frame)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"detection failed: {exc}")
            return

        columns = bd.group_into_columns(books)
        self._publish_detections(books, columns)

        column_index = int(self.get_parameter("target_column_index").value)
        if column_index < 0 or not self.book_colour:
            return  # target column not established yet

        if column_index >= len(columns):
            self.get_logger().warn(
                f"target column index {column_index} but only {len(columns)} column(s) in view"
            )
            return

        column = columns[column_index]
        row = bd.row_of(column, self.book_colour)
        if row is None:
            self.get_logger().warn(
                f"row not resolvable: column has {len(column)} of "
                f"{bd.ROWS_PER_COLUMN} books visible"
            )
            return

        if row != self.last_reported_row:
            self.get_logger().info(f"target {self.book_colour} book is on row {row}")
            self.last_reported_row = row
        self.pub_target_row.publish(Int32(data=row))

        target = bd.find_book(columns, column_index, self.book_colour)
        if target is not None:
            self._maybe_save(frame, books, target, row)

    # ------------------------------------------------------------------ helpers

    def _publish_detections(self, books: List[bd.Book],
                            columns: List[List[bd.Book]]) -> None:
        msg = Detection2DArray()
        if self.latest_header is not None:
            msg.header = self.latest_header

        # Map each book to its column so consumers do not have to re-cluster.
        column_of = {}
        for ci, column in enumerate(columns):
            for book in column:
                column_of[id(book)] = ci

        for book in books:
            det = Detection2D()
            det.header = msg.header
            bbox = BoundingBox2D()
            bbox.center.position.x = book.cx
            bbox.center.position.y = book.cy
            bbox.size_x = float(book.w)
            bbox.size_y = float(book.h)
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = book.colour
            hyp.hypothesis.score = 1.0
            det.results.append(hyp)
            det.id = str(column_of.get(id(book), -1))
            msg.detections.append(det)

        self.pub_detections.publish(msg)

    def _maybe_save(self, frame, books: List[bd.Book], target: bd.Book, row: int) -> None:
        """Save an annotated frame, rate-limited.

        The rules require images to be written by the solution during the trial from the
        live feed, each carrying a timestamp so judges can verify when it was captured.
        The timestamp goes in the filename and is burned into the image, since a filename
        alone is trivially alterable after the fact.
        """
        if not self.save_images:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_saved_at is not None and (now - self.last_saved_at) < self.min_save_interval:
            return
        self.last_saved_at = now

        stamp = datetime.now().strftime("%Y-%m-%dT%H-%M-%S.%f")[:-3]
        caption = f"AVAA {stamp}  row={row}  colour={self.book_colour}"
        vis = bd.annotate(frame, books, highlight=target, caption=caption)

        path = os.path.join(self.image_dir, f"row_{row}_{self.book_colour}_{stamp}.png")
        try:
            import cv2

            cv2.imwrite(path, vis)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"could not save {path}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT from a terminal, or SIGTERM from `ros2 launch` shutting the run down.
        # Both are ordinary exits, not faults -- do not spew a traceback over the logs
        # the judges will be reading.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
