"""Perception node — reads the column markers and finds books in the head camera feed.

Kept separate from the mission state machine so perception, planning and execution stay
distinct processes, which the Phase 1 rubric marks explicitly.

Two identifications are worth points, each scored twice (topic +1, annotated image +2):

* which shelf column carries the target marker digit
* which row the target-coloured book sits on within that column

This node works both out, publishes them on ``/avaa/perception/*``, and writes the
annotated images. It deliberately does **not** publish to ``/erc/shelf_column_identification``
or ``/erc/shelf_row_identification``: the mission node owns the scoring topics, so there is
exactly one place responsible for what reaches the committee.
"""

import os
from datetime import datetime
from typing import List, Optional, Tuple

import cv2
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
from avaa_solution.vision import marker_reader as mr

TOPIC_RGB = "/head_front_camera/head_front_camera/color/image_raw"
TOPIC_DETECTIONS = "/avaa/perception/books"
TOPIC_TARGET_ROW = "/avaa/perception/target_row"
TOPIC_TARGET_COLUMN = "/avaa/perception/target_column"

# The camera publishes best-effort; a reliable subscriber receives nothing at all.
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
        self.declare_parameter("shelf_column_number", 0)
        # Manual override of the column index, for testing without markers in view.
        # -1 (the default) means work it out from the marker digits.
        self.declare_parameter("target_column_index", -1)
        self.declare_parameter("save_images", True)
        # Only src/ is bind-mounted into the container, so this is the deepest path that
        # still lands inside the git repository on the host. See PERCEPTION.md.
        self.declare_parameter("image_dir", "/opt/erc_ws/src/avaa_solution/erc_images")
        self.declare_parameter("min_save_interval_sec", 2.0)
        self.declare_parameter("detect_period_sec", 0.2)

        self.book_colour = str(self.get_parameter("book_colour").value).lower()
        self.target_digit = int(self.get_parameter("shelf_column_number").value)
        self.image_dir = str(self.get_parameter("image_dir").value)
        self.save_images = bool(self.get_parameter("save_images").value)
        self.min_save_interval = float(self.get_parameter("min_save_interval_sec").value)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_header = None
        self.last_column_save: Optional[float] = None
        self.last_book_save: Optional[float] = None
        self.reported_column: Optional[int] = None
        self.reported_row: Optional[int] = None

        if self.save_images:
            os.makedirs(self.image_dir, exist_ok=True)

        self.create_subscription(Image, TOPIC_RGB, self._on_image, SENSOR_QOS)
        self.pub_detections = self.create_publisher(Detection2DArray, TOPIC_DETECTIONS, 10)
        self.pub_row = self.create_publisher(Int32, TOPIC_TARGET_ROW, 10)
        self.pub_column = self.create_publisher(Int32, TOPIC_TARGET_COLUMN, 10)

        self.create_timer(float(self.get_parameter("detect_period_sec").value), self._process)

        if self.book_colour and self.book_colour not in bd.COLOURS:
            self.get_logger().error(
                f"book_colour {self.book_colour!r} is not one of {bd.COLOURS}"
            )
        try:
            mr.load_templates()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"could not load marker templates: {exc}")

        self.get_logger().info(
            f"perception up — target marker {self.target_digit}, "
            f"colour {self.book_colour or '(unset)'}, "
            f"images -> {self.image_dir if self.save_images else 'disabled'}"
        )

    # ------------------------------------------------------------------ callbacks

    def _on_image(self, msg: Image) -> None:
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_header = msg.header
        except Exception as exc:  # noqa: BLE001 - a bad frame must not kill the node
            self.get_logger().warn(f"could not convert frame: {exc}")

    def _process(self) -> None:
        if self.latest_frame is None:
            return
        frame = self.latest_frame

        try:
            books = bd.detect_books(frame)
            markers = sorted(mr.read_markers(frame), key=lambda m: m.cx)
            # The markers define the columns. Falling back to gap clustering only when
            # none are visible, since that cannot identify a target column anyway.
            if markers:
                columns = bd.group_by_anchors(books, [m.cx for m in markers])
            else:
                columns = bd.group_into_columns(books)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"perception failed on this frame: {exc}")
            return

        self._publish_detections(books, columns)

        column_index = self._target_column_index(markers)
        if column_index is None:
            return

        if column_index != self.reported_column:
            self.get_logger().info(
                f"marker {self.target_digit} is column index {column_index} "
                f"({len(columns)} column(s) in view)"
            )
            self.reported_column = column_index
        self.pub_column.publish(Int32(data=column_index))
        self._save_column_image(frame, books, columns, markers, column_index)

        if not self.book_colour:
            return
        row = bd.row_of(columns[column_index], self.book_colour)
        if row is None:
            self.get_logger().warn(
                f"row not resolvable: column {column_index} shows "
                f"{len(columns[column_index])} of {bd.ROWS_PER_COLUMN} books"
            )
            return

        if row != self.reported_row:
            self.get_logger().info(f"target {self.book_colour} book is on row {row}")
            self.reported_row = row
        self.pub_row.publish(Int32(data=row))

        target = bd.find_book(columns, column_index, self.book_colour)
        if target is not None:
            self._save_book_image(frame, books, target, row)

    # ------------------------------------------------------------------ helpers

    def _target_column_index(self, markers: List[mr.Marker]) -> Optional[int]:
        """Index into ``columns`` of the column carrying the target marker digit.

        Because columns are anchored to the markers, the marker's own left-to-right
        position is the column index.
        """
        override = int(self.get_parameter("target_column_index").value)
        if override >= 0:
            return override
        if not self.target_digit:
            return None
        hits = [i for i, m in enumerate(markers)
                if m.digit == self.target_digit and m.confident]
        if len(hits) != 1:
            return None  # absent, or read twice -- move for a better view
        return hits[0]

    def _publish_detections(self, books: List[bd.Book],
                            columns: List[List[bd.Book]]) -> None:
        msg = Detection2DArray()
        if self.latest_header is not None:
            msg.header = self.latest_header

        column_of = {}
        for index, column in enumerate(columns):
            for book in column:
                column_of[id(book)] = index

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

    def _due(self, last: Optional[float]) -> bool:
        if not self.save_images:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return last is None or (now - last) >= self.min_save_interval

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _write(self, image, name: str) -> None:
        try:
            cv2.imwrite(os.path.join(self.image_dir, name), image)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"could not save {name}: {exc}")

    def _save_column_image(self, frame, books, columns, markers, column_index: int) -> None:
        """Annotated frame with a box around the identified column. Worth +2."""
        if not self._due(self.last_column_save):
            return
        self.last_column_save = self._now()

        stamp = datetime.now().strftime("%Y-%m-%dT%H-%M-%S.%f")[:-3]
        caption = f"AVAA {stamp}  column marker={self.target_digit}"
        vis = bd.annotate(frame, books, caption=caption)

        marker = markers[column_index] if column_index < len(markers) else None
        box = column_bbox(columns[column_index], marker)
        if box is not None:
            x, y, w, h = box
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 255, 255), 2)
        self._write(vis, f"column_{self.target_digit}_{stamp}.png")

    def _save_book_image(self, frame, books, target: bd.Book, row: int) -> None:
        """Annotated frame with a box around the target book. Worth +2."""
        if not self._due(self.last_book_save):
            return
        self.last_book_save = self._now()

        stamp = datetime.now().strftime("%Y-%m-%dT%H-%M-%S.%f")[:-3]
        caption = f"AVAA {stamp}  row={row}  colour={self.book_colour}"
        vis = bd.annotate(frame, books, highlight=target, caption=caption)
        self._write(vis, f"row_{row}_{self.book_colour}_{stamp}.png")


# ---------------------------------------------------------------------- pure helpers


def column_bbox(column: List[bd.Book],
                marker: Optional[mr.Marker] = None
                ) -> Optional[Tuple[int, int, int, int]]:
    """Bounding box spanning a column's books and, when given, its marker above.

    Including the marker matters: the points are awarded for a box around the identified
    *column*, and the marker is what identifies it.
    """
    xs: List[int] = []
    ys: List[int] = []
    for book in column:
        xs += [book.x, book.x + book.w]
        ys += [book.y, book.y + book.h]
    if marker is not None:
        xs += [marker.x, marker.x + marker.w]
        ys += [marker.y, marker.y + marker.h]
    if not xs:
        return None

    pad = 6
    x0, x1 = max(0, min(xs) - pad), max(xs) + pad
    y0, y1 = max(0, min(ys) - pad), max(ys) + pad
    return (x0, y0, x1 - x0, y1 - y0)


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
