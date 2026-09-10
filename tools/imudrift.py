#!/usr/bin/env python3
"""Can the IMU see the base turning, when odometry cannot?

    tools/in-sim imudrift.py [seconds_per_window] [windows]

The base will not hold still. With nothing commanded it yaws at about 0.27 deg/s and
never stops, and over a grasp that takes a minute or more that is 19 degrees -- which
swings a book 0.72 m out through 240 mm sideways, past everything downstream that tries
to correct for it. Odometry is no help: measured over 45 s, ground truth had the base
turn -9.02 degrees while odom reported +0.75.

The base hold currently corrects yaw from the shelf plane in the depth image, which works
but only while the shelf is in frame and only at about 1.4 degrees of accuracy, and the
arm goes in front of the camera exactly when it matters most.

The robot also carries an IMU, and an IMU measures angular RATE directly rather than
integrating wheel motion that never happened. So this asks whether it sees the drift.
Three sources over the same windows: the IMU, odometry, and Gazebo ground truth.
"""
import math
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)

WINDOW = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
WINDOWS = int(sys.argv[2]) if len(sys.argv) > 2 else 4

SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=10)


def gz(*args, timeout=20):
    try:
        return subprocess.run(["gz", *args], capture_output=True, text=True,
                              timeout=timeout).stdout
    except Exception:  # noqa: BLE001
        return ""


def truth():
    """Base position and yaw from Gazebo, or None."""
    raw = gz("topic", "-e", "-t", "/world/erc_world/dynamic_pose/info", "-n", "1")
    name = None
    section = None
    fields = {}
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith('name: "'):
            if name == "tiago_pro" and "px" in fields:
                break
            name = line.split('"')[1]
            section = None
            fields = {}
        elif line.startswith("position"):
            section = "p"
        elif line.startswith("orientation"):
            section = "q"
        elif name and section and ":" in line:
            key, _, value = line.partition(":")
            key = key.strip()
            if key in ("x", "y", "z", "w"):
                tag = section + key
                if tag not in fields:
                    try:
                        fields[tag] = float(value)
                    except ValueError:
                        pass
    if name != "tiago_pro" or "px" not in fields or "qw" not in fields:
        return None
    yaw = math.atan2(2.0 * (fields["qw"] * fields["qz"] + fields["qx"] * fields["qy"]),
                     1.0 - 2.0 * (fields["qy"] ** 2 + fields["qz"] ** 2))
    return fields["px"], fields["py"], yaw


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = rclpy.create_node("imudrift")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

    state = {"imu": None, "odom": None, "imu_n": 0}

    def on_imu(msg):
        state["imu"] = msg
        state["imu_n"] += 1

    node.create_subscription(Imu, "/base_imu", on_imu, SENSOR_QOS)
    node.create_subscription(Odometry, "/odom",
                             lambda m: state.__setitem__("odom", m), 10)

    deadline = time.time() + 8.0
    while time.time() < deadline and (state["imu"] is None or state["odom"] is None):
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["imu"] is None:
        print("nothing on /base_imu after 8 s")
        return 1
    print("IMU up, %d messages in the first 8 s" % state["imu_n"])
    print("")
    print("  window   imu mean   imu integ   odom       truth      imu err")
    print("           deg/s      deg         deg        deg        deg")
    print("  " + "-" * 66)

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    for index in range(WINDOWS):
        start_truth = truth()
        start_odom = state["odom"]
        start_sim = sim_now()
        samples = []
        while sim_now() - start_sim < WINDOW:
            rclpy.spin_once(node, timeout_sec=0.05)
            if state["imu"] is not None:
                samples.append(float(state["imu"].angular_velocity.z))
        elapsed = sim_now() - start_sim
        end_truth = truth()
        end_odom = state["odom"]
        if not samples or start_truth is None or end_truth is None:
            print("  %-8d (no reading)" % (index + 1))
            continue

        imu_mean = sum(samples) / len(samples)
        imu_integrated = imu_mean * elapsed
        turned = wrap(end_truth[2] - start_truth[2])

        odom_turned = float("nan")
        if start_odom is not None and end_odom is not None:
            def oyaw(msg):
                q = msg.pose.pose.orientation
                return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
            odom_turned = wrap(oyaw(end_odom) - oyaw(start_odom))

        print("  %-8d %+8.4f   %+8.3f   %+8.3f   %+8.3f   %+8.3f"
              % (index + 1, math.degrees(imu_mean), math.degrees(imu_integrated),
                 math.degrees(odom_turned), math.degrees(turned),
                 math.degrees(imu_integrated - turned)))

    print("")
    print("An IMU that tracks the truth is a yaw the base hold can cancel without the")
    print("camera, at whatever rate the IMU publishes, whether or not the shelf is in")
    print("frame and whether or not the arm is in the way of it.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
