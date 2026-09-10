#!/usr/bin/env python3
"""Can the base be braked against its own yaw, using the IMU?

    tools/in-sim yawbrake.py [gain ...]

The base coasts. Nothing damps it, so whatever angular velocity it picks up it keeps, and
during a grasp that ran a minute and a half it turned 19.4 degrees -- which swings a book
0.72 m out through 240 mm sideways and defeats everything downstream. The wheels cannot
shed it: mu2 is zero across the roller axis and commanding zero wheel speed asks the
wheels not to turn rather than asking the base to stop.

But a counter-command does work when it can be aimed. tools/stopcoast.py measured the
linear coast falling from 7.07 mm/s to 2.02 driving against it at a gain of 2, and the
only reason that was never applied to the yaw is that nothing could measure the yaw:
odometry sees about a quarter of it. The IMU sees essentially all of it -- integrated over
four ten-second windows it tracked ground truth to under half a degree (tools/imudrift.py).

So this closes that loop and measures what it buys. For each gain it publishes
angular.z = -gain * (IMU yaw rate) at 20 Hz for a fixed window, and reports the drift
before, during and after against Gazebo ground truth. A gain that removes most of the
turn without oscillating is the one to put in the base hold.
"""
import math
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)

ALPHA = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
GAINS = [float(a) for a in sys.argv[2:]] or [0.0, 1.0, 2.0, 4.0]
WINDOW = 10.0
DEADBAND = 0.004        # rad/s, about 0.23 deg/s
MAX_YAW = 0.15          # rad/s, grasp_node's hold_max_yaw_rad_s

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
    """Base x, y and yaw from Gazebo, or None."""
    raw = gz("topic", "-e", "-t", "/world/erc_world/dynamic_pose/info", "-n", "1")
    name = None
    section = None
    fields = {}
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith('name: "'):
            if name == "tiago_pro" and "qw" in fields:
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
    if name != "tiago_pro" or "qw" not in fields:
        return None
    yaw = math.atan2(2.0 * (fields["qw"] * fields["qz"] + fields["qx"] * fields["qy"]),
                     1.0 - 2.0 * (fields["qy"] ** 2 + fields["qz"] ** 2))
    return fields["px"], fields["py"], yaw


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    rclpy.init()
    node = rclpy.create_node("yawbrake")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    state = {"rate": 0.0, "seen": 0, "raw": []}

    def on_imu(msg):
        # A light low pass, so a single noisy sample cannot command a lurch.
        state["rate"] = (1.0 - ALPHA) * state["rate"] + ALPHA * float(msg.angular_velocity.z)
        state["raw"].append(float(msg.angular_velocity.z))
        if len(state["raw"]) > 4000:
            del state["raw"][:2000]
        state["seen"] += 1

    node.create_subscription(Imu, "/base_imu", on_imu, SENSOR_QOS)

    deadline = time.time() + 8.0
    while time.time() < deadline and state["seen"] < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    if state["seen"] == 0:
        print("nothing on /base_imu")
        return 1

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    print("braking on the IMU yaw rate, %.0f s per gain, deadband %.3f rad/s, "
          "filter alpha %.2f" % (WINDOW, DEADBAND, ALPHA))

    # What the raw signal actually looks like, before anything is done with it.
    #
    # This matters more than it sounds. A rate damper is only sensible if the rate it
    # damps is mostly signal. If the mean is small and the spread is large, then the
    # base is shaking rather than turning, and braking on the instantaneous rate spends
    # the whole command budget fighting the shake.
    state["raw"] = []
    settle = sim_now()
    while sim_now() - settle < 6.0:
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.01)
    raw = list(state["raw"])
    if raw:
        mean = sum(raw) / len(raw)
        spread = (sum((r - mean) ** 2 for r in raw) / len(raw)) ** 0.5
        print("  raw IMU over 6 s, %d samples: mean %+0.4f, spread %0.4f, "
              "range %+0.4f to %+0.4f rad/s"
              % (len(raw), mean, spread, min(raw), max(raw)))
        print("  in degrees a second: mean %+0.2f, spread %0.2f, range %+0.2f to %+0.2f"
              % (math.degrees(mean), math.degrees(spread),
                 math.degrees(min(raw)), math.degrees(max(raw))))
    print("")
    print("  gain    turned deg   rate at end   moved mm   commands   worst cmd")
    print("  " + "-" * 66)

    for gain in GAINS:
        start = truth()
        if start is None:
            print("  %-6.1f  (no ground truth)" % gain)
            continue
        start_sim = sim_now()
        commands = 0
        worst = 0.0
        while sim_now() - start_sim < WINDOW:
            rclpy.spin_once(node, timeout_sec=0.02)
            twist = Twist()
            rate = state["rate"]
            if gain > 0.0 and abs(rate) > DEADBAND:
                value = float(max(-MAX_YAW, min(MAX_YAW, -gain * rate)))
                twist.angular.z = value
                commands += 1
                worst = max(worst, abs(value))
            pub.publish(twist)
            time.sleep(0.05)
        end = truth()
        if end is None:
            print("  %-6.1f  (no ground truth at the end)" % gain)
            continue
        turned = wrap(end[2] - start[2])
        moved = math.hypot(end[0] - start[0], end[1] - start[1])
        print("  %-6.1f  %+9.3f    %+8.4f      %7.1f    %7d    %7.3f"
              % (gain, math.degrees(turned), math.degrees(state["rate"]),
                 moved * 1000, commands, worst))
        # Let it settle between gains, with nothing commanded, so each trial starts
        # from whatever the last one left rather than from a command still in flight.
        pub.publish(Twist())
        rest = sim_now()
        while sim_now() - rest < 3.0:
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.02)

    pub.publish(Twist())
    print("")
    print("Gain 0 is the control: that row is the drift with nothing done about it.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
