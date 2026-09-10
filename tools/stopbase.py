#!/usr/bin/env python3
"""Cancel whatever the last thing to touch cmd_vel left behind.

    tools/in-sim stopbase.py [seconds]

A node killed with SIGKILL does not get to publish a stop, and the mecanum plugin keeps
applying the last velocity it was given -- forever, because nothing damps this base. So a
run that ends badly leaves the robot turning, and the NEXT run inherits it: measured, a
grasp opened with the base 107 degrees off square, having been teleported to yaw zero
forty seconds earlier by a fixture that then said nothing about velocity.

That is a test rig manufacturing the fault it is being used to measure, and worse, it is
indistinguishable in the logs from the drift that is real.

A zero twist does not stop a base that is already coasting -- that is measured and it is
why tools/stopcoast.py exists -- but it does cancel a standing COMMAND, which is a
different thing and the thing that matters here. Publish it for a couple of seconds so it
survives whatever else is still in flight.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0


def main():
    rclpy.init()
    node = rclpy.create_node("stopbase")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    sent = 0
    began = time.time()
    while time.time() - began < SECONDS:
        pub.publish(Twist())
        sent += 1
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.05)
    print("published %d zero twists over %.1f s" % (sent, SECONDS))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
