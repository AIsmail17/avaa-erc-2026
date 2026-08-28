"""A small synchronous client for move_group.

MoveIt 2 in Humble has no Python bindings -- moveit_py arrived later -- so this talks to
move_group over its action and service interfaces directly. It is deliberately small: the
grasp controller needs four things, and everything else MoveIt offers is a distraction.

    plan and move to a joint configuration
    plan and move so the gripper reaches a pose
    move the gripper along a straight line, collision-checked
    tell the planner where the shelf is

Why this exists at all. The first version of this solution drove the arm from its own
analytic IK, which places a gripper accurately and knows nothing about what the arm passes
through on the way. Reaching into a 0.33 m shelf opening, the arm repeatedly arrived at
correct points by paths that went through the shelf: link contact sensors caught
arm_left_6 at world z=0.44, under the lowest shelf surface at 0.587, while its target sat
at 1.247. Four separate heuristics were added to the IK -- off the joint stops, near the
previous posture, out of the shelf volume, inside the target opening -- and each helped
and none of them is a collision checker.

The client owns its own node and spins it on a background thread. Blocking on a future
from inside another node's timer callback is re-entrant spinning, which rclpy refuses, and
the grasp controller is a timer-driven state machine.
"""

from __future__ import annotations

import threading
from typing import List, Optional, Sequence

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

ARM_GROUP = "arm_left_torso"
GRIPPER_GROUP = "gripper_left"
TIP_LINK = "gripper_left_grasping_link"
PLANNING_FRAME = "base_link"


def error_name(code: int) -> str:
    """Turn a MoveItErrorCodes value into something a log reader can act on."""
    known = {
        MoveItErrorCodes.SUCCESS: "success",
        MoveItErrorCodes.PLANNING_FAILED: "planning failed",
        MoveItErrorCodes.INVALID_MOTION_PLAN: "invalid motion plan",
        MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            "the scene changed under the plan",
        MoveItErrorCodes.CONTROL_FAILED: "the controller did not follow the trajectory",
        MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: "no sensor data",
        MoveItErrorCodes.TIMED_OUT: "timed out",
        MoveItErrorCodes.PREEMPTED: "preempted",
        MoveItErrorCodes.START_STATE_IN_COLLISION: "start state is in collision",
        MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            "start state violates the path constraints",
        MoveItErrorCodes.GOAL_IN_COLLISION: "goal is in collision",
        MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            "goal violates the path constraints",
        MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "goal constraints violated",
        MoveItErrorCodes.INVALID_GROUP_NAME: "no such planning group",
        MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "invalid goal constraints",
        MoveItErrorCodes.NO_IK_SOLUTION: "no IK solution",
    }
    return known.get(code, "error %d" % code)


class MoveItClient:
    """Synchronous access to move_group, on its own node and thread."""

    def __init__(self, name: str = "avaa_moveit_client", use_sim_time: bool = True):
        self.node = rclpy.create_node(name)
        self.node.set_parameters([
            rclpy.parameter.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, use_sim_time)
        ])
        self.move = ActionClient(self.node, MoveGroup, "move_action")
        self.execute = ActionClient(self.node, ExecuteTrajectory, "execute_trajectory")
        self.cartesian = self.node.create_client(
            GetCartesianPath, "compute_cartesian_path")
        self.apply_scene = self.node.create_client(
            ApplyPlanningScene, "apply_planning_scene")

        self.last_failure = ""
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ lifecycle

    def wait_until_ready(self, timeout: float = 30.0) -> bool:
        """Whether move_group is up. It takes a few seconds longer than the sim does."""
        return (self.move.wait_for_server(timeout_sec=timeout)
                and self.cartesian.wait_for_service(timeout_sec=timeout)
                and self.apply_scene.wait_for_service(timeout_sec=timeout))

    def shutdown(self) -> None:
        self._executor.shutdown()
        self.node.destroy_node()

    # ------------------------------------------------------------------ the scene

    def add_box(self, name: str, frame: str, centre, size,
                orientation: Optional[Quaternion] = None) -> bool:
        """Put a box in the planning scene, replacing any box of the same name."""
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [float(v) for v in size]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in centre)
        pose.orientation = orientation or Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        obj = CollisionObject()
        obj.header.frame_id = frame
        obj.id = name
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene.call_async(request)
        result = self._wait(future, timeout=10.0)
        return bool(result and result.success)

    def remove_object(self, name: str) -> bool:
        obj = CollisionObject()
        obj.id = name
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        result = self._wait(self.apply_scene.call_async(request), timeout=10.0)
        return bool(result and result.success)

    # ------------------------------------------------------------------ motion

    def move_to_joints(self, joint_names: Sequence[str], values: Sequence[float],
                       group: str = ARM_GROUP, tolerance: float = 0.01,
                       timeout: float = 120.0):
        """Plan and execute to a joint configuration. Returns an error code."""
        constraints = Constraints()
        for name, value in zip(joint_names, values):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return self._send_move(group, constraints, timeout)

    def move_to_pose(self, position, orientation: Quaternion,
                     group: str = ARM_GROUP, frame: str = PLANNING_FRAME,
                     link: str = TIP_LINK, position_tolerance: float = 0.01,
                     orientation_tolerance: float = 0.1, timeout: float = 120.0):
        """Plan and execute so ``link`` reaches a pose. Returns an error code."""
        constraints = Constraints()

        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [float(position_tolerance)]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(v) for v in position)
        pose.orientation = orientation

        wanted = PositionConstraint()
        wanted.header.frame_id = frame
        wanted.link_name = link
        wanted.constraint_region.primitives = [region]
        wanted.constraint_region.primitive_poses = [pose]
        wanted.weight = 1.0
        constraints.position_constraints = [wanted]

        facing = OrientationConstraint()
        facing.header.frame_id = frame
        facing.link_name = link
        facing.orientation = orientation
        facing.absolute_x_axis_tolerance = orientation_tolerance
        facing.absolute_y_axis_tolerance = orientation_tolerance
        facing.absolute_z_axis_tolerance = orientation_tolerance
        facing.weight = 1.0
        constraints.orientation_constraints = [facing]

        return self._send_move(group, constraints, timeout)

    def straight_line(self, waypoints: List[Pose], group: str = ARM_GROUP,
                      link: str = TIP_LINK, step: float = 0.01,
                      timeout: float = 120.0):
        """Move the gripper along a straight line, checking collisions as it goes.

        This is the reach into the shelf. A joint-space plan between two points either
        side of a shelf opening bows the arm sideways and the bow goes through the shelf;
        asking for the straight line keeps the hand on the one path that fits.

        Returns (error code, fraction of the path achieved). A fraction below 1.0 means
        the planner ran out of room, which is information: it says the reach is blocked
        rather than that the arm failed to follow it.
        """
        request = GetCartesianPath.Request()
        request.header.frame_id = PLANNING_FRAME
        request.group_name = group
        request.link_name = link
        request.waypoints = waypoints
        request.max_step = step
        request.jump_threshold = 0.0
        request.avoid_collisions = True

        result = self._wait(self.cartesian.call_async(request), timeout=30.0)
        if result is None:
            return MoveItErrorCodes.FAILURE, 0.0
        if result.fraction < 0.0:
            return result.error_code.val, 0.0
        if not result.solution.joint_trajectory.points:
            return result.error_code.val, float(result.fraction)

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = result.solution
        code = self._send_action(self.execute, goal, timeout)
        return code, float(result.fraction)

    # ------------------------------------------------------------------ plumbing

    def _send_move(self, group: str, constraints: Constraints, timeout: float):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.goal_constraints = [constraints]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        # Scaled down hard. This arm covers about 3 rad in 28 s and its controller aborts
        # trajectories it cannot keep up with, which presents as an arm that simply does
        # not move.
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        return self._send_action(self.move, goal, timeout)

    def _send_action(self, client: ActionClient, goal, timeout: float) -> int:
        """Send a goal and wait for its result, saying which step failed if one does.

        Returning a bare FAILURE for every problem here made three different faults look
        identical -- server missing, goal rejected, plan failed -- which is exactly the
        kind of silence this project has lost the most time to.
        """
        self.last_failure = ""
        if not client.wait_for_server(timeout_sec=10.0):
            self.last_failure = "action server never appeared"
            return MoveItErrorCodes.FAILURE
        handle = self._wait(client.send_goal_async(goal), timeout=15.0)
        if handle is None:
            self.last_failure = "no reply to the goal request"
            return MoveItErrorCodes.FAILURE
        if not handle.accepted:
            self.last_failure = "goal rejected by move_group"
            return MoveItErrorCodes.FAILURE
        wrapper = self._wait(handle.get_result_async(), timeout=timeout)
        if wrapper is None:
            self.last_failure = "no result within %.0f s" % timeout
            return MoveItErrorCodes.TIMED_OUT
        result = getattr(wrapper, "result", None)
        if result is None:
            self.last_failure = "result message had no payload"
            return MoveItErrorCodes.FAILURE
        code = getattr(result, "error_code", None)
        if code is None:
            self.last_failure = "result payload had no error_code: %s" % type(result)
            return MoveItErrorCodes.FAILURE
        return int(code.val)

    @staticmethod
    def _wait(future, timeout: float):
        """Wait on a future that another thread is spinning."""
        import time
        deadline = time.time() + timeout
        while time.time() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception:  # noqa: BLE001 - a failed call is not a crash
                    return None
            time.sleep(0.02)
        return None
