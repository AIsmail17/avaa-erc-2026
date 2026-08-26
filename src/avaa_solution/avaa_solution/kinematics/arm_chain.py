"""Forward and inverse kinematics for the TIAGo Pro left arm.

Built directly from the URDF rather than through MoveIt. MoveIt 2 is installed in the
competition image, but there is **no SRDF for this robot** anywhere in it -- only MoveIt's
own test fixtures -- so there is no configured planning group to ask for IK, and authoring
a robot config is not a good use of the remaining time.

The chain from base_link to gripper_left_grasping_link has eight moving joints: the
prismatic torso lift and seven revolute arm joints, every one of them rotating about its
own local Z. That is simple enough to compose directly.

No ROS dependency, so it is unit-testable without a simulator.

    >>> chain = ArmChain.from_urdf(path)
    >>> chain.fk([0.0] * 8)[:3, 3]        # gripper position in base_link
    >>> chain.ik([0.75, 0.2, 1.2])        # joint values reaching that point
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

DEFAULT_URDF = "/opt/erc_ws/src/erc_description/urdf/tiago_pro.urdf"
ROOT_LINK = "base_link"
TIP_LINK = "gripper_left_grasping_link"


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy is fixed-axis: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation about an arbitrary unit axis."""
    a = axis / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + s * k + (1 - c) * (k @ k)


def transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


@dataclass
class Joint:
    name: str
    kind: str                 # "revolute", "prismatic" or "fixed"
    origin: np.ndarray        # 4x4, parent -> joint frame at zero
    axis: Optional[np.ndarray]
    lower: float
    upper: float

    @property
    def moving(self) -> bool:
        return self.kind in ("revolute", "prismatic", "continuous")

    def transform_at(self, value: float) -> np.ndarray:
        if not self.moving or self.axis is None:
            return self.origin
        if self.kind == "prismatic":
            return self.origin @ transform(np.eye(3), self.axis * value)
        return self.origin @ transform(axis_rotation(self.axis, value), np.zeros(3))

    def clamp(self, value: float) -> float:
        return min(self.upper, max(self.lower, value))


class ArmChain:
    def __init__(self, joints: List[Joint]):
        self.joints = joints
        self.moving = [j for j in joints if j.moving]

    # ------------------------------------------------------------------ construction

    @classmethod
    def from_urdf(cls, path: str = DEFAULT_URDF,
                  root: str = ROOT_LINK, tip: str = TIP_LINK) -> "ArmChain":
        tree = ET.parse(path)
        raw = {}
        parent_of = {}
        for element in tree.getroot().findall("joint"):
            name = element.get("name")
            child = element.find("child").get("link")
            origin = element.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None
                                      else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None
                                      else "0 0 0").split()]
            axis_el = element.find("axis")
            axis = ([float(v) for v in axis_el.get("xyz").split()]
                    if axis_el is not None and axis_el.get("xyz") else None)
            limit = element.find("limit")
            lower = float(limit.get("lower")) if limit is not None and limit.get("lower") else -math.pi
            upper = float(limit.get("upper")) if limit is not None and limit.get("upper") else math.pi
            raw[name] = Joint(
                name=name,
                kind=element.get("type"),
                origin=transform(rpy_to_matrix(*rpy), np.array(xyz)),
                axis=np.array(axis) if axis else None,
                lower=lower,
                upper=upper,
            )
            parent_of[child] = (name, element.find("parent").get("link"))

        chain: List[Joint] = []
        link = tip
        while link != root:
            entry = parent_of.get(link)
            if entry is None:
                raise ValueError(f"chain from {root} to {tip} is broken at {link}")
            jname, parent_link = entry
            chain.append(raw[jname])
            link = parent_link
        chain.reverse()
        return cls(chain)

    # ------------------------------------------------------------------ kinematics

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self.moving]

    @property
    def limits(self) -> List[tuple]:
        return [(j.lower, j.upper) for j in self.moving]

    def fk(self, values: Sequence[float]) -> np.ndarray:
        """4x4 pose of the grasping link in base_link, for the moving-joint values."""
        if len(values) != len(self.moving):
            raise ValueError(f"expected {len(self.moving)} values, got {len(values)}")
        result = np.eye(4)
        index = 0
        for joint in self.joints:
            if joint.moving:
                result = result @ joint.transform_at(float(values[index]))
                index += 1
            else:
                result = result @ joint.origin
        return result

    def position(self, values: Sequence[float]) -> np.ndarray:
        return self.fk(values)[:3, 3]

    def clamp(self, values: Sequence[float]) -> List[float]:
        return [j.clamp(float(v)) for j, v in zip(self.moving, values)]

    def ik(self, target: Sequence[float], seed: Optional[Sequence[float]] = None,
           tolerance: float = 0.005) -> Optional[List[float]]:
        """Joint values placing the gripper at ``target`` (x, y, z) in base_link.

        Position only -- orientation is left free. For reaching into a shelf the approach
        direction matters, but the arm has seven joints for three constraints, so pinning
        orientation as well tends to make the solve fail outright rather than return
        something workable. Orientation is handled by seeding from a posture that already
        points the right way.

        Returns None when no solution reaches within ``tolerance``.
        """
        from scipy.optimize import least_squares

        target = np.asarray(target, dtype=float)
        seed = list(seed) if seed is not None else [
            0.5 * (lo + hi) for lo, hi in self.limits
        ]
        seed = self.clamp(seed)
        lower = [lo for lo, _ in self.limits]
        upper = [hi for _, hi in self.limits]

        def residual(values):
            return self.position(values) - target

        best = None
        # A few restarts: the arm is redundant and the solver can settle in a local
        # minimum that does not reach, particularly near the limits.
        for attempt in range(6):
            start = seed if attempt == 0 else [
                np.random.uniform(lo, hi) for lo, hi in self.limits
            ]
            try:
                result = least_squares(
                    residual, self.clamp(start), bounds=(lower, upper),
                    xtol=1e-10, ftol=1e-10, max_nfev=2000,
                )
            except Exception:  # noqa: BLE001 - a failed attempt is not fatal
                continue
            error = float(np.linalg.norm(result.fun))
            if best is None or error < best[0]:
                best = (error, list(result.x))
            if error <= tolerance:
                break

        if best is None or best[0] > tolerance:
            return None
        return best[1]
