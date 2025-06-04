#!/usr/bin/env python3
"""
trajectory_control.py
=====================

Point‑to‑point position controller for a quadrotor running PX4 in
**off‑board** mode (ROS 2).

The node implements the *outer* loop of the cascaded architecture:

    trajectory_control  ──►  attitude_control  ──►  mixer
                  (this)       (angle_control.py)     (PX4)

Unlike the earlier demo that made the vehicle fly a circle, this version
accepts an **ordered list of N way‑points** `[(x₀,y₀,z₀), …, (xₙ,yₙ,zₙ)]`
and drives the UAV *sequentially* from one point to the next without any
interpolation.  Each target is considered “reached” once the position
error drops below `wp_tol` (default ⩽ 0.15 m).

The control law is based on the linearised translational dynamics around
hover (Mahony‑Kumar‑Corke eq. (28))

    ẍ =  g θ      ,   θ* =  ẍ*/g
    ÿ = −g φ  ◄──┤   φ* = −ÿ*/g
    ż = 1/m · u₁−g ,   u₁  =  m (ż*+g)

A PD term in Cartesian space generates the commanded acceleration
``a_cmd``; the above mapping turns it into reference roll `φ*`,
pitch `θ*` and collective thrust `u₁` published for the inner attitude
loop.

Author  : Gizmo • updated 28 May 2025
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class TrajectoryController(Node):
    """Outer‑loop *position* controller (point‑to‑point variant).

    Parameters (can be set as ROS 2 params before launch)
    -----------------------------------------------------
    waypoints : List[Tuple[float,float,float]]
        Ordered list of target (x, y, z) positions in **NED** metres.
    wp_tol    : float, default = 0.15
        Euclidean tolerance to declare the current waypoint reached [m].
    Kp        : float[3] or diagonal, default (1.5, 1.5, 2.0)
        Proportional gains for x, y, z errors.
    Kd        : float[3] or diagonal, default (2.0, 2.0, 2.5)
        Derivative gains for vx, vy, vz errors.
    mass      : float, default 1.50
        Vehicle mass [kg].
    g         : float, default 9.81
        Gravitational acceleration [m s⁻²] (positive value).

    Publications
    ------------
    * ``att_thrust_cmd`` (geometry_msgs/Vector3)
        x = φ* (rad), y = θ* (rad), z = u₁ (N)

    Subscriptions
    -------------
    * ``/fmu/out/vehicle_local_position``  (VehicleLocalPosition)
    * ``/fmu/out/vehicle_attitude``        (VehicleAttitude) – yaw cache only
    """

    # ------------------------------------------------------------------
    def __init__(self) -> None:
        super().__init__("trajectory_controller")

        # ~~~~~~~~~~~~~~~~~~~~~ configurable parameters ~~~~~~~~~~~~~~~~~~~~~~
        self.declare_parameter("waypoints",          [])
        self.declare_parameter("wp_tol",             0.15)
        self.declare_parameter("Kp",                 [1.5, 1.5, 2.0])
        self.declare_parameter("Kd",                 [2.0, 2.0, 2.5])
        self.declare_parameter("mass",               2.3)
        self.declare_parameter("g",                  9.81)

        # load parameters ----------------------------------------------------
        wp_list = self.get_parameter("waypoints").get_parameter_value().string_array_value
        if wp_list:
            # ROS param CLI passes a YAML‑style string list; evaluate safely
            self.waypoints: List[Tuple[float, float, float]] = [
                tuple(map(float, p.strip("()[]").split(";"))) for p in wp_list
            ]
        else:
            # fallback demo square (NED frame, z negative = up!)
            self.waypoints = [
                (0.0, 0.0, -2.0),
                (2.0, 0.0, -2.0),
                (2.0, 2.0, -2.0),
                (0.0, 2.0, -2.0),
                (0.0, 0.0, -2.0),
            ]

        self.wp_tol: float = float(self.get_parameter("wp_tol").value)

        Kp_val = self.get_parameter("Kp").value
        Kd_val = self.get_parameter("Kd").value
        self.Kp = np.diag(Kp_val if len(Kp_val) == 3 else [Kp_val]*3)
        self.Kd = np.diag(Kd_val if len(Kd_val) == 3 else [Kd_val]*3)

        self.mass: float = float(self.get_parameter("mass").value)
        self.g:    float = float(self.get_parameter("g").value)

        # ~~~~~~~~~~~~~~~~~~~~~~~~~ internal state ~~~~~~~~~~~~~~~~~~~~~~~~~~~
        self._yaw: float = 0.0                      # [rad] last yaw (unused)
        self._current_wp: int = 0                   # index of active waypoint

        # ~~~~~~~~~~~~~~~~~~~~~~~~ ROS interfaces ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.pose_cb,
            qos_profile,
        )
        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.att_cb,
            qos_profile,
        )
        self.cmd_pub = self.create_publisher(Vector3, "att_thrust_cmd", 10)

        self.get_logger().info(
            f"Loaded {len(self.waypoints)} way‑points, tolerance {self.wp_tol:.2f} m"
        )

    # ==================================================================
    #                             callbacks
    # ==================================================================
    def pose_cb(self, msg: VehicleLocalPosition) -> None:
        """Outer control loop — executed at every local position update."""

        # ---------- current state -------------------------------------
        p = np.array([msg.x, msg.y, msg.z])
        v = np.array([msg.vx, msg.vy, msg.vz])

        # ---------- waypoint management -------------------------------
        target = np.array(self.waypoints[self._current_wp])
        err    = target - p
        if np.linalg.norm(err) <= self.wp_tol and self._current_wp < len(self.waypoints) - 1:
            self._current_wp += 1
            target = np.array(self.waypoints[self._current_wp])
            self.get_logger().info(f"Switched to waypoint #{self._current_wp}: {target}")

        # ---------- desired signals (stationary waypoint) -------------
        p_d = target
        v_d = np.zeros(3)
        a_d = np.zeros(3)

        # ---------- PD in position space ------------------------------
        a_cmd = a_d + self.Kd @ (v_d - v) + self.Kp @ (p_d - p)

        # ---------- map to φ*, θ*, u₁ (small‑angle) --------------------
        theta_cmd =  a_cmd[0] / self.g
        phi_cmd   = -a_cmd[1] / self.g
        u1        =  self.mass * (a_cmd[2] + self.g)

        # ---------- safety saturations --------------------------------
        max_angle = math.radians(20.0)
        phi_cmd   = float(np.clip(phi_cmd,  -max_angle,  max_angle))
        theta_cmd = float(np.clip(theta_cmd, -max_angle,  max_angle))
        u1        = float(max(u1, 0.0))

        # ---------- publish ------------------------------------------
        cmd = Vector3(x=phi_cmd, y=theta_cmd, z=u1)
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    def att_cb(self, msg: VehicleAttitude) -> None:
        """Cache current yaw (ψ) — not used directly in this controller."""
        w, x, y, z = msg.q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    # ==================================================================
    #                               main
    # ==================================================================
    def run(self) -> None:
        rclpy.spin(self)


# ----------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
