#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-11-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike
from ctu_mars_control_unit import MarsControlUnit

from ctu_crs.gripper import Gripper


class CRSRobot:
    def __init__(
        self, tty_dev: str | None = "/dev/ttyUSB0", baudrate: int = 19200, **crs_kwargs
    ):
        super().__init__()
        self._mars = (
            MarsControlUnit(tty_dev=tty_dev, baudrate=baudrate)
            if tty_dev is not None
            else None
        )

        self.link_lengths = np.array([0.3052, 0.3048, 0.3302, 0.0762])
        self.gripper_length = 0.108712
        self.finger_length = 0.0254

        # conversion IRC to radians
        irc = np.array([1000, 1000, 1000, 500, 500, 500])  # IRC per rotation of motor.
        gearing = np.array([100, 100, 100, 101, 100, 101])
        direction = np.array([1, 1, -1, -1, -1, -1])
        self._deg_to_irc = (
            irc * direction * gearing * 4 / 360
        )  # degtoirc=1 degree in IRC

        self._motors_ids = "ABCDEF"

        self._hh_rad = np.array([0, 0, 0, 0, 0, 0])
        self._hh_irc = np.array(crs_kwargs["hh_irc"], dtype=int)

        lower_bound_irc = crs_kwargs["lower_bound_irc"]
        upper_bound_irc = crs_kwargs["upper_bound_irc"]
        self.q_min = self._irc_to_joint_values(lower_bound_irc)
        self.q_max = self._irc_to_joint_values(upper_bound_irc)
        self.q_home = np.deg2rad([0, 0, -45, 0, -45, 0])

        self._default_speed_irc256_per_ms = np.array(
            crs_kwargs["default_speed_irc256_per_ms"], dtype=int
        )
        self._min_speed_irc256_per_ms = np.rint(self._default_speed_irc256_per_ms / 5)
        self._max_speed_irc256_per_ms = np.rint(self._default_speed_irc256_per_ms * 2)

        self._default_acceleration_irc_per_ms = np.array(
            crs_kwargs["default_acceleration_irc_per_ms"], dtype=int
        )
        self._min_acceleration_irc_per_ms = np.rint(
            self._default_acceleration_irc_per_ms / 5
        )
        self._max_acceleration_irc_per_ms = np.rint(
            self._default_acceleration_irc_per_ms * 2
        )

        self.gripper = Gripper(self._mars, **crs_kwargs["gripper"])

        self._REGME = [32000, 32000, 32000, 32000, 32000, 32000]
        self._REGP = [10, 12, 70, 35, 45, 100]
        self._REGI = [80, 63, 50, 80, 65, 300]
        self._REGD = [300, 200, 200, 130, 230, 350]
        self._REGCFG = [1489, 1490, 1490, 1481, 1474, 1490]
        self._IDLEREL = 1200
        self._timeout = 200

        self._initialized = False

    def release(self):
        """Release errors and reset control unit."""
        self._mars.send_cmd("RELEASE:\n")

    def reset_motors(self):
        """Reset motors of robot."""
        self._mars.send_cmd("PURGE:\n")

    def close(self):
        """Close connection to the robot."""
        self._mars.close_connection()

    def initialize(self, home: bool = True):
        """Initialize communication with robot and set all necessary parameters.
        This command will perform following settings:
         - synchronize communication with mars control unit
         - reset motors and wait for them to be ready
         - set PID control parameters, maximum speed and acceleration
         - set value for IDLE release
         - perform hard home and soft home, if @param home is True
        """
        self._mars.sync_cmd_fifo()
        print("Resetting motors")
        self._mars.send_cmd("PURGE:\n")
        self._mars.send_cmd("STOP:\n")
        assert self._mars.check_ready()
        self._mars.wait_ready()

        fields = ["REGME", "REGCFG", "REGP", "REGI", "REGD"]
        for f in fields:
            field_values = getattr(self, f"_{f}")
            assert field_values is not None
            assert len(field_values) == len(self._motors_ids)
            for motor_id, value in zip(self._motors_ids, field_values):
                self._mars.send_cmd(f"{f}{motor_id}:{value}\n")

        self.set_speed(self._default_speed_irc256_per_ms)
        self.set_acceleration(self._default_acceleration_irc_per_ms)

        self._mars.send_cmd(f"IDLEREL:{self._IDLEREL}\n")
        self.gripper.initialize()
        self._mars.send_cmd("SPDTB:0,300\n")

        self._mars.setup_coordmv(self._motors_ids)
        if home:
            self.hard_home()
            self.soft_home()

        self._initialized = True

    def _joint_values_to_irc(self, joint_values: ArrayLike) -> np.ndarray:
        """Convert joint values [rad] to IRC."""
        j = np.asarray(joint_values)
        assert j.shape == (len(self._motors_ids),), "Incorrect number of joints."
        irc = (
            np.rad2deg((joint_values + self._hh_rad)) * self._deg_to_irc + self._hh_irc
        )
        return np.rint(irc)

    def _irc_to_joint_values(self, irc: ArrayLike) -> np.ndarray:
        """Convert IRC to joint values [rad]."""
        irc = np.asarray(irc)
        assert irc.shape == (len(self._motors_ids),), "Incorrect number of joints."
        return np.deg2rad((irc - self._hh_irc) / self._deg_to_irc) + self._hh_rad

    def set_speed(self, speed_irc256_ms: ArrayLike):
        """Set speed for each motor in IRC*256/msec."""
        assert len(speed_irc256_ms) == len(self._motors_ids)
        for axis, speed in zip(self._motors_ids, speed_irc256_ms):
            self._mars.send_cmd(f"REGMS:{axis}:{np.rint(speed)}\n")

    def set_speed_relative(self, fraction: float):
        """Set speed for each motor in fraction (0-1) of maximum speed."""
        assert 0 <= fraction <= 1, "Fraction must be in [0,1]."
        s = self._min_speed_irc256_per_ms + fraction * (
            self._max_speed_irc256_per_ms - self._min_speed_irc256_per_ms
        )
        self.set_speed(s)

    def set_acceleration(self, acceleration_irc_ms: ArrayLike):
        """Set acceleration for each motor in IRC/msec."""
        assert len(acceleration_irc_ms) == len(self._motors_ids)
        for axis, acceleration in zip(self._motors_ids, acceleration_irc_ms):
            self._mars.send_cmd(f"REGACC:{axis}:{np.rint(acceleration)}\n")

    def set_acceleration_relative(self, fraction: float):
        """Set acceleration for each motor in fraction (0-1) of maximum acceleration."""
        assert 0 <= fraction <= 1, "Fraction must be in [0,1]."
        a = self._min_acceleration_irc_per_ms + fraction * (
            self._max_acceleration_irc_per_ms - self._min_acceleration_irc_per_ms
        )
        self.set_acceleration(a)

    def hard_home(self):
        """Perform hard home of the robot s.t. prismatic joint is homed first followed
        by joint A, B, and D. The speed is reset to default value before homing."""
        self.set_speed(self._default_speed_irc256_per_ms)
        self.set_acceleration(self._default_acceleration_irc_per_ms)
        for a in "BACDEF":
            self._mars.send_cmd("HH" + a + ":\n")
            self._mars.wait_ready()

    def soft_home(self):
        """Move robot to the home position using coordinated movement."""
        self._mars.coordmv(self._joint_values_to_irc(self.q_home))
        self.wait_for_motion_stop()

    def move_to_q(self, q: ArrayLike):
        """Move robot to the given joint configuration [rad] using coordinated movement.
        Initialization has be called before to set up coordinate movements."""
        assert self._initialized, "You need to initialize the robot before moving it."
        assert self.in_limits(q), "Joint limits violated."
        self._mars.coordmv(self._joint_values_to_irc(q))

    def get_q(self) -> np.ndarray:
        """Get current joint configuration."""
        return self._irc_to_joint_values(
            self._mars.get_current_q_irc()[: len(self._motors_ids)]
        )

    def in_motion(self) -> bool:
        """Return whether the robot is in motion."""
        return not self._mars.check_ready()

    def wait_for_motion_stop(self):
        """Wait until the robot stops moving."""
        self._mars.wait_ready()

    def in_limits(self, q: ArrayLike) -> bool:
        """Return whether the given joint configuration is in joint limits."""
        return np.all(q >= self.q_min) and np.all(q <= self.q_max)

    # def fk(self, q: ArrayLike) -> tuple[float, float, float, float]:
    #     """Compute forward kinematics for the given joint configuration @param q.
    #     The output is (x,y,z,phi) where x,y,z are position of the end-effector w.r.t.
    #     the base frame and phi is the orientation around z-axis of the end-effector
    #     w.r.t. the base.
    #     """
    #     q = np.asarray(q)
    #     assert q.shape == (len(self._motors_ids),), "Incorrect number of joints."
    #     l1, l2 = self.link_lengths
    #     x = l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1])
    #     y = l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1])
    #     z = self._z_offset + q[2]
    #     phi = np.arctan2(np.sin(q[3]), np.cos(q[3]))
    #     return x, y, z, phi
    #
    # def ik_xyz(
    #     self, x: float, y: float, z: float = 0, q3: float = 0
    # ) -> list[np.ndarray]:
    #     """Compute IK s.t. the end-effector is at the given position w.r.t. the
    #     reference frame. The last joint value is set to the given fixed value. It does
    #     not influence solution of IK, it is just passed to the output.
    #     Internally, :param x and :param y are used to compute first and second
    #     (revolute) joint values. The :param z is used to compute third (prismatic) joint
    #     value. Return all solutions that are in joint limits.
    #     """
    #     sols = []
    #     bs = circle_circle_intersection(
    #         np.zeros(2), self.link_lengths[0], [x, y], self.link_lengths[1]
    #     )
    #     for b in bs:
    #         q = np.zeros(4)
    #         q[0] = np.arctan2(*b[::-1])
    #         rot = np.array(
    #             [[np.cos(q[0]), -np.sin(q[0])], [np.sin(q[0]), np.cos(q[0])]]
    #         )
    #         d = rot.T @ (np.asarray([x, y]) - b)
    #         q[1] = np.arctan2(*d[::-1])
    #         q[2] = z - self._z_offset
    #         q[3] = q3
    #         if self.in_limits(q):
    #             sols.append(q)
    #     return sols
    #
    # def ik(
    #     self, x: float, y: float, z: float = 0.0, phi: float = 0
    # ) -> list[np.ndarray]:
    #     """Compute IK s.t. the end-effector is at the given position w.r.t. the
    #     reference frame. Internally xyz is computed by ik_xyz function for all possible
    #     tool orientation (phi). Return all solutions that are in joint limits.
    #     If no solution exists, return empty list.
    #     """
    #     phi = np.arctan2(np.sin(phi), np.cos(phi))  # normalize to [-pi,pi]
    #     sols = self.ik_xyz(x, y, z, phi)
    #     for k in range(1, int((self.q_max[-1] - self.q_min[-1]) / (2 * np.pi)) + 1):
    #         for plus_minus in [-1, 1]:
    #             q3 = phi + plus_minus * k * 2 * np.pi
    #             if self.q_min[-1] <= q3 <= self.q_max[-1]:
    #                 sols.extend(self.ik_xyz(x, y, z, q3))
    #
    #     return sols
