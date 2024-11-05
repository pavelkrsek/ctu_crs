#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-11-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from __future__ import annotations
import numpy as np
from numpy import allclose
from numpy.typing import ArrayLike
from ctu_mars_control_unit import MarsControlUnit

from ctu_crs.gripper import Gripper


class CRSRobot:
    def __init__(
        self, tty_dev: str | None = "/dev/mars", baudrate: int = 19200, **crs_kwargs
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

        # DH notation

        self.dh_offset = np.deg2rad(np.array([0.0, -270.0, -90.0, 0.0, 0.0, 0.0]))
        self.dh_d = [
            self.link_lengths[0],
            0,
            0,
            self.link_lengths[2],
            0,
            self.link_lengths[3] + self.gripper_length + self.finger_length,
        ]
        self.dh_a = [0, self.link_lengths[1], 0, 0, 0, 0]
        self.dh_alpha = np.deg2rad(np.array([90.0, 0.0, 270.0, 90.0, 270.0, 0]))

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

    @staticmethod
    def dh_to_se3(d: float, theta: float, a: float, alpha: float) -> np.ndarray:
        """Compute SE3 matrix from DH parameters."""
        tz = np.eye(4)
        tz[2, 3] = d
        rz = np.eye(4)
        rz[:2, :2] = np.array(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
        )
        tx = np.eye(4)
        tx[0, 3] = a
        rx = np.eye(4)
        rx[1:3, 1:3] = np.array(
            [[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]]
        )
        return tz @ rz @ tx @ rx

    def fk_flange_pos(self, q: ArrayLike) -> np.ndarray:
        """Compute forward kinematics for the given joint configuration @param q.
        Returns 3D position of the flange w.r.t. base of the robot."""
        return (self.fk(q) @ np.array([0, 0, -self.dh_d[-1], 1]))[:3]

    def fk(self, q: ArrayLike) -> np.ndarray:
        """Compute forward kinematics for the given joint configuration @param q.
        Returns 4x4 homogeneous transformation matrix (SE3) of the end-effector w.r.t.
        base of the robot."""
        pose = np.eye(4)
        for d, a, alpha, theta, qi in zip(
            self.dh_d, self.dh_a, self.dh_alpha, self.dh_offset, q
        ):
            pose = pose @ self.dh_to_se3(d, qi + theta, a, alpha)
        return pose

    def _ik_flange_pos(
        self, flange_pos: np.ndarray, singularity_theta1=0
    ) -> list[np.ndarray]:
        """Solve IK for position of the flange. This implementation supports only the
        robot configurations that are above the ground."""
        d = self.dh_d
        a = self.dh_a
        b = flange_pos[2] - d[0]

        if allclose(flange_pos[:2], 0):  # last link pointing up
            max_b = d[3] + a[1]
            if allclose(b, max_b):  # full length
                return [np.array([singularity_theta1, 0, 0])]
            if b > d[0]:
                arg1 = (a[1] ** 2 + b**2 - d[3] ** 2) / (2 * a[1] * b)
                arg2 = (a[1] ** 2 + d[3] ** 2 - b**2) / (2 * a[1] * d[3])
                if np.abs(arg1) > 1.0 or np.abs(arg2) > 1.0:
                    return []
                th2 = -np.arccos(arg1)
                th3 = np.pi - np.arccos(arg2)
                return [
                    np.array([singularity_theta1, th2, th3]),
                    np.array([singularity_theta1, -th2, -th3]),
                ]
            return []

        c = np.sqrt(b**2 + flange_pos[0] ** 2 + flange_pos[1] ** 2)
        if allclose(c, d[3] + a[1]):  # full length
            tmp = -np.pi / 2 + np.arcsin(b / c)
            return [
                np.array([np.arctan2(flange_pos[1], flange_pos[0]), tmp, 0]),
                np.array([np.arctan2(-flange_pos[1], -flange_pos[0]), -tmp, 0]),
            ]
        if c >= d[3] + a[1]:
            return []

        theta2_base = (
            np.pi / 2
            - np.arcsin(b / c)
            + np.arccos((a[1] ** 2 + c**2 - d[3] ** 2) / (2 * a[1] * c))
        )
        th2_term1 = np.atan2(np.sin(theta2_base), np.cos(theta2_base))

        theta1_pos = np.arctan2(flange_pos[1], flange_pos[0])
        theta1_neg = np.arctan2(-flange_pos[1], -flange_pos[0])
        theta3_term1 = np.pi - np.arccos(
            (a[1] ** 2 + d[3] ** 2 - c**2) / (2 * a[1] * d[3])
        )

        th2_term2 = (
            -np.pi / 2
            + np.arcsin(b / c)
            + np.arccos((a[1] ** 2 + c**2 - d[3] ** 2) / (2 * a[1] * c))
        )

        return [
            np.array([theta1_pos, -th2_term1, theta3_term1]),
            np.array([theta1_neg, th2_term1, -theta3_term1]),
            np.array([theta1_pos, th2_term2, -theta3_term1]),
            np.array([theta1_neg, -th2_term2, theta3_term1]),
        ]

    def ik(self, pose: np.ndarray) -> list[np.ndarray]:
        """Compute inverse kinematics for the given pose. Returns array of joint
        configurations [rad] which can achieve the given pose."""

        # X=A01*A12*A23 * [0 0 0 1]' because A34*A45*A57==R34*R45*R56 is pure rotation
        flange_pos = pose @ np.array([0, 0, -self.dh_d[5], 1])
        sols_q_03 = self._ik_flange_pos(flange_pos)

        singularity_theta4 = 0

        sols = []
        for q_03 in sols_q_03:
            rot_03 = self.fk(q_03)[:3, :3]
            rot_36 = rot_03.T @ pose[:3, :3]

            # Euler Z - Y Z for joints 4, 5, 6
            P = rot_36
            if np.isclose(P[2][2], 1):  # np.cos(theta5) == 1
                sols.append(
                    np.concatenate(
                        [
                            q_03,
                            [
                                singularity_theta4,
                                0,
                                np.arctan2(P[1][0], P[0][0]) - singularity_theta4,
                            ],
                        ]
                    )
                )
            elif np.isclose(P[2][2], -1):  # np.cos(theta5) == -1
                sols.append(
                    np.concatenate(
                        [
                            q_03,
                            [
                                singularity_theta4,
                                np.pi,
                                np.arctan2(P[1][0], -P[0][0]) + singularity_theta4,
                            ],
                        ]
                    )
                )
            else:  # non - degenerate
                theta5 = np.arccos(P[2][2])
                sols.append(
                    np.concatenate(
                        [
                            q_03,
                            [
                                np.arctan2(
                                    P[1][2] * np.sign(np.sin(theta5)),
                                    P[0][2] * np.sign(np.sin(theta5)),
                                ),
                                -theta5,
                                np.arctan2(
                                    P[2][1] * np.sign(np.sin(theta5)),
                                    -P[2][0] * np.sign(np.sin(theta5)),
                                ),
                            ],
                        ]
                    )
                )
                sols.append(
                    np.concatenate(
                        [
                            q_03,
                            [
                                np.arctan2(
                                    P[1][2] * np.sign(np.sin(-theta5)),
                                    P[0][2] * np.sign(np.sin(-theta5)),
                                ),
                                theta5,
                                np.arctan2(
                                    P[2][1] * np.sign(np.sin(-theta5)),
                                    -P[2][0] * np.sign(np.sin(-theta5)),
                                ),
                            ],
                        ]
                    )
                )
        np.random.shuffle(sols)
        return sols
