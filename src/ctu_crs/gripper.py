#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-11-7
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import time

import numpy as np
from ctu_mars_control_unit import MarsControlUnit


class Gripper:
    def __init__(self, mars: MarsControlUnit | None = None, bounds=None, axis=None):
        super().__init__()
        self._mars = mars

        # Constants for controlling the gripper
        self._axis = axis  # mars8 axis for gripper

        # Analog input of position sensor
        self._ADC = 10
        # Maximal curent limit (0-255)
        # Value 16 corresponds to 250mA when feedback is 500
        self._current = 16
        # Limitation constant (feeadback from overcurrent)
        self._feedback = 500
        # Maximal energy limits voltage on motor
        # (0 - 32000 corresponds to 0-24V)
        # 2010-02-17 Martin Matousek note:
        # Manual says that gripper can survive up to 15 Volts, but
        # force more than 75# must not be used longer then 15 second. Thus
        # safe power seems to be 11.25 V. We are using more conservative value here
        # 10/32 * 24 = 7.5 V

        self._REGME = 10000
        # Maximal speed
        self._REGMS = 500
        # Axis configuration word
        self._REGCFG = 256
        # PID parameter of controller
        self._REGP = 200
        self._REGI = 0
        self._REGD = 100

        self.gripper_poll_time = 0.2
        self.gripper_poll_diff = 50

        # Gripper range
        self.bounds = bounds

        self._initialized = False

    def initialize(self):
        assert self._mars is not None
        """Initialize the gripper controller by setting the parameters."""
        # Set analog mode of controller
        self._mars.send_cmd(f"ANAXSETUP{self._axis}:{self._ADC},{self._current}\n")
        # Maximal current limit (0 - 255)
        self._mars.send_cmd(f"REGS1{self._axis}:{self._current}\n")
        # Limitation constant (feedback from overcurrent)
        self._mars.send_cmd(f"REGS2{self._axis}:{self._feedback}\n")
        # Maximal energy limits voltage on motor
        self._mars.send_cmd(f"REGME{self._axis}:{self._REGME}\n")
        # Maximal speed
        self._mars.send_cmd(f"REGMS{self._axis}:{self._REGMS}\n")
        # Axis configuration word
        self._mars.send_cmd(f"REGCFG{self._axis}:{self._REGCFG}\n")
        # PID parameters of controller
        self._mars.send_cmd(f"REGP{self._axis}:{self._REGP}\n")
        self._mars.send_cmd(f"REGI{self._axis}:{self._REGI}\n")
        self._mars.send_cmd(f"REGD{self._axis}:{self._REGD}\n")
        self._initialized = True

    def control_position_relative(self, fraction: float):
        """Control the gripper by relative position 0 = bounds[0], 1 = bounds[1]"""
        assert self._initialized, "Gripper controller must be initialized first."
        # Set position on gripper
        target = int(self.bounds[0] + (self.bounds[1] - self.bounds[0]) * fraction)
        self.control_position(target)

    def control_position(self, position: float):
        """Control the gripper by absolute position."""
        assert self._initialized, "Gripper controller must be initialized first."
        self.release()
        self._mars.send_cmd(f"G{self._axis}:{position}\n")
        if np.isclose(position, self.bounds[1]):
            if not self.wait_for_motion_stop():
                print("Cannot wait for motion stop, assuming it is done.")
            self.release()

    def release(self):
        """Release the gripper and reset the control unit."""
        assert self._initialized, "Gripper controller must be initialized first."
        self._mars.send_cmd(f"RELEASE{self._axis}:\n")

    def wait_for_motion_stop(self):
        """Wait until the gripper stops moving."""
        assert self._initialized, "Gripper controller must be initialized first."
        self._mars.send_cmd(f"\nR{self._axis}:\n")
        buf = "\n"
        while True:
            resp = self._mars.read_response()
            if resp is None:
                return False
            buf += resp
            if buf.find("\nFAIL!") >= 0:
                return False
            if buf.find(f"R{self._axis}!") >= 0:
                break

        # R! found
        last = float("inf")
        while True:
            self._mars.send_cmd(f"AP{self._axis}?\n")
            response = self._mars.read_response()
            if response.find("\nFAIL!") >= 0:
                return False
            ifs = response.find(f"AP{self._axis}=")
            if ifs >= 0:
                ifl = response.find("\r\n")
                p = float(response[ifs + 4 : ifl])
                if abs(last - p) < self.gripper_poll_diff:
                    break
                last = p
        time.sleep(self.gripper_poll_time)
        return True
