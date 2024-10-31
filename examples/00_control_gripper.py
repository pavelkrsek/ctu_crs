#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-29
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from ctu_crs.crs97 import CRS97

robot = CRS97(tty_dev=None)
robot.initialize()
robot.gripper.control_position_relative(0.5)
robot.close()
