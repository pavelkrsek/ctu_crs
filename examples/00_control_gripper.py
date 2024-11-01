#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-29
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>

from ctu_crs import CRS97

robot = CRS97()
robot.initialize()
robot.gripper.control_position_relative(0.5)
robot.gripper.control_position(robot.gripper.bounds[0])
robot.close()
