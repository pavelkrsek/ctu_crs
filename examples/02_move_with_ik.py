#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np

from ctu_crs.crs97 import CRS97

robot = CRS97()
robot.initialize()

q0 = robot.q_home
current_pose = robot.fk(robot.get_q())
current_pose[:3, 3] += np.array([0.1, 0.1, 0.1])
ik_sols = robot.ik(current_pose)
assert len(ik_sols) > 0
closest_solution = np.argmin([np.linalg.norm(q - q0) for q in ik_sols])
robot.move_to_q(closest_solution)
robot.close()
