#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np
from ctu_crs import CRS97

robot = CRS97()
robot.initialize()

q0 = robot.q_home
for i in range(len(q0)):
    q = q0.copy()
    q[i] += np.deg2rad(10)
    robot.move_to_q(q)

robot.close()
