# !/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from ctu_crs.crs93 import CRS93
import numpy as np
import unittest


class TestKinematics(unittest.TestCase):

    def test_fk_zeros(self):
        r = CRS93(tty_dev=None)
        pose = r.fk([0, 0, 0, 0, 0, 0])
        exp_translation = [
            0.0,
            0.0,
            np.sum(r.link_lengths) + r.gripper_length + r.finger_length,
        ]
        np.testing.assert_allclose(pose[:3, 3], exp_translation, atol=1e-6)
        np.testing.assert_allclose(pose[:3, :3], np.eye(3), atol=1e-6)

    def test_ik_xyz_flange_on_z_axis(self):
        np.random.seed(0)
        r = CRS93(tty_dev=None)
        for i in range(100):
            q = np.zeros(6)
            if i < 50:
                h = np.random.uniform(0.3, 0.6)
            else:
                h = np.random.uniform(-1, 1)
            sols = r._ik_flange_pos(np.array([0, 0, h]))
            for s in sols:
                q[:3] = s
                pose = r.fk(q) @ np.array([0, 0, -r.dh_d[-1], 1])
                np.testing.assert_allclose(pose[:3], [0, 0, h], atol=1e-6)

    def test_ik_xyz_flange_straight(self):
        np.random.seed(0)
        r = CRS93(tty_dev=None)
        for _ in range(100):
            q = np.random.uniform(r.q_min, r.q_max)
            q[2] = 0
            flange_pos = r.fk_flange_pos(q)
            sols = r._ik_flange_pos(flange_pos)
            self.assertTrue(any([np.allclose(q[:3], s, atol=1e-6) for s in sols]))
            for s in sols:
                q[:3] = s
                np.testing.assert_allclose(r.fk_flange_pos(q), flange_pos, atol=1e-6)

    def test_ik_xyz_flange_all(self):
        np.random.seed(0)
        r = CRS93(tty_dev=None)
        for _ in range(100):
            q = np.random.uniform(r.q_min, r.q_max)
            flange_pos = r.fk_flange_pos(q)
            sols = r._ik_flange_pos(flange_pos)
            self.assertTrue(any([np.allclose(q[:3], s, atol=1e-6) for s in sols]))
            for s in sols:
                q[:3] = s
                np.testing.assert_allclose(r.fk_flange_pos(q), flange_pos, atol=1e-6)

    def test_ik(self):
        np.random.seed(0)
        r = CRS93(tty_dev=None)
        for i in range(100):
            q = np.random.uniform(r.q_min, r.q_max)
            pose = r.fk(q)
            sols = r.ik(pose)
            for c in sols:
                np.testing.assert_allclose(r.fk(c), pose, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
