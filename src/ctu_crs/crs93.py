#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-30
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from pathlib import Path

import yaml

from ctu_crs.crs_robot import CRSRobot


class CRS93(CRSRobot):
    def __init__(self, tty_dev: str | None = "/dev/mars", baudrate: int = 19200):
        yaml_path = Path(__file__).parent / "params_crs93.yaml"
        with open(yaml_path, "r") as f:
            crs_params = yaml.safe_load(f)
        super().__init__(tty_dev, baudrate, **crs_params)
