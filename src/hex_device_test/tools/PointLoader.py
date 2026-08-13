#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2026 taigong26. All rights reserved.
# Author: taigong26 thetaigon@qq.com
# Date  : 2026-07-15
################################################################

import json
from typing import Optional
import numpy as np
import os

class TaskConfigLoader:
    """Load recorded trajectory from trajectory.json and provide config for planners."""

    def __init__(self, config_path=None):
        """
        Args:
            config_path: Path to trajectory.json. If None, use default path.
        """
        if config_path is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(script_dir, '..', '..', 'jsons',
                                       'trajectory.json')

        with open(config_path, 'r') as f:
            data = json.load(f)

        self.info = data['info']

        # Parse points in sequential order
        self.raw_points = []
        for i in range(1, self.info['total_points'] + 1):
            self.raw_points.append(data['point'][str(i)])

        self.joint_waypoints = [p['arm'] for p in self.raw_points]
        self.gripper_type = data['info'].get('gripper_type', '')
        self.grip_waypoints = [p.get('grip', []) for p in self.raw_points]

    def get_waypoints(self):
        return self.joint_waypoints.copy()

    def get_grip_position(self) -> list:
        """Return grip waypoints copy. Empty list if no gripper data."""
        return self.grip_waypoints.copy()

    def has_grip(self) -> bool:
        """Double-check: valid gripper_type AND non-empty grip data."""
        if not self.gripper_type or self.gripper_type == "empty":
            return False
        if not self.grip_waypoints:
            return False
        return len(self.grip_waypoints[0]) > 0

    def get_timestamps(self):
        """Return timestamps in seconds, relative to first point (ts[0]=0.0).

        One timestamp per waypoint, preserving the original time spacing.
        """
        base_ns = self.raw_points[0]['ts_ns']
        return [(p['ts_ns'] - base_ns) * 1e-9 for p in self.raw_points]

