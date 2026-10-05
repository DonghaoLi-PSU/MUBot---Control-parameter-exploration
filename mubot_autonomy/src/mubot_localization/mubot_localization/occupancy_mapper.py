"""Log-odds occupancy grid built from range beams at known (optimized) poses."""
import math

import numpy as np


class OccupancyMapper:
    def __init__(self, width=2.4, height=1.4, resolution=0.01, origin=(-1.2, -0.7),
                 l_occ=0.85, l_free=-0.4, l_min=-4.0, l_max=4.0, max_range=1.0):
        self.res = resolution
        self.origin = origin
        self.nx, self.ny = int(round(width / resolution)), int(round(height / resolution))
        self.L = np.zeros((self.ny, self.nx))
        self.l_occ, self.l_free, self.l_min, self.l_max = l_occ, l_free, l_min, l_max
        self.max_range = max_range

    def cell(self, x, y):
        return int((x - self.origin[0]) / self.res), int((y - self.origin[1]) / self.res)

    def _inside(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    def integrate(self, pose, sensor_offset, bearings, ranges):
        """pose: robot (x, y, yaw); sensor_offset: (x, y, yaw) of the sensor in the robot frame."""
        x, y, a = pose
        sx = x + sensor_offset[0] * math.cos(a) - sensor_offset[1] * math.sin(a)
        sy = y + sensor_offset[0] * math.sin(a) + sensor_offset[1] * math.cos(a)
        i0, j0 = self.cell(sx, sy)
        for b, r in zip(bearings, ranges):
            hit = r < self.max_range
            r = min(r, self.max_range)
            ang = a + sensor_offset[2] + b
            i1, j1 = self.cell(sx + r * math.cos(ang), sy + r * math.sin(ang))
            ray = self._bresenham(i0, j0, i1, j1)
            for i, j in ray[:-1]:
                if self._inside(i, j):
                    self.L[j, i] = max(self.l_min, self.L[j, i] + self.l_free)
            i, j = ray[-1]
            if self._inside(i, j):
                d = self.l_occ if hit else self.l_free
                self.L[j, i] = min(self.l_max, max(self.l_min, self.L[j, i] + d))

    def probabilities(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))

    def to_ros_data(self):
        """Values for nav_msgs/OccupancyGrid: -1 unknown, 0..100 otherwise."""
        p = self.probabilities()
        out = np.where(self.L == 0.0, -1, np.round(p * 100)).astype(np.int8)
        return out.flatten().tolist()

    @staticmethod
    def _bresenham(i0, j0, i1, j1):
        cells, di, dj = [], abs(i1 - i0), -abs(j1 - j0)
        si, sj = (1 if i0 < i1 else -1), (1 if j0 < j1 else -1)
        err = di + dj
        while True:
            cells.append((i0, j0))
            if i0 == i1 and j0 == j1:
                return cells
            e2 = 2 * err
            if e2 >= dj:
                err += dj
                i0 += si
            if e2 <= di:
                err += di
                j0 += sj
