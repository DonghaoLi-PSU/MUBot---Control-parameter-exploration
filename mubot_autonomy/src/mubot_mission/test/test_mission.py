import math

import numpy as np
import pytest

from mubot_mission.explore import frontier_cells, nearest_frontier
from mubot_mission.goals import bearing_to_point


def test_frontiers_border_unknown():
    g = np.full((5, 5), -1)
    g[1:4, 1:4] = 0
    g[2, 2] = 100
    cells = {tuple(c) for c in frontier_cells(g)}
    assert (1, 1) in cells and (2, 2) not in cells
    assert nearest_frontier(g, (1, 2)) is not None
    assert nearest_frontier(np.zeros((3, 3), int), (1, 1)) is None


def test_bearing_uses_minus_x_forward():
    b, d = bearing_to_point((0, 0, math.pi), (1, 0))   # body −x points to +x in the map
    assert b == pytest.approx(0.0) and d == pytest.approx(1.0)
    b, _ = bearing_to_point((0, 0, math.pi), (0, 1))
    assert b == pytest.approx(math.pi / 2)
