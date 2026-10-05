"""Frontier detection on an occupancy grid (Explore mission)."""
import numpy as np

UNKNOWN, FREE_MAX, OCC_MIN = -1, 25, 65


def frontier_cells(grid: np.ndarray):
    """Free cells next to unknown cells. grid: int array, -1 unknown, 0..100."""
    free = (grid >= 0) & (grid <= FREE_MAX)
    unknown = grid == UNKNOWN
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    return np.argwhere(free & near_unknown)  # rows of (j, i)


def nearest_frontier(grid, robot_cell):
    cells = frontier_cells(grid)
    if len(cells) == 0:
        return None
    d = np.hypot(cells[:, 1] - robot_cell[0], cells[:, 0] - robot_cell[1])
    j, i = cells[int(np.argmin(d))]
    return int(i), int(j)
