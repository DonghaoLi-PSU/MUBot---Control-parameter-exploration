"""VFH+ free-gap selection on a polar obstacle histogram."""
import math
from typing import Optional, Sequence


def select_direction(histogram: Sequence[float], min_angle: float, bin_width: float,
                     goal_angle: float, threshold: float, min_gap_bins: int) -> Optional[float]:
    """Return the bearing of the free gap closest to `goal_angle`, or None if blocked.

    histogram[i] covers [min_angle + i·w, min_angle + (i+1)·w). Bins with a value
    at or above `threshold` are blocked. A gap must span at least `min_gap_bins`.
    Bearings are in the body frame (+ = left).
    """
    free = [h < threshold for h in histogram]
    gaps, start = [], None
    for i, f in enumerate(free + [False]):
        if f and start is None:
            start = i
        elif not f and start is not None:
            if i - start >= min_gap_bins:
                gaps.append((start, i - 1))
            start = None
    if not gaps:
        return None

    def centre(i):
        return min_angle + (i + 0.5) * bin_width

    best, best_cost = None, math.inf
    for a, b in gaps:
        lo, hi = centre(a), centre(b)
        if lo <= goal_angle <= hi and (b - a + 1) >= min_gap_bins:
            # Goal direction is free: keep at least half a gap from the edges.
            margin = (min_gap_bins // 2) * bin_width
            candidate = min(max(goal_angle, lo + margin), hi - margin) if hi - lo >= 2 * margin \
                else (lo + hi) / 2
        else:
            # Steer to the gap edge nearest the goal, offset inward.
            margin = (min_gap_bins // 2) * bin_width
            edge = lo + margin if abs(goal_angle - lo) < abs(goal_angle - hi) else hi - margin
            candidate = min(max(edge, lo), hi)
        cost = abs(candidate - goal_angle)
        if cost < best_cost:
            best, best_cost = candidate, cost
    return best
