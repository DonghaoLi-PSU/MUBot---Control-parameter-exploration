import math
import random
from types import SimpleNamespace as Obs

import numpy as np
import pytest

from mubot_localization.frontend import FrontEnd
from mubot_localization.occupancy_mapper import OccupancyMapper


def test_keyframe_spacing_and_relative_pose():
    f = FrontEnd(min_translation=0.05, min_rotation=math.radians(10))
    assert f.needs_keyframe((0, 0, 0))
    f.add((0, 0, 0), [])
    assert not f.needs_keyframe((0.03, 0, 0.05))
    assert f.needs_keyframe((0.06, 0, 0))
    assert f.needs_keyframe((0, 0, math.radians(11)))
    rel = FrontEnd.relative((1, 1, math.pi / 2), (1, 2, math.pi / 2))
    assert rel == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)


def test_frontend_drops_far_landmarks():
    f = FrontEnd(max_range=0.8)
    kf = f.add((0, 0, 0), [Obs(id=1, range=0.5), Obs(id=2, range=1.2)])
    assert [o.id for o in kf.observations] == [1]


def test_occupancy_marks_wall_and_free_space():
    m = OccupancyMapper(width=2.0, height=1.0, resolution=0.01, origin=(-1, -0.5))
    for _ in range(5):
        m.integrate((0, 0, 0), (0, 0, 0), [0.0], [0.5])
    p = m.probabilities()
    i_hit, j = m.cell(0.5, 0.0)
    i_free, _ = m.cell(0.25, 0.0)
    assert p[j, i_hit] > 0.9 and p[j, i_free] < 0.2
    data = m.to_ros_data()
    assert len(data) == m.nx * m.ny and data[0] == -1


def test_pose_graph_corrects_drift_with_landmark():
    gtsam = pytest.importorskip('gtsam')
    from mubot_localization.pose_graph import PoseGraph
    rng = random.Random(0)
    lm = (1.0, 0.3)
    g = PoseGraph(odom_sigmas=(0.01, 0.01, math.radians(1)))
    true = [(0.05 * k, 0.0, 0.0) for k in range(21)]
    odom = [(0.0, 0.0, 0.0)]
    for k in range(1, 21):   # odometry over-reports speed by 10 %
        x, y, a = odom[-1]
        odom.append((x + 0.055, y, a))

    def observe(p):
        dx, dy = lm[0] - p[0], lm[1] - p[1]
        return Obs(id=3, bearing=math.atan2(dy, dx) - p[2], range=math.hypot(dx, dy),
                   bearing_std=math.radians(1), range_std=0.005)

    for k in range(21):
        delta = FrontEnd.relative(odom[k - 1], odom[k]) if k else None
        obs = [observe(true[k])] if k in (0, 20) else []
        est = g.add_keyframe(delta, odom[k], obs)
    assert abs(odom[20][0] - true[20][0]) == pytest.approx(0.1, abs=1e-9)
    assert est[0] == pytest.approx(true[20][0], abs=0.02)  # loop closure removes most drift
    assert g.landmark(3) == pytest.approx(lm, abs=0.01)
