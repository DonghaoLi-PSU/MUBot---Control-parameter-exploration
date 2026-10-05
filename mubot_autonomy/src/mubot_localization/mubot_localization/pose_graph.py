"""SE(2) pose graph with bearing-range landmark factors (GTSAM iSAM2)."""
import math

import gtsam
from gtsam.symbol_shorthand import L, X


class PoseGraph:
    def __init__(self, odom_sigmas=(0.01, 0.01, math.radians(2)), prior_sigmas=(1e-3, 1e-3, 1e-3)):
        self.isam = gtsam.ISAM2()
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(list(odom_sigmas))
        self.prior_noise = gtsam.noiseModel.Diagonal.Sigmas(list(prior_sigmas))
        self.estimate = gtsam.Values()
        self.landmarks = set()
        self.n = 0

    def add_keyframe(self, odom_delta, initial_guess, observations):
        """odom_delta: (dx, dy, dyaw) from the previous keyframe, or None for the first."""
        graph, values = gtsam.NonlinearFactorGraph(), gtsam.Values()
        k = self.n
        guess = gtsam.Pose2(*initial_guess)
        if k == 0:
            graph.add(gtsam.PriorFactorPose2(X(0), guess, self.prior_noise))
        else:
            graph.add(gtsam.BetweenFactorPose2(X(k - 1), X(k), gtsam.Pose2(*odom_delta),
                                               self.odom_noise))
        values.insert(X(k), guess)
        for o in observations:
            noise = gtsam.noiseModel.Diagonal.Sigmas([o.bearing_std, o.range_std])
            graph.add(gtsam.BearingRangeFactor2D(X(k), L(o.id), gtsam.Rot2(o.bearing), o.range, noise))
            if o.id not in self.landmarks:
                a = guess.theta() + o.bearing
                values.insert(L(o.id), gtsam.Point2(guess.x() + o.range * math.cos(a),
                                                    guess.y() + o.range * math.sin(a)))
                self.landmarks.add(o.id)
        self.isam.update(graph, values)
        self.estimate = self.isam.calculateEstimate()
        self.n += 1
        return self.pose(k)

    def pose(self, k=None):
        p = self.estimate.atPose2(X(self.n - 1 if k is None else k))
        return p.x(), p.y(), p.theta()

    def landmark(self, marker_id):
        p = self.estimate.atPoint2(L(marker_id))
        return float(p[0]), float(p[1])
