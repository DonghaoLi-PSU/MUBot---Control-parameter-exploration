"""Rollout workers.

GzWorker runs one headless gz sim server (its own GZ_PARTITION) and evaluates
gait parameters by stepping an exact number of iterations. AnalyticWorker is
a stand-in with a known optimum, used by tests and for checking the trainer
without Gazebo.
"""
import math
import os
import subprocess

import numpy as np

from mubot_learning.reward import forward_speed_reward


class AnalyticWorker:
    """Smooth toy reward with a known optimum. For tests only."""

    def __init__(self, optimum):
        self.optimum = np.asarray(optimum, dtype=float)

    def evaluate(self, gamma, case):
        d = np.asarray(gamma, dtype=float) - self.optimum
        n = len(d)
        for i in range(1, n - 1, 2):  # phases are circular
            d[i] = (d[i] + 0.5) % 1.0 - 0.5
        return float(math.exp(-0.5 * np.sum((d / 3.0) ** 2)))

    def close(self):
        pass


class GzWorker:
    """One headless gz sim server evaluating rollouts.

    TODO(port): implement with the gz-transport Python bindings
    (`from gz.transport13 import Node`, `from gz.msgs10.double_v_pb2 import Double_V`,
    `world_control_pb2.WorldControl`). Per rollout:
      1. reset the world: WorldControl(reset.all=True) on /world/open_water/control
      2. publish the GaitCmd as Double_V on /model/mubot/gait_cmd
         (packing: see mubot_gazebo/gait_cmd_packing.hpp)
      3. step exactly round(rollout_time / 0.00025) iterations with
         WorldControl(multi_step=N) and wait for the step to finish
      4. collect link velocities (pose/odometry topic), compute the CoG speed,
         and return forward_speed_reward(...)
    """

    STEP = 0.00025

    def __init__(self, index: int, world: str, model_sdf: str):
        self.partition = f'mubot_w{index}'
        env = dict(os.environ, GZ_PARTITION=self.partition)
        self.proc = subprocess.Popen(['gz', 'sim', '-s', '-r', '--iterations', '0', world],
                                     env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        self.model_sdf = model_sdf

    def evaluate(self, gamma, case):
        n_steps = round(case['rollout_time'] / self.STEP)
        raise NotImplementedError(
            f'GzWorker.evaluate: reset, send gamma, step {n_steps} iterations, '
            'then forward_speed_reward(t, v_cog_x, case["reward_window"])')

    def close(self):
        self.proc.terminate()
        self.proc.wait(timeout=10)


_ = forward_speed_reward  # used by GzWorker once implemented
