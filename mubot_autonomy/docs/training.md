# Training the forward gait

```bash
ros2 launch mubot_bringup train.launch.py case:=AN4_Kmed_HM3_AR2 workers:=8
# or without launch:
python -m mubot_learning.trainer --case AN4_Kmed_HM3_AR2 --workers 8
```

- Algorithm: EPHE / reward-weighted EM (`em_pgpe.py`), unchanged from the paper:
  50 rollouts per episode, top 25 update μ and σ, 40 episodes, 3 trials.
- Each worker is a headless gz sim server with its own `GZ_PARTITION`.
- Each rollout: reset world, publish γ, step exactly 24 400 iterations
  (6.1 s at 0.25 ms), reward = mean forward velocity of the centre of gravity
  over the last 2 s.
- Output: `data/train/<case>/trial_<n>/{param,reward,rollout_param}.csv` and
  the best policy in `src/mubot_learning/policies/gamma_star_<case>.yaml`.
- Plot: `python tools/plot_training.py data/train/<case>`.
