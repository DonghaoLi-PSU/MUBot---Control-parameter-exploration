# Full actuator command for one gait. Sent by gait_generator at 50 Hz and
# converted to gz.msgs.Double_V by mubot_gz_cmd_bridge.
std_msgs/Header header
float64[] amplitude      # e_j [V], one per active joint
float64[] phase          # Psi_j [cycles], phase[0] = 0 (reference joint)
float64 frequency        # f [Hz]
float64 bias             # b [V], added to every active joint
float64 stiffness        # K [N·m/rad] for active joints
float64 tail_ratio       # caudal joint stiffness = tail_ratio * K
bool switch_at_phase     # true: apply at the next phase zero crossing
