std_msgs/Header header
uint8 mode               # Primitive mode constants
float64 phase            # current gait phase, 0..1
float64 frequency        # [Hz]
float64 amplitude_scale  # k, 0..1
float64 bias             # [V], as currently applied (after ramping)
