uint8 FORWARD=0
uint8 TURN=1
uint8 BACKWARD=2
uint8 STOP=3
std_msgs/Header header
uint8 mode
float64 bias             # [V]; sign sets turn direction
float64 amplitude_scale  # k in [0, 1], scales every e_j
