std_msgs/Header header
float64[] voltage        # applied voltage per active joint [V]
bool[] clipped           # true when the ±15 V limit was hit this step
float64 power            # total electrical power [W]
bool watchdog_tripped
bool estop
