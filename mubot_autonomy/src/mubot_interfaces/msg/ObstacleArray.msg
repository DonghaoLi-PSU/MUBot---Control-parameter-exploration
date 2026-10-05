std_msgs/Header header
Obstacle[] obstacles
float64 front_range      # min range within the front sector [m]
float64 min_ttc          # min TTC over all sectors [s]
float32[] histogram      # polar occupancy histogram, for VFH+ and RViz
float64 histogram_min_angle
float64 histogram_bin_width
