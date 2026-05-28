std_msgs/Header header

uint32 id

geometry_msgs/Pose pose
geometry_msgs/Point target_point

float64 yaw_deg
float64 pitch_deg
bool pitch_enabled

float64 view_distance
float64 incidence_angle_deg
float64 coverage_gain
float64 safety_clearance
float64 score

bool is_safe
uint32[] covered_point_indices