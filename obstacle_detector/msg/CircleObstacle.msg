geometry_msgs/Point center      # Central point [m]
geometry_msgs/Vector3 velocity  # Linear velocity [m/s]
float64 radius                  # Radius with added margin [m]
float64 true_radius             # True measured radius [m]
string id
float64 direction

int64 state
int64 LCSS_state
int64 distance
string action
int64[] state_list
int64[] direction_list
int64 prev_direction
##int32 possibility