# Trajectory Waypoint Message
# Single waypoint in a trajectory (matches nav_msgs/Path PoseStamped)

uint64 timestamp        # time since system start (microseconds)
uint32 trajectory_id    # references trajectory header ID
uint16 seq              # waypoint sequence number 0..N-1

uint8 ORB_QUEUE_LENGTH = 50
float32 pos_x           # position X (meters)
float32 pos_y           # position Y (meters)
float32 pos_z           # position Z (meters)
float32 quat_w          # quaternion W component
float32 quat_x          # quaternion X component
float32 quat_y          # quaternion Y component
float32 quat_z          # quaternion Z component
