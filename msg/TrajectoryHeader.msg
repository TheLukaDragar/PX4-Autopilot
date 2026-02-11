# Trajectory Header Message
# Contains metadata for a trajectory

uint64 timestamp        # time since system start (microseconds)
uint32 trajectory_id    # unique trajectory ID
uint16 total_points     # total number of waypoints
float32 total_duration  # total duration in seconds
uint8 frame_id          # coordinate frame (0=local_origin, 1=global, 2=body)
