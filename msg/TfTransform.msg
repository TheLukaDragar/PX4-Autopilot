# TF Transform Message
# Transform between coordinate frames streamed over MAVLink

uint64 timestamp        # time since system start (microseconds)
uint8 frame_id          # parent frame ID (0=local_origin, 1=px4_body, 2=camera_link, etc.)
uint8 child_frame_id    # child frame ID
float32 translation_x   # translation X in meters
float32 translation_y   # translation Y in meters
float32 translation_z   # translation Z in meters
float32 rotation_x      # quaternion X component
float32 rotation_y      # quaternion Y component
float32 rotation_z      # quaternion Z component
float32 rotation_w      # quaternion W component
