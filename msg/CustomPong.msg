# Custom Pong Message (Response to CustomPing)
# This message is sent from ROS2 -> uORB -> MAVLink

uint64 timestamp        # time since system start (microseconds)
uint32 sequence         # echo of ping sequence number
float32 response_data1  # response data field 1
float32 response_data2  # response data field 2
uint8 response_status   # response status (0=failed, 1=success)
