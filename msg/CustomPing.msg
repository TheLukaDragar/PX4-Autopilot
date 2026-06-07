# Custom Ping Message
# This message is received via MAVLink and published to uORB

uint64 timestamp        # time since system start (microseconds)
uint32 sequence         # ping sequence number
float32 data1           # custom data field 1
float32 data2           # custom data field 2
uint8 status            # status flag (0=inactive, 1=active)
