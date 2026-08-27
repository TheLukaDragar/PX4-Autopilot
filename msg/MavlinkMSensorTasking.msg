# Inbound sensor/payload tasking from a UCI C2 node, and the platform's tasking-status response
#
# MAVLink-M message SENSOR_TASKING (id 53050).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint16 task_id # Tasking correlation id (echoed in the status response).
uint8 sensor_mode # Commanded sensor/payload mode.
uint8 status # Tasking-status response (ACK/REJECTED/COMPLETE/ACTIVE).
int32 poi_lat # [degE7] Point-of-interest latitude to slew to, WGS84. INT32_MAX = not provided (0 is a valid coordinate).
int32 poi_lon # [degE7] Point-of-interest longitude to slew to, WGS84. INT32_MAX = not provided (0 is a valid coordinate).
float32 poi_alt # [m] Point-of-interest altitude, MSL. NaN = not provided.
