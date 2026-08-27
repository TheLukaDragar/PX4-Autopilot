# Defines a circular target set area in WGS84
#
# MAVLink-M message TARGET_SET_COORD (id 53011).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint32 target_set_id # Unique identifier of the target set.
char[50] target_set_name # Human-readable name of the target set (null-terminated).
int32 lat # [degE7] Centroid latitude, WGS84.
int32 lon # [degE7] Centroid longitude, WGS84.
float32 alt # [m] Centroid altitude, MSL.
float32 radius # [m] Radius of the circular target set area.
uint64 time_start # [us] Area validity start time (UNIX epoch, microseconds UTC). 0 = immediate.
uint64 time_end # [us] Area validity end time (UNIX epoch, microseconds UTC). 0 = indefinite.
