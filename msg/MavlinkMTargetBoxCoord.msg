# Defines a polygonal (quadrilateral) target set area using four WGS84 corner coordinates ordered c...
#
# MAVLink-M message TARGET_BOX_COORD (id 53012).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint32 target_set_id # Unique identifier of the target set.
char[50] target_set_name # Human-readable name of the target set (null-terminated).
int32[4] lat # [degE7] Corner latitudes, WGS84, clockwise order.
int32[4] lon # [degE7] Corner longitudes, WGS84, clockwise order.
float32[4] alt # [m] Corner altitudes, MSL.
uint64 time_start # [us] Area validity start time (UNIX epoch, microseconds UTC). 0 = immediate.
uint64 time_end # [us] Area validity end time (UNIX epoch, microseconds UTC). 0 = indefinite.
