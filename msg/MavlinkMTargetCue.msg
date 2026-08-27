# Non-kinetic point-of-interest cue relayed from a sensing/ISR platform to one or more consumers (e
#
# MAVLink-M message TARGET_CUE (id 53001).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint32 cue_id # Unique identifier of this cue, assigned by the originator.
uint32 target_set_id # Parent target set identifier, if associated. 0 = none.
int32 lat # [degE7] Cue latitude, WGS84.
int32 lon # [degE7] Cue longitude, WGS84.
float32 alt # [m] Cue altitude, MSL. NaN if unknown.
float32 vx # [m/s] Velocity north (NED) if the point is moving. NaN if unknown/static.
float32 vy # [m/s] Velocity east (NED) if the point is moving. NaN if unknown/static.
float32 vz # [m/s] Velocity down (NED) if the point is moving. NaN if unknown/static.
float32 confidence_score # Confidence of the geolocation and of the provisional class/force read [0.0-1.0]. NaN if not provided.
uint8 origin_sysid # MAVLink system ID of the originating (sensing) platform.
uint8 cue_type # Intent of the cue. Non-kinetic.
uint8 target_class # Provisional target classification from the sensing platform, qualified by confidence_score. A hint, not an identification.
uint8 target_force # Provisional force affiliation from the sensing platform, qualified by confidence_score. A hint, not a combat-ID determination; do not use as a release input.
char[20] name # Short human-readable label (null-terminated).
