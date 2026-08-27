# Engagement-control directive making abort, check-fire, resume, and re-target first-class actions ...
#
# MAVLink-M message ENGAGEMENT_DIRECTIVE (id 53023).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint8[16] track_uid # Track UID the directive applies to. All-zero if addressed by sequence only.
uint16 sequence # Fire mission sequence number the directive applies to.
uint32 effector_id # Effector the directive is addressed to. 0 = all assigned.
int32 retarget_lat # [degE7] New target latitude for RETARGET, WGS84. INT32_MAX = not provided (0,0 is a valid coordinate). Ignored unless directive is RETARGET.
int32 retarget_lon # [degE7] New target longitude for RETARGET, WGS84. INT32_MAX = not provided. Ignored unless directive is RETARGET.
float32 retarget_alt # [m] New target altitude for RETARGET, MSL. NaN = not provided. Ignored unless directive is RETARGET.
uint8 directive # Directive type.
uint8 origin_sysid # System ID issuing the directive.
