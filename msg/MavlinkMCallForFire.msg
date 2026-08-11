# Complete call-for-fire method elements (ATP 3-09
#
# MAVLink-M message CALL_FOR_FIRE (id 53024).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint16 sequence # Fire-mission sequence number this CFF applies to.
uint8 mission_type # Warning-order mission type.
uint8 location_method # Method of target location.
uint8 adjustment_type # Area vs precision.
uint8 trajectory # Low/high angle.
uint8 sheaf # Distribution / sheaf.
uint8 control_method # Method of fire and control.
uint8 danger_close # 1 = danger close (target within doctrinal threshold of friendlies); 0 = not. Flag only, no RED table.
uint16 rounds # Number of rounds for fire for effect. 0 = unspecified.
uint16 duration_s # [s] Duration of fire. 0 = unspecified.
uint8 tle_category # Target-location-error category label.
uint8 mensurated # 1 = coordinate mensurated from precision imagery; 0 = map spot.
uint16 sheaf_length_m # [m] Linear-target sheaf length (special sheaf). 0 if N/A.
uint16 sheaf_width_m # [m] Linear-target sheaf width (special sheaf). 0 if N/A.
uint16 sheaf_attitude_cdeg # [cdeg] Linear-target sheaf attitude (special sheaf), centidegrees [0..35999]. 0xFFFF = N/A (0 is a valid attitude, due north).
uint16 ot_direction_cdeg # [cdeg] Virtual observer-target direction (bearing observer/drone to target), centidegrees [0..35999]. Lets the gateway orient VMF K02.22 relative corrections (Add/Drop, L/R) from an absolute-coordinate edge feed. 0xFFFF = not provided (0 is a valid bearing, due north).
uint16 ot_factor_dm # [dm] Observer-target factor: OT range / 1000, in decimeters of shift per mil. Supports relative-correction math at the gateway. 0xFFFF = not provided.
uint8 effect_verb # Mission-effect task verb (suppress/neutralize/destroy/...), descriptive tasking intent. Distinct from mission_type and from the fuze-effect intent carried on FIRES.requested_effect.
uint8[16] track_uid # Globally-unique track identifier (UUID) of the TRACK_IDENTITY this message correlates to. Ties the engagement/assessment to the authorized track. All-zero if not associated.
