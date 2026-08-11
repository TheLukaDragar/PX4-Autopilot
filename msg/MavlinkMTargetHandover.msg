# Transfers tracking responsibility for a target to another system, including kinematic state, clas...
#
# MAVLink-M message TARGET_HANDOVER (id 53002).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
# Same in/out split as TARGET — do not mix peer RX with companion TX on one uORB.
#   mavlink_m_target_handover      — RX → /fmu/out
#   mavlink_m_target_handover_send — /fmu/in → TARGET_HANDOVER stream
#
# TOPICS mavlink_m_target_handover mavlink_m_target_handover_send

uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint64 detected_first_usec # [us] Time target was first detected (UNIX epoch, microseconds UTC).
uint64 valid_until_usec # [us] Handover data validity expiry (UNIX epoch, microseconds UTC).
int32 lat # [degE7] Target latitude, WGS84.
int32 lon # [degE7] Target longitude, WGS84.
float32 alt # [m] Target altitude, MSL.
float32 vx # [m/s] Velocity north component (NED).
float32 vy # [m/s] Velocity east component (NED).
float32 vz # [m/s] Velocity down component (NED).
float32 cov_pos_x # Position covariance, north axis (m^2).
float32 cov_pos_y # Position covariance, east axis (m^2).
float32 cov_pos_z # Position covariance, down axis (m^2).
float32 cov_vel_x # Velocity covariance, north axis (m^2/s^2).
float32 cov_vel_y # Velocity covariance, east axis (m^2/s^2).
float32 cov_vel_z # Velocity covariance, down axis (m^2/s^2).
uint32 target_set_id # Parent target set identifier.
char[50] target_name # Human-readable target name (null-terminated).
char[50] match_media_url # Short URL referencing matching media asset (null-terminated).
float32 confidence_score # Confidence score of identification [0.0-1.0].
uint8[8] authorization # Opaque authorization token.
uint8 target_class # Target classification.
uint8 target_force # Force affiliation.
uint8 match_media_type # Type of matching media referenced by match_media_url.
uint8[16] track_uid # Globally-unique track identifier (UUID) of the TRACK_IDENTITY this handover transfers. The authoritative correlation handle across aircraft; target_name is a human label only. All-zero if not associated with a track.
