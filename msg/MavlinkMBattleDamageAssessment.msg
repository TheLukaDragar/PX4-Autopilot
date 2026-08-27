# Reports the assessed destruction level and confidence for an engaged target following a fire mission
#
# MAVLink-M message BATTLE_DAMAGE_ASSESSMENT (id 53022).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
int32 lat # [degE7] Assessed target latitude, WGS84.
int32 lon # [degE7] Assessed target longitude, WGS84.
float32 alt # [m] Assessed target altitude, MSL.
float32 vx # [m/s] Velocity north component (NED). 0 if target is static.
float32 vy # [m/s] Velocity east component (NED). 0 if target is static.
float32 vz # [m/s] Velocity down component (NED). 0 if target is static.
float32 cov_pos_x # Position covariance, north axis (m^2).
float32 cov_pos_y # Position covariance, east axis (m^2).
float32 cov_pos_z # Position covariance, down axis (m^2).
float32 cov_vel_x # Velocity covariance, north axis (m^2/s^2).
float32 cov_vel_y # Velocity covariance, east axis (m^2/s^2).
float32 cov_vel_z # Velocity covariance, down axis (m^2/s^2).
uint32 target_set_id # Parent target set identifier.
char[50] target_name # Human-readable target name (null-terminated).
uint8[8] authorization # Opaque authorization token.
uint8 destruction_pct # [%] Assessed target destruction level [0-100].
uint8 confidence_pct # [%] Confidence of assessment [0-100].
uint8 target_class # Target classification.
uint8 target_force # Force affiliation.
uint8 functional_damage # Discrete functional damage state (joint-aligned), carried alongside destruction_pct.
uint8 physical_damage # Discrete physical damage state (combat-assessment layer).
uint8 reattack_recommended # Reattack recommendation: 1 = yes, 0 = no. Operational output of BDA.
uint8[16] track_uid # Globally-unique track identifier (UUID) of the TRACK_IDENTITY this message correlates to. Ties the engagement/assessment to the authorized track. All-zero if not associated.
