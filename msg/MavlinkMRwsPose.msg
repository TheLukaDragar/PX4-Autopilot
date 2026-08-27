# Remote Weapon System (RWS) pose relative to the host vehicle
#
# MAVLink-M message RWS_POSE (id 53040).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
int32 lat # [degE7] RWS latitude, WGS84. INT32_MAX if unknown.
int32 lon # [degE7] RWS longitude, WGS84. INT32_MAX if unknown.
float32 alt # [m] RWS altitude, MSL. NaN if unknown.
float32 vx # [m/s] Velocity north component (NED). NaN if unknown.
float32 vy # [m/s] Velocity east component (NED). NaN if unknown.
float32 vz # [m/s] Velocity down component (NED). NaN if unknown.
float32 cov_pos_x # Position covariance, north axis (m^2). NaN if unknown.
float32 cov_pos_y # Position covariance, east axis (m^2). NaN if unknown.
float32 cov_pos_z # Position covariance, down axis (m^2). NaN if unknown.
float32 cov_vel_x # Velocity covariance, north axis (m^2/s^2). NaN if unknown.
float32 cov_vel_y # Velocity covariance, east axis (m^2/s^2). NaN if unknown.
float32 cov_vel_z # Velocity covariance, down axis (m^2/s^2). NaN if unknown.
float32 offset_x # [m] Mounting offset X from vehicle reference point in coordinate_frame.
float32 offset_y # [m] Mounting offset Y from vehicle reference point in coordinate_frame.
float32 offset_z # [m] Mounting offset Z from vehicle reference point in coordinate_frame.
float32[4] q # RWS orientation quaternion [w, x, y, z] relative to coordinate_frame.
float32 accuracy_roll # [rad] Roll orientation accuracy (1-sigma).
float32 accuracy_pitch # [rad] Pitch orientation accuracy (1-sigma).
float32 accuracy_yaw # [rad] Yaw orientation accuracy (1-sigma).
uint8 coordinate_frame # Coordinate frame for offset and orientation. See MAV_FRAME.
