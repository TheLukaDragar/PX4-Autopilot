# Close air support 9-line brief (JP 3-09
#
# MAVLink-M message CAS_9LINE (id 53060).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
int32 ip_lat # [degE7] Line 1: initial point / battle position latitude, WGS84.
int32 ip_lon # [degE7] Line 1: initial point / battle position longitude, WGS84.
uint16 heading_deg # [deg] Line 2: heading (IP to target).
int16 offset_deg # [deg] Line 2: offset (L/R); sign indicates side.
uint32 distance_m # [m] Line 3: distance IP to target.
int32 tgt_elev_m # [m] Line 4: target elevation, MSL.
int32 tgt_lat # [degE7] Line 6: target latitude, WGS84.
int32 tgt_lon # [degE7] Line 6: target longitude, WGS84.
uint8 mark_type # Line 7: type of mark / terminal guidance.
uint8 laser_code_slot # Line 7: 1 = a laser code is assigned (slot); 0 = none. Never the numeric code.
int32 friendlies_lat # [degE7] Line 8: friendlies latitude, WGS84.
int32 friendlies_lon # [degE7] Line 8: friendlies longitude, WGS84.
uint16 egress_deg # [deg] Line 9: egress heading.
uint16 final_attack_heading_deg # [deg] Remarks: final attack heading (FAH). 0xFFFF if unspecified.
uint8 danger_close # Remarks: 1 = danger close / troops in contact; 0 = not.
uint8 delivery # Bomb-on-coordinate vs bomb-on-target.
