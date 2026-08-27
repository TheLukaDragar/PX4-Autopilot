# Fire mission order
#
# MAVLink-M message FIRES (id 53020).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint64 time_impact_usec # [us] Expected time of impact (UNIX epoch, microseconds UTC).
int32 lat # [degE7] Target latitude, WGS84.
int32 lon # [degE7] Target longitude, WGS84.
float32 alt # [m] Target altitude, MSL.
uint32 effector_id # Identifier of the assigned effector.
uint16 sequence # Fire mission sequence number.
float32 cep_expected # [m] Expected circular error probable at impact.
uint16 prf_code # Laser designator PRF code for SAL weapons (e.g. 1111-1788). 0 = no laser code assigned.
uint8 store_id # Target store/bay on a multi-store carrier. 0 = system default / vehicle itself (e.g. OWA).
uint8 requested_effect # Coarse requested effect (intent), for non-artillery/OWA effectors. For call-for-fire missions use munition_class + fuze_action instead.
uint8 munition_class # Call-for-fire projectile/shell class (method of engagement). UNKNOWN if N/A.
uint8 fuze_mode # Call-for-fire requested fuze mode (method of engagement). UNKNOWN if N/A.
uint8 hob_intent # Airburst height-of-burst intent for proximity/time modes (descriptive selector, not a value).
uint8 fuze_mofa_capable # 1 if the fielded fuze is multi-option (e.g. M782 MOFA) and settable to any mode; 0 otherwise. Capability flag, not a mode.
uint8[16] track_uid # Globally-unique track identifier (UUID) of the TRACK_IDENTITY this message correlates to. Ties the engagement/assessment to the authorized track. All-zero if not associated.
