# Post-impact adjustment spotting used to correct fire mission accuracy, structured per joint obser...
#
# MAVLink-M message SPLASH_CORRECTION (id 53021).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp of observation (UNIX epoch, microseconds UTC).
int32 lat # [degE7] Observed splash latitude, WGS84.
int32 lon # [degE7] Observed splash longitude, WGS84.
float32 alt # [m] Observed splash altitude, MSL.
uint16 sequence # Fire mission sequence number this correction applies to.
uint8 type_detected # Coarse splash detector cue (implementation-defined; e.g. 0=unknown, 1=smoke, 2=explosion). Doctrinal observed-fire adjustment is carried by deviation, range_spot, hob, impact_surface, and obscuration.
float32 cep_expected # [m] Estimated CEP of the observed splash.
uint8 deviation # Deviation spotting relative to the observer-target line.
uint8 range_spot # Range spotting along the observer-target line.
uint8 hob # Height-of-burst spotting (observation, not munition/fuze setting).
uint8 impact_surface # Supplementary: what was struck / what the splash landed in/on.
uint8 obscuration # Supplementary: cause of obscuration, if observation was masked.
uint8 command # Procedural fire-mission command accompanying this spotting (NONE if pure spotting).
uint8[16] track_uid # Globally-unique track identifier (UUID) of the TRACK_IDENTITY this message correlates to. Ties the engagement/assessment to the authorized track. All-zero if not associated.
