# Lean C-UAS track identity (MAVLink TRITRI_TRACK id 53900).
# Published by: mavlink_receiver. Companion TX on same topic via /fmu/in.
#
# TOPICS mavlink_m_tritri_track

uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint64 first_detected_usec # [us] Time the object was first detected (UNIX epoch, microseconds UTC).
uint8[16] track_uid # Globally-unique track identifier (UUID). Stable across the whole chain.
uint32 target_set_id # Associated target set identifier, if any. 0 = none.
float32 id_confidence # Confidence of identification [0.0-1.0]. NaN if not provided.
uint16 atr_model_id # Identifier of the ATR model/version that produced atr_confidence_pct. 0 = unspecified.
uint8 origin_sysid # System ID of the platform that originated/owns this track.
uint8 origin_sensor # Sensor/method underlying the current identification.
uint8 id_method # Method by which identification was reached.
uint8 pid_status # Positive identification status (descriptive).
uint8 target_class # Classification (what it is).
uint8 target_force # Force affiliation.
uint8 stanag_identity # STANAG/APP-6 standard identity.
uint8 environment # STANAG/APP-6 battle dimension / environment.
uint8 atr_confidence_pct # [%] ATR model confidence [0-100]. 255 = N/A.
uint8 atr_conf_tier # OPTIONAL advisory display tier. Not for automated ROE.
uint8 sidc_context # Reality vs exercise vs simulation (SIDC digit 3).
