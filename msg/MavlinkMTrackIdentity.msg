# Correlation spine for the kill chain
#
# MAVLink-M message TRACK_IDENTITY (id 53000).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
uint8[16] track_uid # Globally-unique track identifier (UUID). Stable across the whole chain.
uint8[16] parent_track_uid # Parent track UID for split/merge/re-id lineage. All-zero if none.
uint32 target_set_id # Associated target set identifier, if any. 0 = none.
uint64 first_detected_usec # [us] Time the object was first detected (UNIX epoch, microseconds UTC).
float32 id_confidence # Confidence of identification [0.0-1.0]. NaN if not provided.
uint8 origin_sysid # System ID of the platform that originated/owns this track.
uint8 origin_sensor # Sensor/method underlying the current identification.
uint8 id_method # Method by which identification was reached.
uint8 pid_status # Positive identification status (descriptive).
uint8 track_rel # Relationship to parent track (split/merge/re-id).
uint8 target_class # Classification (what it is), distinct from id_method (how it was identified).
uint8 target_force # Force affiliation.
char[50] id_basis # Optional free-text basis/notes for the identification (null-terminated).
char[20] external_track_number # External/joint track number (e.g. Link 16 track number or VMF URN), as text for gateway round-trip. Empty if none. Encoding per external_track_type.
uint8 external_track_type # Namespace of external_track_number.
uint8 stanag_identity # STANAG/APP-6 standard identity (richer than target_force).
uint8 environment # STANAG/APP-6 battle dimension / environment.
uint8 atr_confidence_pct # [%] ATR model confidence [0-100]. 255 = N/A. Raw score; the C2 node owns any ROE threshold.
uint16 atr_model_id # Identifier of the ATR model/version that produced atr_confidence_pct. 0 = unspecified.
uint8 atr_conf_tier # OPTIONAL advisory display tier derived from atr_confidence_pct. Not for automated ROE.
uint8 sidc_context # Reality context (MIL-STD-2525D SIDC digit 3): real-world vs exercise vs simulation. Prevents a live track and an exercise/simulated track from being indistinguishable on a joint net.
