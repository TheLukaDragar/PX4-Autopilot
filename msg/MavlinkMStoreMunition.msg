# Descriptive munition inventory/status for a store, for mapping into a joint weapon-status report (e
#
# MAVLink-M message STORE_MUNITION (id 53035).
# Published by: mavlink_receiver (when dialect includes military.xml).
#
uint64 timestamp # [us] Time since system start

uint64 time_usec # [us] Timestamp (UNIX epoch, microseconds UTC).
float32 mass # [kg] Munition mass. NaN if unknown.
char[20] external_track_number # External/joint track or store reference for gateway correlation. Empty if none.
char[30] nomenclature # Munition nomenclature/designation (null-terminated).
uint16 quantity_remaining # Count of this munition remaining available on the store/carrier.
uint8 store_id # Store instance identifier.
uint8 munition_type # Munition type code (implementation-defined; consistent with ESAD_STATE.munition_type).
uint8 guidance # Coarse guidance category.
uint8 warhead # Coarse warhead category.
uint8 arming_status # Safe/armed state (mirrored from ESAD for convenience).
uint8 fuze_health # Coarse fuze HEALTH (functioning/fault), not fuze configuration.
uint8 external_track_type # Namespace of external_track_number.
uint8 munition_family # Composite-munition family (finer than the warhead/guidance pair). Descriptive.
uint8 munition_payload # Composite-munition payload category. Descriptive.
uint8 warhead_mode # Coarse selectable warhead MODE currently set for a selectable/multi-mode warhead (e.g. surface vs penetrate on a multi-drop magazine store). Reports the mode only. It is a COARSE enum and MUST NOT be widened to carry a fuze timing / inter-stage delay value; the actual timing is device-internal and configured on the component bus (e.g. DroneCAN) below the FMU.
uint8 sd_duration # Scatterable-mine self-destruct duration CATEGORY (not exact seconds), for mine payloads.
