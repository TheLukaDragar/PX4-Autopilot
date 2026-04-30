# List of currently registered mode executors and their names
# Published by commander when modes are registered/unregistered

uint32 MESSAGE_VERSION = 0

uint64 timestamp # time since system start (microseconds)

uint8 nav_state            # navigation state (NAVIGATION_STATE_*)
uint8 executor_in_charge   # which executor is active (matches VehicleStatus)
char[25] mode_name         # human-readable name for this mode (empty for unregistered modes)
bool valid                 # true if this mode slot is registered
bool not_user_selectable   # true if mode cannot be selected by user
