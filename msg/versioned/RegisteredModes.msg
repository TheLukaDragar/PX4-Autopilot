# List of all registered external mode executors and their names
# Published by commander periodically and when modes change

uint32 MESSAGE_VERSION = 0

uint8 MAX_MODES = 8
uint8 MODE_NAME_LENGTH = 25

uint64 timestamp # time since system start (microseconds)

uint8 executor_in_charge   # which executor is currently active (0=Autopilot)

# Array for all 8 external mode slots (EXTERNAL1-8)
uint8[8] nav_state         # navigation state for each slot (NAVIGATION_STATE_EXTERNAL1-8)
char[200] mode_name        # flattened array: 8 modes * 25 chars each (indexed as mode_name[i*25])
bool[8] valid              # true if this mode slot is registered
bool[8] not_user_selectable # true if mode cannot be selected by user
