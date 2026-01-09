# Speed Settings for Position Mode

1. **Velocity limits** - Software speed caps
2. **Tilt angle limits** - How much the quad can tilt forward
3. **Acceleration limits** - How fast it reaches max speed
4. **Thrust limits** - Motor power available

## Key Parameters to Increase Speed


| Parameter | Default | Default (km/h) | Range | Description |
|-----------|---------|----------------|-------|-------------|
| `MPC_VEL_MANUAL` | 10 m/s | 36 km/h | 3-20 m/s | Max speed in Position mode |
| `MPC_XY_VEL_MAX` | 12 m/s | 43 km/h | 0-20 m/s | Absolute max horizontal speed |
| `MPC_MAN_TILT_MAX` | 35° | - | 1-70° | Max tilt angle in manual modes |
| `MPC_TILTMAX_AIR` | 45° | - | 20-89° | Max tilt angle in flight |
| `MPC_ACC_HOR_MAX` | 5 m/s² | - | 2-15 m/s² | Max acceleration |

**Important relationships:**
- `MPC_XY_VEL_MAX` must always be greater than or equal to `MPC_VEL_MANUAL`
- `MPC_TILTMAX_AIR` should be greater than `MPC_MAN_TILT_MAX` to give the flight controller reserve authority for wind compensation and corrections

### Vertical Speed

| Parameter | Default | Default (km/h) | Range |
|-----------|---------|----------------|-------|
| `MPC_Z_VEL_MAX_UP` | 3 m/s | 11 km/h | 0.5-8 m/s |
| `MPC_Z_VEL_MAX_DN` | 1.5 m/s | 5.4 km/h | 0.5-4 m/s |

## Position Mode Behavior

**Default: `MPC_POS_MODE = 4` (Acceleration based)**


## How to Change Parameters

### Using QGroundControl:
1. Connect to drone
2. **Vehicle Setup** → **Parameters**
3. Search for parameter (e.g., `MPC_VEL_MANUAL`)
4. Change value → **Save**

