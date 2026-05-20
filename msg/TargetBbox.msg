# Target bounding boxes from a camera / companion CV pipeline.
# Coordinates are normalised to the image frame: (0,0) = top-left, (1,1) = bottom-right.
# One message carries all targets for one image frame (atomic update for OSD).

uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_image  # capture timestamp of the source image (microseconds)

uint8 MAX_DETECTIONS = 4
uint8 LABEL_LEN = 8     # bytes per label slot including null terminator

uint8 count             # number of valid entries [0..MAX_DETECTIONS]

float32[4] cx           # normalised centre x [0, 1]
float32[4] cy           # normalised centre y [0, 1]
float32[4] w            # normalised width    [0, 1]
float32[4] h            # normalised height   [0, 1]

# Box and label colour — maps to MSP DisplayPort font page (attr bits 0-1):
#   0 = white   1 = green   2 = red   3 = yellow
# (exact colours depend on VTX/goggles firmware font table)
uint8[4] color

# Flat label buffer: box i uses label_buf[i*8 .. i*8+7], null-padded, max 7 visible chars
char[32] label_buf
