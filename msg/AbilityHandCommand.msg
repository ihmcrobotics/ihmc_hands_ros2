# The hand's serial number. E.g. 24ABH265
string<=8 identifier

byte POSITION_CONTROL=0
byte VELOCITY_CONTROL=1
byte VEL_TO_POS_CONTROL=2
byte GRIP_CONTROL=3

# Specifies the control mode
# Default = position control
byte control_mode 0

byte POWER_GRIP=0
byte KEY_GRIP=1
byte TRIPOD_OPEN_GRIP=2
byte TRIPOD_CLOSED_GRIP=3
byte RELAX_GRIP=3
byte RUDE_GRIP=4
byte HOOK_GRIP=5
byte PINCH_OPEN_GRIP=6
byte PINCH_CLOSED_GRIP=7

byte grip

float32[6] goal_positions
float32[6] goal_velocities
