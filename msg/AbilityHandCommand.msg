# The hand's serial number. E.g. 24ABH265
string<=32 identifier

byte POSITION_CONTROL=0
byte VELOCITY_CONTROL=1
byte GRIP_CONTROL=2

# Specifies the control mode (ordinal of AbilityHandControlMode)
# Default = position control
byte control_mode 0

# Goal velocities when executing commands in velocity control mode
# Max velocities when in position or grip control modes
float32[6] goal_velocities

# Goal positions when using POSITION_CONTROL mode
float32[6] goal_positions

byte OPEN_GRIP=0
byte CLOSE_GRIP=1
byte PINCH_GRIP=2
byte FLAT_GRIP=3
byte HOOK_GRIP=4
byte RELAX_GRIP=5
byte DOOR_LEVER_OPEN_GRIP=6
byte DOOR_LEVER_CLOSE_GRIP=7
byte DOOR_LEVER_CRUSH_GRIP=8
byte KEY_OPEN_GRIP=9
byte KEY_CLOSE_GRIP=10

# Grip to execute in GRIP_CONTROL mode
byte grip
