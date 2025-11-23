# The hand's serial number. E.g. 24ABH265
string<=32 identifier

byte LEFT=0
byte RIGHT=1

# Specifies whether the hand is a left or right hand
byte hand_side 255

# The actuator positions in degrees
float32[6] actuator_positions

# The actuator velocities in degrees/second
float32[6] actuator_velocities

# The actuator currents in milliamps
float32[6] actuator_currents

# The touch sensor pressure readings in Newtons
float32[30] touch_sensor_readings

# If grip control mode, the current grip stage
int32 grip_stage

# The goal positions in degrees
float32[6] goal_positions

# The goal velocities (units unknown, 30 is slow, 300 is really fast)
float32[6] goal_velocities
