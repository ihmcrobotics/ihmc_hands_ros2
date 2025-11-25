# The hand's serial number. E.g. 24ABH265
string<=32 identifier

byte LEFT=0
byte RIGHT=1

# Specifies whether the hand is a left or right hand
byte hand_side 255

# The actuator positions in degrees
float32[6] actuator_positions

# The actuator rotor velocities in radians/second,
# To convert to degress/second:
#   gear_ratio * rotor_velocity * (180 / pi)
# Gear ratios:
# Index, Middle, Ring, Pinky, Thumb Flexor: 649
# Thumb Rotator: 162.45
float32[6] actuator_velocities

# The actuator currents in amps
float32[6] actuator_currents

# The touch sensor pressure readings in Newtons
float32[30] touch_sensor_readings

# If grip control mode, the current grip stage
int32 grip_stage

# The goal positions in degrees
float32[6] goal_positions

# The goal velocities (units unknown, 30 is slow, 300 is really fast)
float32[6] goal_velocities
