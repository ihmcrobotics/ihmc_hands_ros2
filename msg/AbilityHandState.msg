# The actuator positions in degrees
float32[6] actuator_positions

# The current finger velocities (deg/sec)
float32[6] actuator_velocities

# The actuator currents in amps
float32[6] actuator_currents

# The touch sensor pressure readings in Newtons
float32[30] touch_sensor_readings

# If grip control mode, the current grip stage
int32 grip_stage

# The goal positions in degrees
float32[6] goal_positions

# The goal or max velocities (deg/sec)
float32[6] goal_velocities
