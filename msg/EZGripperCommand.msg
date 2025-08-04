byte LEFT=0
byte RIGHT=1

# Specifies the side of the robot of the gripper being referred to
byte robot_side 255

byte POSITION_CONTROL=0
byte CALIBRATION=1
byte ERROR_RESET=2

# Specifies the desired operation mode.
byte operation_mode 255

# The temperature limit. Once reached, the gripper will go into cooldown mode
# until the actuator's temperature falls by 10% of the temperature limit.
# By default set to 75. It is not recommended going above 80.
# Automatic cooldown can be disabled by setting this value to 255.
uint8 temperature_limit 75

# The goal position
# 0.0 = fully closed, 1.0 = fully open
float32 goal_position

# Maximum effort to put into achieving the goal position
# 0.0 = no effort (fingers will not move), 1.0 = maximum effort (can quickly overheat actuator)
# 0.3 is a reasonable normal value, and it is recommended not to exceed 0.8.
float32 max_effort

# Keeping the torque off when not needed can help keep the gripper's temperature down
bool torque_on
