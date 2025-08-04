byte LEFT=0
byte RIGHT=1

# Specifies the side of the robot of the gripper being referred to
byte robot_side 255

byte POSITION_CONTROL=0
byte CALIBRATION=1
byte ERROR_RESET=2
byte COOLDOWN=3

# Specifies the current operation mode.
byte operation_mode 255

# Temperature of the Dynamixel in Celsius
uint8 temperature

# The current Dynamixel position
# 0.0 = fully closed, 1.0 = fully open
float32 current_position

# The current amount of effort being used
# 0.0 = no effort, 1.0 = maximum effort
float32 current_effort

# Dynamixel's error code
# See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
uint8 error_code

# Realtime tick of the Dynamixel
# If this value isn't changing, communication with the gripper is broken
int32 realtime_tick

# Whether the hand has been calibrated
bool is_calibrated
