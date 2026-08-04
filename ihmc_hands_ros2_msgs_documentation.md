# ihmc_hands_ros2 — Message Documentation

---

- **AbilityHandCommand** — Holds desired control commands for the Ability Hand

    - control_mode
        - Type: byte (default: 0)
        - Description: Specifies the control mode. Constants: POSITION_CONTROL=0, VELOCITY_CONTROL=1, GRIP_CONTROL=2. Defaults to position control.

    - goal_velocities
        - Type: float32[6]
        - Description: Goal velocities for each finger in velocity control mode. Used as maximum velocities in position or grip control modes.

    - goal_positions
        - Type: float32[6]
        - Description: Goal positions for each finger when using POSITION_CONTROL mode.

    - grip
        - Type: byte
        - Description: Grip preset to execute when in GRIP_CONTROL mode. Constants: OPEN_GRIP=0, CLOSE_GRIP=1, PINCH_GRIP=2, FLAT_GRIP=3, HOOK_GRIP=4, RELAX_GRIP=5, DOOR_LEVER_OPEN_GRIP=6, DOOR_LEVER_CLOSE_GRIP=7, DOOR_LEVER_CRUSH_GRIP=8, KEY_OPEN_GRIP=9, KEY_CLOSE_GRIP=10.

---

- **AbilityHandState** — Holds the current state of the Ability Hand

    - actuator_positions
        - Type: float32[6]
        - Description: Current position of each finger actuator in degrees.

    - actuator_velocities
        - Type: float32[6]
        - Description: Current velocity of each finger actuator in degrees per second.

    - actuator_currents
        - Type: float32[6]
        - Description: Current draw of each finger actuator in amps.

    - touch_sensor_readings
        - Type: float32[30]
        - Description: Pressure readings from the 30 touch sensors distributed across the hand, in Newtons.

    - grip_stage
        - Type: int32
        - Description: When in grip control mode, indicates the current stage of the grip execution.

    - goal_positions
        - Type: float32[6]
        - Description: Current goal position for each finger actuator in degrees.

    - goal_velocities
        - Type: float32[6]
        - Description: Current goal or maximum velocity for each finger actuator in degrees per second.

---

- **EZGripperCommand** — Holds desired control commands for the EZGripper

    - operation_mode
        - Type: byte (default: 255)
        - Description: Specifies the desired operation mode. Constants: POSITION_CONTROL=0, CALIBRATION=1, ERROR_RESET=2.

    - temperature_limit
        - Type: uint8 (default: 75)
        - Description: Actuator temperature limit in Celsius. The gripper enters cooldown mode when this threshold is reached, resuming once the temperature drops by 10%. Set to 255 to disable automatic cooldown. Not recommended to exceed 80.

    - goal_position
        - Type: float32
        - Description: Desired gripper position. 0.0 = fully closed, 1.0 = fully open.

    - max_effort
        - Type: float32
        - Description: Maximum effort applied toward achieving the goal position. 0.0 = no effort, 1.0 = maximum effort. A value of 0.3 is recommended for normal use; avoid exceeding 0.8 to prevent overheating.

    - torque_on
        - Type: bool
        - Description: Enables torque output from the actuator. Disabling when not in use helps reduce actuator temperature.

---

- **EZGripperState** — Holds the current state of the EZGripper

    - operation_mode
        - Type: byte (default: 255)
        - Description: Current operation mode of the gripper. Constants: POSITION_CONTROL=0, CALIBRATION=1, ERROR_RESET=2, COOLDOWN=3.

    - temperature
        - Type: uint8
        - Description: Current temperature of the Dynamixel actuator in Celsius.

    - current_position
        - Type: float32
        - Description: Current gripper position. 0.0 = fully closed, 1.0 = fully open.

    - current_effort
        - Type: float32
        - Description: Current effort being applied by the actuator. 0.0 = no effort, 1.0 = maximum effort.

    - error_code
        - Type: uint8
        - Description: Dynamixel error code. See Robotis Dynamixel Protocol 1 documentation for error code definitions.

    - realtime_tick
        - Type: int32
        - Description: Realtime tick counter from the Dynamixel. If this value is not changing, communication with the gripper has been lost.

    - is_calibrated
        - Type: bool
        - Description: True if the gripper has been calibrated.
