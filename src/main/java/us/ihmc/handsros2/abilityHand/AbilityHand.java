package us.ihmc.handsros2.abilityHand;

import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;
import us.ihmc.handsros2.YoFloatArray;
import us.ihmc.handsros2.YoIntegerArray;
import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.handsros2.TrapezoidalStep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import static us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName.*;

/**
 * A YoVariable-ized controller for the Ability hand.
 * Can be useful in some applications, though it may be necessary to create
 * a custom implementation for other applications.
 * <p>
 * The hand's finger actuators are specified using the following indices:
 *    <ol start = 0>
 *       <li>Index finger</li>
 *       <li>Middle finger</li>
 *       <li>Ring finger</li>
 *       <li>Pinky finger</li>
 *       <li>Thumb flexor</li>
 *       <li>Thumb rotator</li>
 *    </ol>
 * </p>
 * <p>
 * Following the above method, the hand's fingers are specified using indices [0, 4],
 * where 0 = index finger, 1 = middle finger, and so on for the ring, pinky, and thumb fingers.
 * </p>
 * <p>
 * The hand reports actuator velocities as radians per second.
 * </p>
 * <p>
 * Ability Hands can be optionally equipped with touch sensors.
 * Force sensitive resistors (FSRs) are used to sense pressure exerted on the fingers.
 * Each finger will have 6 sensors, making 30 total.
 * The sensors are indexed following the order of finger indices. I.e. sensors [0, 5] are index finger sensors,
 * [6, 11] are middle finger sensors, and so on with the thumb sensors being last.
 * </p>
 * <p>
 * For more information on the Ability Hands, you can read the
 * <a href="https://github.com/psyonicinc/ability-hand-api/blob/master/Documentation/ABILITY-HAND-ICD.pdf">Ability Hand Interface Control Document</a>.
 * </p>
 *
 * <p>
 * Additionally, this class manages higher-level control of an Ability Hand, including position, velocity,
 * velocity-to-position, and multi-stage grip operations, with YoVariables.
 * </p>
 */
public class AbilityHand implements HandInterface
{
   public static int ACTUATOR_COUNT = 6;
   public static int TOUCH_SENSOR_COUNT = 30;

   private static final float TOLERANCE = 1.0f;
   /** Trajectory configuration: tune acceleration per joint as needed (deg/s^2) */
   private static final float DEFAULT_MAXIMUM_ACCELERATION = 200.0f;

   private final String identifier;
   private final RobotSide handSide;

   /** Command type used for low-level interface. */
   private final YoEnum<AbilityHandCommandType> commandType;

   /** Low-level command values sent to the hand (position or velocity depending on {@link #commandType}). */
   private final YoFloatArray commandValues;
   /** Filtered command values. */
   private final YoFloatArray filteredCommandValues;
   /** Measured actuator positions in degrees. */
   private final YoFloatArray actuatorPositions;
   /** Measured actuator velocities in radians per second. */
   private final YoFloatArray actuatorVelocities;
   /** Filtered actuator velocities in radians per second. */
   private final YoFloatArray filteredActuatorVelocities;
   /** Measured actuator currents in amperes. */
   private final YoFloatArray actuatorCurrents;
   /** Raw touch sensor FSR readings (ADC counts). */
   private final YoIntegerArray rawFSRReadings;

   /** High-level control mode (POSITION, VELOCITY, GRIP). */
   private final YoEnum<AbilityHandControlMode> controlMode;
   private AbilityHandControlMode previousControlMode = null;

   /** High-level multi-stage grip pattern. */
   private final YoEnum<AbilityHandGrip> grip;
   private AbilityHandGrip previousGrip = null;
   private int gripStage = Integer.MAX_VALUE;

   /** Goal positions per actuator, used by POSITION, VEL_TO_POS, and GRIP modes. */
   private final YoFloatArray goalPositions;
   /** Goal velocities per actuator, used as maximum velocities in trajectories and velocity mode. */
   private final YoFloatArray goalVelocities;

   /** Track previous time for computing dt. */
   private long previousControlTimeNanos = -1L;
   private final YoDouble controlDT;
   private long previousFilterTimeNanos = -1L;
   private final YoDouble filterDT;

   /** Previous filtered actuator velocities for low-pass filter */
   private final float[] previousFilteredActuatorVelocities = new float[ACTUATOR_COUNT];
   /** Previous filtered command values for low-pass filter */
   private final float[] previousFilteredCommandValues = new float[ACTUATOR_COUNT];

   /**
    * Creates a new AbilityHand with its own {@link YoRegistry}.
    *
    * @param identifier user-facing identifier of this hand
    * @param handSide   side of the robot this hand is attached to
    */
   public AbilityHand(String identifier, RobotSide handSide)
   {
      this(new YoRegistry("AbilityHand_" + identifier + "_" + handSide.name()), identifier, handSide);
   }

   /**
    * Creates a new AbilityHand with YoVariables for state, commands, goals, and modes.
    *
    * @param registry   YoRegistry to register YoVariables into
    * @param identifier user-facing identifier of this hand
    * @param handSide   side of the robot this hand is attached to
    */
   public AbilityHand(YoRegistry registry, String identifier, RobotSide handSide)
   {
      this.identifier = identifier;
      this.handSide = handSide;

      String prefix = handSide.name() + "AbilityHand_" + identifier + "_";

      // Low-level command type
      commandType = new YoEnum<>(prefix + "CommandType", registry, AbilityHandCommandType.class);

      // Low-level arrays (start at zero)
      commandValues = new YoFloatArray(prefix + "Command", registry, 0, 0, 0, 0, 0, 0);
      filteredCommandValues = new YoFloatArray(prefix + "FilteredCommand", registry, 0, 0, 0, 0, 0, 0);
      actuatorPositions = new YoFloatArray(prefix + "ActuatorPosition", registry, 0, 0, 0, 0, 0, 0);
      actuatorVelocities = new YoFloatArray(prefix + "ActuatorVelocity", registry, 0, 0, 0, 0, 0, 0);
      filteredActuatorVelocities = new YoFloatArray(prefix + "FilteredActuatorVelocity", registry, 0, 0, 0, 0, 0, 0);
      actuatorCurrents = new YoFloatArray(prefix + "ActuatorCurrent", registry, 0, 0, 0, 0, 0, 0);

      int[] fsrInitial = new int[TOUCH_SENSOR_COUNT];
      rawFSRReadings = new YoIntegerArray(prefix + "RawFSR", registry, fsrInitial);

      // High-level control variables
      String managerPrefix = handSide.name() + getClass().getSimpleName();
      controlMode = new YoEnum<>(managerPrefix + "ControlMode", registry, AbilityHandControlMode.class);
      controlMode.set(AbilityHandControlMode.VELOCITY);
      grip = new YoEnum<>(managerPrefix + "Grip", registry, AbilityHandGrip.class);

      // Initialize goal positions and velocities with previous defaults
      goalPositions = new YoFloatArray(managerPrefix + "GoalPosition", registry, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, -30.0f);
      goalVelocities = new YoFloatArray(managerPrefix + "GoalVelocity", registry, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      controlDT = new YoDouble(managerPrefix + "_controlDT", registry);
      filterDT = new YoDouble(managerPrefix + "_filterDT", registry);
   }

   public void updateFilters()
   {
      long nowNanos = System.nanoTime();

      if (previousFilterTimeNanos > 0L)
      {
         long deltaNanos = nowNanos - previousFilterTimeNanos;
         filterDT.set(deltaNanos * 1.0e-9f);

         for (int i = 0; i < ACTUATOR_COUNT; i++)
         {
            {
               float breakFrequency = 0.2f;
               float tau = 1.0f / (2.0f * (float) Math.PI * breakFrequency);
               float alpha = (float) filterDT.getValue() / (tau + (float) filterDT.getValue());
               float rawVelocity = getActuatorVelocity(i);
               float filteredVelocity = alpha * rawVelocity + (1.0f - alpha) * previousFilteredActuatorVelocities[i];
               previousFilteredActuatorVelocities[i] = filteredVelocity;
               filteredActuatorVelocities.set(i, filteredVelocity);
            }

            {
               float breakFrequency = 0.3f;
               float tau = 1.0f / (2.0f * (float) Math.PI * breakFrequency);
               float alpha = (float) filterDT.getValue() / (tau + (float) filterDT.getValue());
               float rawCommand = getCommandValue(i);
               float filteredCommand = alpha * rawCommand + (1.0f - alpha) * previousFilteredCommandValues[i];
               previousFilteredCommandValues[i] = filteredCommand;
               filteredCommandValues.set(i, filteredCommand);
            }
         }
      }

      previousFilterTimeNanos = nowNanos;
   }

   @Override
   public void update()
   {

   }

   /**
    * Updates the hand commands based on the desired values set in this manager.
    * Should be called periodically.
    */
   public void updateControl()
   {
      long nowNanos = System.nanoTime();

      if (previousControlTimeNanos > 0L)
      {
         long deltaNanos = nowNanos - previousControlTimeNanos;
         update(deltaNanos * 1.0e-9f);
      }

      previousControlTimeNanos = nowNanos;
   }

   /**
    * Need to supply virtual dt in tests.
    *
    * @param dt time step in seconds
    */
   void update(float dt)
   {
      controlDT.set(dt);

      AbilityHandControlMode currentControlMode = controlMode.getValue();

      if (currentControlMode != null)
      {
         switch (currentControlMode)
         {
            case POSITION -> updatePositionControl();
            case VELOCITY -> updateVelocityControl();
            case GRIP -> updateGripControl();
         }
      }

      previousControlMode = currentControlMode;
   }

   /**
    * Updates hand in POSITION mode using per-finger trapezoidal steps.
    * goalPositions are the targets; goalVelocities set each finger's max velocity.
    */
   private void updatePositionControl()
   {
      setCommandType(AbilityHandCommandType.POSITION);

      for (int actuatorIndex = 0; actuatorIndex < ACTUATOR_COUNT; actuatorIndex++)
      {
         float commandedPosition = step(actuatorIndex, goalPositions.get(actuatorIndex));
         setCommandValue(actuatorIndex, commandedPosition);
      }
   }

   /**
    * Updates hand to direct velocity control using goalVelocities.
    */
   private void updateVelocityControl()
   {
      setCommandType(AbilityHandCommandType.VELOCITY);
      setCommandValues(goalVelocities.toFloatArray());
   }

   /**
    * Performs multi-stage grip control, moving fingers sequentially through {@link AbilityHandGrip#stages}.
    */
   private void updateGripControl()
   {
      AbilityHandGrip currentGrip = grip.getValue();

      // Check if we're starting a new grip
      if (previousControlMode != AbilityHandControlMode.GRIP || previousGrip != currentGrip)
      {
         gripStage = 0;
         previousGrip = currentGrip;
      }

      // If we're past the last stage, the grip is completed. No need to do anything
      if (currentGrip == null || gripStage >= currentGrip.stages.length)
         return;

      setCommandType(AbilityHandCommandType.POSITION);

      // Normal grip stages: move only the fingers in this stage toward their stage goals,
      // but update all trajectories (steps) every tick.
      int[] actuatorsToMove = currentGrip.stages[gripStage];
      float[] stageGoalPositions = currentGrip.positions[gripStage];

      boolean stageComplete = true;

      for (int actuatorIndex = 0; actuatorIndex < ACTUATOR_COUNT; actuatorIndex++)
      {
         boolean isActive = false;
         float desiredPosition = 0.0f;

         // Check if this finger is active in the current stage
         for (int i = 0; i < actuatorsToMove.length; i++)
         {
            if (actuatorIndex == actuatorsToMove[i])
            {
               isActive = true;
               desiredPosition = stageGoalPositions[i];
               break;
            }
         }

         float currentCommand = getCommandValue(actuatorIndex);
         float targetForThisFinger;

         if (isActive)
         {
            targetForThisFinger = desiredPosition;
         }
         else
         {
            // Not active: hold current command position as its goal
            targetForThisFinger = currentCommand;
         }

         float commandedPosition = step(actuatorIndex, targetForThisFinger);
         setCommandValue(actuatorIndex, commandedPosition);

         if (isActive && Math.abs(commandedPosition - desiredPosition) >= TOLERANCE)
            stageComplete = false;
      }

      if (stageComplete)
      {
         // Snap active fingers to exact stage goal to avoid tiny residuals
         for (int i = 0; i < actuatorsToMove.length; i++)
         {
            int actuatorIndex = actuatorsToMove[i];
            float stageGoalPosition = stageGoalPositions[i];
            setCommandValue(actuatorIndex, stageGoalPosition);
         }

         gripStage++;
      }
   }

   private float stepCurrent(int actuatorIndex, float targetPosition)
   {
      float currentPosition = actuatorPositions.get(actuatorIndex);
      float positionError = targetPosition - currentPosition;

      if (Math.abs(positionError) < 2.0f)
         return 0.0f;

      float direction = Math.signum(positionError);

      return 0.05f * direction;
   }

   private float step(int actuatorIndex, float targetPosition)
   {
      float currentPosition = actuatorPositions.get(actuatorIndex);

      float dt = (float) controlDT.getValue();
      float positionError = targetPosition - currentPosition;
      float direction = Math.signum(positionError);
      float step = currentPosition + direction * goalVelocities.get(actuatorIndex) * dt;

      // Clamp to target position if close
      if (Math.abs(positionError) < 2.0f)
         step = targetPosition;

      // Because the Ability hand uses a signed int 16 internal representation,
      // any commanded position has to be at least a 0.005 increment or it won't move.
      // Math:
      // 150 deg / 32767 (int16 max) = 0.00458 minimum command delta
      float minDelta = 0.1f;
      if (step != currentPosition && Math.abs(step - currentPosition) < minDelta)
         step = currentPosition + minDelta * Math.signum(step - currentPosition);
      return step;
   }

   /**
    * Get the hand object identifier.
    *
    * @return The identifier of this hand.
    */
   @Override
   public String getIdentifier()
   {
      return identifier;
   }

   /**
    * Get the robot side this hand is attached to.
    *
    * @return the robot side
    */
   @Override
   public RobotSide getSide()
   {
      return handSide;
   }

   /**
    * Get the type of this hand.
    *
    * @return {@link HandType#ABILITY_HAND}
    */
   public HandType getType()
   {
      return HandType.ABILITY_HAND;
   }

   /**
    * Sets the control mode for subsequent updates.
    *
    * @param controlMode desired control mode
    */
   public void setControlMode(AbilityHandControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }

   /**
    * Sets the grip pattern for subsequent grip control.
    *
    * @param grip desired Grip pattern
    */
   public void setGrip(AbilityHandGrip grip)
   {
      this.grip.set(grip);
   }

   /**
    * Retrieves the current grip stage.
    *
    * @return the current grip stage
    */
   public int getGripStage()
   {
      return gripStage;
   }

   /**
    * Sets a goal position for a specific actuator.
    *
    * @param index        actuator index (0 to ACTUATOR_COUNT-1)
    * @param goalPosition desired position in degrees
    */
   public void setGoalPosition(int index, float goalPosition)
   {
      goalPositions.set(index, goalPosition);
   }

   /**
    * Sets goal positions for all actuators.
    *
    * @param goalPositions array of target positions, length ACTUATOR_COUNT
    */
   public void setGoalPositions(float[] goalPositions)
   {
      this.goalPositions.setAll(goalPositions);
   }

   /**
    * Sets a goal velocity for a specific actuator.
    *
    * @param index        actuator index (0 to ACTUATOR_COUNT-1)
    * @param goalVelocity desired velocity
    */
   public void setGoalVelocity(int index, float goalVelocity)
   {
      goalVelocities.set(index, goalVelocity);
   }

   /**
    * Sets goal velocities for all actuators.
    *
    * @param goalVelocities array of target velocities, length ACTUATOR_COUNT
    */
   public void setGoalVelocities(float[] goalVelocities)
   {
      this.goalVelocities.setAll(goalVelocities);
   }

   /**
    * Retrieves the current goal position for a specific actuator.
    *
    * @param index actuator index
    * @return the current goal position for the specified actuator
    */
   public float getGoalPosition(int index)
   {
      return goalPositions.get(index);
   }

   /**
    * Retrieves the current goal velocity for a specific actuator.
    *
    * @param index actuator index
    * @return the current goal velocity for the specified actuator
    */
   public float getGoalVelocity(int index)
   {
      return goalVelocities.get(index);
   }

   /**
    * Get the current {@link AbilityHandCommandType}.
    *
    * @return The current command type.
    */
   public AbilityHandCommandType getCommandType()
   {
      return commandType.getValue();
   }

   /**
    * Set the current {@link AbilityHandCommandType}.
    *
    * @param commandType The command type.
    */
   public void setCommandType(AbilityHandCommandType commandType)
   {
      this.commandType.set(commandType);
   }

   /**
    * Get the command value at the specified index.
    *
    * @param index Index to read the value from.
    * @return The command value.
    */
   public float getCommandValue(int index)
   {
      return commandValues.get(index);
   }

   /**
    * Get the filtered command value at the specified index.
    *
    * @param index Index to read the value from.
    * @return The filtered command value.
    */
   public float getFilteredCommandValue(int index)
   {
      return filteredCommandValues.get(index);
   }

   /**
    * Set the command value at the specified index.
    *
    * @param index Index at which to set the value.
    * @param value The value to set.
    */
   public void setCommandValue(int index, float value)
   {
      commandValues.set(index, value);
   }

   /**
    * Set the command values.
    *
    * @param values The command values.
    */
   public void setCommandValues(float[] values)
   {
      commandValues.setAll(values);
   }

   /**
    * Get the position of the actuator at the specified index.
    *
    * @param index Index to read the position from.
    * @return The position value in degrees.
    */
   public float getActuatorPosition(int index)
   {
      return actuatorPositions.get(index);
   }

   /**
    * Set the position of the actuator at the specified index.
    *
    * @param index Index at which to set the position value, in degrees.
    * @param value The value to set
    */
   public void setActuatorPosition(int index, float value)
   {
      actuatorPositions.set(index, value);
   }

   /**
    * Set the actuator positions.
    *
    * @param positions The actuator positions, in degrees.
    */
   public void setActuatorPositions(float[] positions)
   {
      actuatorPositions.setAll(positions);
   }

   /**
    * Get the velocity of the actuator at the specified index.
    *
    * @param index Index to read the velocity from.
    * @return The velocity value in radians per second.
    */
   public float getActuatorVelocity(int index)
   {
      return actuatorVelocities.get(index);
   }

   /**
    * Get the filtered velocity of the actuator at the specified index.
    *
    * @param index Index to read the velocity from.
    * @return The filtered velocity value in radians per second.
    */
   public float getFilteredActuatorVelocity(int index)
   {
      return filteredActuatorVelocities.get(index);
   }

   /**
    * Set the velocity of the actuator at the specified index.
    *
    * @param index Index at which to set the velocity value, in radians per second.
    * @param value The value to set.
    */
   public void setActuatorVelocity(int index, float value)
   {
      actuatorVelocities.set(index, value);
   }

   /**
    * Get the current of the actuator at the specified index.
    *
    * @param index Index to read the current from.
    * @return The current value in amperes.
    */
   public float getActuatorCurrent(int index)
   {
      return actuatorCurrents.get(index);
   }

   /**
    * Set the current of the actuator at the specified index.
    *
    * @param index Index at which to set the current value, in amperes.
    * @param value The value to set.
    */
   public void setActuatorCurrent(int index, float value)
   {
      actuatorCurrents.set(index, value);
   }

   /**
    * Set the actuator currents.
    *
    * @param currents The actuator currents, in amperes.
    */
   public void setActuatorCurrents(float[] currents)
   {
      actuatorCurrents.setAll(currents);
   }

   /**
    * Get the raw FSR ADC value measured by the touch sensor at the specified index.
    *
    * @param index Index of the touch sensor.
    * @return Raw FSR ADC value.
    */
   public int getRawFSRValue(int index)
   {
      return rawFSRReadings.get(index);
   }

   /**
    * Set the value measured by the touch sensor at the specified index.
    *
    * @param index Index of the touch sensor.
    * @param value Raw FSR ADC value.
    */
   public void setRawFSRValue(int index, int value)
   {
      rawFSRReadings.set(index, value);
   }

   /**
    * Set all raw FSR ADC values.
    *
    * @param values Raw FSR ADC values.
    */
   public void setRawFSRValues(int[] values)
   {
      rawFSRReadings.setAll(values);
   }

   /**
    * Get the pressure measured by the touch sensor at the specified index.
    *
    * @param index Index of the touch sensor.
    * @return Pressure measured by the touch sensor, in Newtons.
    */
   public float getSensedPressure(int index)
   {
      int rawADCValue = getRawFSRValue(index);

      // When a touch sensor is not present, the raw adc value reported is 0.
      if (rawADCValue == 0)
         return 0.0f;

      // Do a bunch of funky math to get the approximate force in Newtons
      return (121591f / (40960000f / rawADCValue + 10000f)) + 0.878894f;
   }

   /** {@inheritDoc} */
   public void readJointAngles(double[] jointAngles)
   {
      jointAngles[INDEX_Q1.getIndex(getSide())] = Math.toRadians(getActuatorPosition(0));
      jointAngles[INDEX_Q2.getIndex(getSide())] = Q2_JOINT_MULTIPLIER * Math.toRadians(getActuatorPosition(0)) + Q2_JOINT_OFFSET;
      jointAngles[MIDDLE_Q1.getIndex(getSide())] = Math.toRadians(getActuatorPosition(1));
      jointAngles[MIDDLE_Q2.getIndex(getSide())] = Q2_JOINT_MULTIPLIER * Math.toRadians(getActuatorPosition(1)) + Q2_JOINT_OFFSET;
      jointAngles[RING_Q1.getIndex(getSide())] = Math.toRadians(getActuatorPosition(2));
      jointAngles[RING_Q2.getIndex(getSide())] = Q2_JOINT_MULTIPLIER * Math.toRadians(getActuatorPosition(2)) + Q2_JOINT_OFFSET;
      jointAngles[PINKY_Q1.getIndex(getSide())] = Math.toRadians(getActuatorPosition(3));
      jointAngles[PINKY_Q2.getIndex(getSide())] = Q2_JOINT_MULTIPLIER * Math.toRadians(getActuatorPosition(3)) + Q2_JOINT_OFFSET;
      jointAngles[THUMB_Q1.getIndex(getSide())] = Math.toRadians(getActuatorPosition(5));
      jointAngles[THUMB_Q2.getIndex(getSide())] = Math.toRadians(getActuatorPosition(4));
   }

   /** {@inheritDoc} */
   public int getJointCount()
   {
      return AbilityHandJointName.values.length;
   }
}
