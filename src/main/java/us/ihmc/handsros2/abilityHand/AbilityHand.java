package us.ihmc.handsros2.abilityHand;

import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;
import us.ihmc.handsros2.YoFloatArray;
import us.ihmc.handsros2.YoIntegerArray;
import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoLong;

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

   private static final float DEADZONE = 3.0f; // TODO: Reduce with current control

   private final String identifier;
   private final RobotSide handSide;

   /** Command type used for low-level interface. */
   private final YoEnum<AbilityHandCommandType> commandType;

   /** Position command values sent to the hand. */
   private final YoFloatArray positionCommands;
   /** Velocity command values sent to the hand. */
   private final YoFloatArray velocityCommands;
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
   private final YoEnum<AbilityHandGrip> previousGrip;
   private final YoLong gripStage;

   /** Goal positions per actuator, used by POSITION, VEL_TO_POS, and GRIP modes. */
   private final YoFloatArray goalPositions;
   /** Goal velocities per actuator, used as maximum velocities in trajectories and velocity mode. */
   private final YoFloatArray goalVelocities;

   /** Track previous time for computing dt. */
   private long previousControlTimeNanos = -1L;
   private final YoDouble controlDT;

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

      commandType = new YoEnum<>(prefix + "CommandType", registry, AbilityHandCommandType.class);
      commandType.set(AbilityHandCommandType.VELOCITY);
      controlMode = new YoEnum<>(prefix + "ControlMode", registry, AbilityHandControlMode.class);
      controlMode.set(AbilityHandControlMode.VELOCITY);
      grip = new YoEnum<>(prefix + "Grip", registry, AbilityHandGrip.class, true);
      grip.set(null);
      previousGrip = new YoEnum<>(prefix + "PreviousGrip", registry, AbilityHandGrip.class, true);
      previousGrip.set(null);
      gripStage = new YoLong(prefix + "GripStage", registry);

      positionCommands = new YoFloatArray(prefix + "PositionCommand", registry, 0, 0, 0, 0, 0, 0);
      velocityCommands = new YoFloatArray(prefix + "VelocityCommand", registry, 0, 0, 0, 0, 0, 0);
      filteredCommandValues = new YoFloatArray(prefix + "FilteredCommand", registry, 0, 0, 0, 0, 0, 0);
      actuatorPositions = new YoFloatArray(prefix + "ActuatorPosition", registry, 0, 0, 0, 0, 0, 0);
      actuatorVelocities = new YoFloatArray(prefix + "ActuatorVelocity", registry, 0, 0, 0, 0, 0, 0);
      filteredActuatorVelocities = new YoFloatArray(prefix + "FilteredActuatorVelocity", registry, 0, 0, 0, 0, 0, 0);
      actuatorCurrents = new YoFloatArray(prefix + "ActuatorCurrent", registry, 0, 0, 0, 0, 0, 0);

      int[] fsrInitial = new int[TOUCH_SENSOR_COUNT];
      rawFSRReadings = new YoIntegerArray(prefix + "RawFSR", registry, fsrInitial);

      goalPositions = new YoFloatArray(prefix + "GoalPosition", registry, 30.0f, 30.0f, 30.0f, 30.0f, 30.0f, -30.0f);
      goalVelocities = new YoFloatArray(prefix + "GoalVelocity", registry, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

      controlDT = new YoDouble(prefix + "_controlDT", registry);
   }

   @Override
   public void update()
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

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         {
            float breakFrequency = 1.0f;
            float tau = 1.0f / (2.0f * (float) Math.PI * breakFrequency);
            float alpha = dt / (tau + dt);
            float rawVelocity = actuatorVelocities.get(i);
            float filteredVelocity = alpha * rawVelocity + (1.0f - alpha) * previousFilteredActuatorVelocities[i];
            previousFilteredActuatorVelocities[i] = filteredVelocity;
            filteredActuatorVelocities.set(i, filteredVelocity);
         }

         {
            float breakFrequency = 1.0f;
            float tau = 1.0f / (2.0f * (float) Math.PI * breakFrequency);
            float alpha = dt / (tau + dt);
            float rawCommand = positionCommands.get(i);
            float filteredCommand = alpha * rawCommand + (1.0f - alpha) * previousFilteredCommandValues[i];
            previousFilteredCommandValues[i] = filteredCommand;
            filteredCommandValues.set(i, filteredCommand);
         }
      }

      AbilityHandControlMode currentControlMode = controlMode.getValue();

      if (currentControlMode != null)
      {
         if (currentControlMode == AbilityHandControlMode.VELOCITY)
         {
            commandType.set(AbilityHandCommandType.VELOCITY);
            velocityCommands.setAll(goalVelocities.toFloatArray());

            positionCommands.setAll(actuatorPositions.toFloatArray()); // Make sure filtered commands dont jump when switching
            goalPositions.setAll(actuatorPositions.toFloatArray()); // Make sure goals stay up to date
         }
         else
         {
            commandType.set(AbilityHandCommandType.POSITION);

            if (currentControlMode == AbilityHandControlMode.GRIP && grip.getValue() != null)
               updateGripControl();

            for (int i = 0; i < ACTUATOR_COUNT; i++)
            {
               float direction = Math.signum(goalPositions.get(i) - positionCommands.get(i));
               float step = direction * goalVelocities.get(i) * (float) controlDT.getValue();
               positionCommands.getYoDouble(i).add(step);
               velocityCommands.set(i, 0.0f); // Just keeping these 0 in case
            }
         }
      }

      previousControlMode = currentControlMode;
   }

   /**
    * Performs multi-stage grip control, moving fingers sequentially through {@link AbilityHandGrip#stages}.
    */
   private void updateGripControl()
   {
      AbilityHandGrip currentGrip = grip.getValue();

      // Check if we're starting a new grip
      if (previousControlMode != AbilityHandControlMode.GRIP || previousGrip.getValue() != currentGrip)
      {
         gripStage.set(0);
         previousGrip.set(currentGrip);
      }

      int stage = (int) gripStage.getValue();
      if (stage >= 0 && stage < currentGrip.stages.length)
      {
         for (int s = 0; s <= stage; s++) // Keep previous stage goals as well
            for (int i = 0; i < currentGrip.stages[s].length; i++)
               goalPositions.set(currentGrip.stages[s][i], currentGrip.positions[s][i]);

         boolean stageComplete = true;
         for (int i = 0; i < currentGrip.stages[stage].length; i++)
         {
            int f = currentGrip.stages[stage][i];
            stageComplete &= Math.abs(goalPositions.get(f) - actuatorPositions.get(f)) < DEADZONE;
         }

         if (stageComplete)
         {
            if (stage + 1 == currentGrip.stages.length)
            {
               controlMode.set(AbilityHandControlMode.POSITION);
               gripStage.set(-1);
            }
            else
            {
               gripStage.increment();
            }
         }
      }
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
      return (int) gripStage.getValue();
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

   public YoEnum<AbilityHandControlMode> getControlMode()
   {
      return controlMode;
   }

   /**
    * Get the command value at the specified index.
    *
    * @param index Index to read the value from.
    * @return The command value.
    */
   public float getCommandValue(int index)
   {
      return switch (getControlMode().getEnumValue())
      {
         case VELOCITY -> velocityCommands.get(index);
         case POSITION, GRIP -> filteredCommandValues.get(index);
      };
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
      int rawADCValue = rawFSRReadings.get(index);

      // When a touch sensor is not present, the raw adc value reported is 0.
      if (rawADCValue == 0)
         return 0.0f;

      // Do a bunch of funky math to get the approximate force in Newtons
      return (121591f / (40960000f / rawADCValue + 10000f)) + 0.878894f;
   }

   /** {@inheritDoc} */
   public void readJointAngles(double[] jointAngles)
   {
      jointAngles[INDEX_Q1.getIndex(handSide)] = Math.toRadians(actuatorPositions.get(0));
      jointAngles[INDEX_Q2.getIndex(handSide)] = Q2_JOINT_MULTIPLIER * Math.toRadians(actuatorPositions.get(0)) + Q2_JOINT_OFFSET;
      jointAngles[MIDDLE_Q1.getIndex(handSide)] = Math.toRadians(actuatorPositions.get(1));
      jointAngles[MIDDLE_Q2.getIndex(handSide)] = Q2_JOINT_MULTIPLIER * Math.toRadians(actuatorPositions.get(1)) + Q2_JOINT_OFFSET;
      jointAngles[RING_Q1.getIndex(handSide)] = Math.toRadians(actuatorPositions.get(2));
      jointAngles[RING_Q2.getIndex(handSide)] = Q2_JOINT_MULTIPLIER * Math.toRadians(actuatorPositions.get(2)) + Q2_JOINT_OFFSET;
      jointAngles[PINKY_Q1.getIndex(handSide)] = Math.toRadians(actuatorPositions.get(3));
      jointAngles[PINKY_Q2.getIndex(handSide)] = Q2_JOINT_MULTIPLIER * Math.toRadians(actuatorPositions.get(3)) + Q2_JOINT_OFFSET;
      jointAngles[THUMB_Q1.getIndex(handSide)] = Math.toRadians(actuatorPositions.get(5));
      jointAngles[THUMB_Q2.getIndex(handSide)] = Math.toRadians(actuatorPositions.get(4));
   }

   /** {@inheritDoc} */
   public int getJointCount()
   {
      return AbilityHandJointName.values.length;
   }
}
