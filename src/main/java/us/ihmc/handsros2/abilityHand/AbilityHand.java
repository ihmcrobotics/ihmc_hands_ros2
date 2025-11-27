package us.ihmc.handsros2.abilityHand;

import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;
import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

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
 * These can be converted into degrees/sec using:
 *     <code>finger_velocity_deg = gear_ratio * rotor_velocity_rad * (180 / pi)</code>
 * using the following gear ratios:
 * Index, Middle, Ring, Pinky, Thumb Flexor: 649
 * Thumb Rotator: 162.45
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
 */
public class AbilityHand implements HandInterface
{
   public static int ACTUATOR_COUNT = 6;
   public static int TOUCH_SENSOR_COUNT = 30;
   private final String identifier;
   private final RobotSide handSide;
   private final YoEnum<AbilityHandCommandType> commandType;
   private final YoDouble[] commandValues = new YoDouble[ACTUATOR_COUNT];
   private final YoDouble[] actuatorPositions = new YoDouble[ACTUATOR_COUNT];
   private final YoDouble[] actuatorVelocities = new YoDouble[ACTUATOR_COUNT];
   private final YoDouble[] actuatorCurrents = new YoDouble[ACTUATOR_COUNT];
   private final YoInteger[] rawFSRReadings = new YoInteger[TOUCH_SENSOR_COUNT];

   public AbilityHand(String identifier, RobotSide handSide)
   {
      this(new YoRegistry("AbilityHand_" + identifier + "_" + handSide.name()), identifier, handSide);
   }

   public AbilityHand(YoRegistry registry, String identifier, RobotSide handSide)
   {
      this.identifier = identifier;
      this.handSide = handSide;

      String prefix = handSide.name() + "AbilityHand_" + identifier + "_";
      commandType = new YoEnum<>(prefix + "CommandType", registry, AbilityHandCommandType.class);
      commandType.set(AbilityHandCommandType.VELOCITY);

      for (int i = 0; i < ACTUATOR_COUNT; ++i)
      {
         commandValues[i] = new YoDouble(prefix + "Command" + i, registry);
         commandValues[i].set(0.0f);

         actuatorPositions[i] = new YoDouble(prefix + "ActuatorPosition" + i, registry);
         actuatorPositions[i].set(0.0);

         actuatorVelocities[i] = new YoDouble(prefix + "ActuatorVelocity" + i, registry);
         actuatorVelocities[i].set(0.0);

         actuatorCurrents[i] = new YoDouble(prefix + "ActuatorCurrent" + i, registry);
         actuatorCurrents[i].set(0.0);
      }

      for (int i = 0; i < TOUCH_SENSOR_COUNT; ++i)
      {
         rawFSRReadings[i] = new YoInteger(prefix + "RawFSR" + i, registry);
         rawFSRReadings[i].set(0);
      }
   }

   @Override
   public String getIdentifier()
   {
      return identifier;
   }

   @Override
   public RobotSide getSide()
   {
      return handSide;
   }

   public HandType getType()
   {
      return HandType.ABILITY_HAND;
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
      return (float) commandValues[index].getValue();
   }

   /**
    * Set the command value at the specified index.
    *
    * @param index Index at which to set the value.
    * @param value The value to set.
    */
   public void setCommandValue(int index, float value)
   {
      commandValues[index].set(value);
   }

   /**
    * Set the command values.
    *
    * @param values The command values.
    */
   public void setCommandValues(float[] values)
   {
      for (int i = 0; i < ACTUATOR_COUNT && i < values.length; ++i)
         setCommandValue(i, values[i]);
   }

   /**
    * Get the position of the actuator at the specified index.
    *
    * @param index Index to read the position from.
    * @return The position value in degrees.
    */
   public float getActuatorPosition(int index)
   {
      return (float) actuatorPositions[index].getValue();
   }

   /**
    * Set the position of the actuator at the specified index.
    *
    * @param index Index at which to set the position value, in degrees.
    * @param value The value to set
    */
   public void setActuatorPosition(int index, float value)
   {
      actuatorPositions[index].set(value);
   }

   /**
    * Set the actuator positions.
    *
    * @param positions The actuator positions, in degrees.
    */
   public void setActuatorPositions(float[] positions)
   {
      for (int i = 0; i < ACTUATOR_COUNT && i < positions.length; ++i)
         setActuatorPosition(i, positions[i]);
   }

   /**
    * Get the velocity of the actuator at the specified index.
    *
    * @param index Index to read the velocity from.
    * @return The velocity value in radians per second.
    */
   public float getActuatorVelocity(int index)
   {
      return (float) actuatorVelocities[index].getValue();
   }

   /**
    * Set the velocity of the actuator at the specified index.
    *
    * @param index Index at which to set the velocity value, in radians per second.
    * @param value The value to set.
    */
   public void setActuatorVelocity(int index, float value)
   {
      actuatorVelocities[index].set(value);
   }

   /**
    * Set the actuator velocities.
    *
    * @param velocities The actuator velocities, in radians per second.
    */
   public void setActuatorVelocities(float[] velocities)
   {
      for (int i = 0; i < ACTUATOR_COUNT && i < velocities.length; ++i)
         setActuatorVelocity(i, velocities[i]);
   }

   /**
    * Get the current of the actuator at the specified index.
    *
    * @param index Index to read the current from.
    * @return The current value in amperes.
    */
   public float getActuatorCurrent(int index)
   {
      return (float) actuatorCurrents[index].getValue();
   }

   /**
    * Set the current of the actuator at the specified index.
    *
    * @param index Index at which to set the current value, in amperes.
    * @param value The value to set.
    */
   public void setActuatorCurrent(int index, float value)
   {
      actuatorCurrents[index].set(value);
   }

   /**
    * Set the actuator currents.
    *
    * @param currents The actuator currents, in amperes.
    */
   public void setActuatorCurrents(float[] currents)
   {
      for (int i = 0; i < ACTUATOR_COUNT && i < currents.length; ++i)
         setActuatorCurrent(i, currents[i]);
   }

   /**
    * Get the raw FSR ADC value measured by the touch sensor at the specified index.
    *
    * @param index Index of the touch sensor.
    * @return Raw FSR ADC value.
    */
   public int getRawFSRValue(int index)
   {
      return rawFSRReadings[index].getValue();
   }

   /**
    * Set the value measured by the touch sensor at the specified index.
    *
    * @param index Index of the touch sensor.
    * @param value Raw FSR ADC value.
    */
   public void setRawFSRValue(int index, int value)
   {
      rawFSRReadings[index].set(value);
   }

   public void setRawFSRValues(int[] values)
   {
      for (int i = 0; i < TOUCH_SENSOR_COUNT && i < values.length; ++i)
         setRawFSRValue(i, values[i]);
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
