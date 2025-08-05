package us.ihmc.handsros2.abilityHand;

import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;

/**
 * <p>
 * Generic interface for an Ability Hand.
 * </p>
 * The hand's finger actuators are specified using the following indices:
 *    <ol start = 0>
 *       <li>Index finger</li>
 *       <li>Middle finger</li>
 *       <li>Ring finger</li>
 *       <li>Pinky finger</li>
 *       <li>Thumb flexor</li>
 *       <li>Thumb rotator</li>
 *    </ol>
 * <p>
 * Following the above method, the hand's fingers are specified using indices [0, 4],
 * where 0 = index finger, 1 = middle finger, and so on for the ring, pinky, and thumb fingers.
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
public interface AbilityHandInterface extends HandInterface
{
   int ACTUATOR_COUNT = 6;
   int TOUCH_SENSOR_COUNT = 30;

   default HandType getType()
   {
      return HandType.ABILITY_HAND;
   }

   /**
    * Get the current {@link AbilityHandCommandType}.
    *
    * @return The current command type.
    */
   AbilityHandCommandType getCommandType();

   /**
    * Set the current {@link AbilityHandCommandType}.
    *
    * @param commandType The command type.
    */
   void setCommandType(AbilityHandCommandType commandType);

   /**
    * Get the command value at the specified index.
    *
    * @param index Index to read the value from.
    * @return The command value.
    */
   float getCommandValue(int index);

   /**
    * Set the command value at the specified index.
    *
    * @param index Index at which to set the value.
    * @param value The value to set.
    */
   void setCommandValue(int index, float value);

   /**
    * Set the command values.
    *
    * @param values The command values.
    */
   default void setCommandValues(float[] values)
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
   float getActuatorPosition(int index);

   /**
    * Set the position of the actuator at the specified index.
    *
    * @param index Index at which to set the position value, in degrees.
    * @param value The value to set
    */
   void setActuatorPosition(int index, float value);

   /**
    * Set the actuator positions.
    *
    * @param positions The actuator positions, in degrees.
    */
   default void setActuatorPositions(float[] positions)
   {
      for (int i = 0; i < ACTUATOR_COUNT && i < positions.length; ++i)
         setActuatorPosition(i, positions[i]);
   }

   /**
    * Get the raw FSR ADC value measured by the touch sensor at the specified index.
    *
    * @param index Index of the touch sensor.
    * @return Raw FSR ADC value.
    */
   int getRawFSRValue(int index);

   /**
    * Set the value measured by the touch sensor at the specified index.
    *
    * @param index Index of the touch sensor.
    * @param value Raw FSR ADC value.
    */
   void setRawFSRValue(int index, int value);

   default void setRawFSRValues(int[] values)
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
   default float getSensedPressure(int index)
   {
      int rawADCValue = getRawFSRValue(index);

      // When a touch sensor is not present, the raw adc value reported is 0.
      if (rawADCValue == 0)
         return 0.0f;

      // Do a bunch of funky math to get the approximate force in Newtons
      return (121591f / (40960000f / rawADCValue + 10000f)) + 0.878894f;
   }
}
