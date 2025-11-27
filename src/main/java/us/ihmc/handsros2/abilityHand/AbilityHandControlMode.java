package us.ihmc.handsros2.abilityHand;

/**
 * Control modes for the Ability Hand Manager.
 */
public enum AbilityHandControlMode
{
   POSITION, VELOCITY, GRIP;

   public static final AbilityHandControlMode[] values = values();

   /**
    * Retrieves a ControlMode by its byte ordinal.
    *
    * @param ordinal the byte ordinal
    * @return the corresponding ControlMode
    */
   public static AbilityHandControlMode fromByte(byte ordinal)
   {
      return values[ordinal];
   }

   /**
    * Converts this ControlMode to its byte representation.
    *
    * @return the ordinal as a byte
    */
   public byte toByte()
   {
      return (byte) this.ordinal();
   }
}
