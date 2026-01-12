package us.ihmc.handsros2.ezGripper;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

/**
 * Enum of errors that can be received from the EZGripper's actuator.
 */
public enum EZGripperError
{
   // Values based off of Dynamixel Protocol 1.0 Error codes (https://emanual.robotis.com/docs/en/dxl/protocol1/#error)
   NONE                 ((byte) 0b00000000),
   OVERLOAD_ERROR       ((byte) 0b00100000),
   CHECKSUM_ERROR       ((byte) 0b00010000),
   RANGE_ERROR          ((byte) 0b00001000),
   OVER_HEATING_ERROR   ((byte) 0b00000100),
   ANGLE_LIMIT_ERROR    ((byte) 0b00000010),
   INPUT_VOLTAGE_ERROR  ((byte) 0b00000001);

   public static final EZGripperError[] values = values();
   public static final EnumSet<EZGripperError> set = EnumSet.allOf(EZGripperError.class);

   public final byte errorCode;

   EZGripperError(byte errorCode)
   {
      this.errorCode = errorCode;
   }

   @Override
   public String toString()
   {
      return name().replace("_", " ");
   }

   /**
    * Get the list of error names from the given error code.
    *
    * @param errorCode The error code.
    * @return A list of the error names from the error code.
    */
   public static List<String> getErrorNames(byte errorCode)
   {
      return getErrorList(errorCode).stream().map(EZGripperError::toString).toList();
   }

   /**
    * Get the list of error enum values from the given error code.
    *
    * @param errorCode The error code.
    * @return A list of the error enum values from the error code.
    */
   public static List<EZGripperError> getErrorList(byte errorCode)
   {
      List<EZGripperError> errorList = new ArrayList<>();
      for (EZGripperError error : values)
         if ((errorCode & error.errorCode) != 0)
            errorList.add(error);

      return errorList;
   }
}
