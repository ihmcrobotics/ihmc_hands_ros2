package us.ihmc.handsros2.ezGripper;

import us.ihmc.handsros2.HandInterface;

/**
 * Generic interface for a SAKE EZGripper.
 */
public interface EZGripperInterface extends HandInterface
{
   int RAW_RANGE_OF_MOTION = 2500;

   // COMMAND METHODS //

   /**
    * <p>Start the calibration process.</p>
    * <p>Generally the grippers can be calibrated by closing until the fingers collide,
    * then recording the position as the fully closed position. The open position is
    * 2500 raw position units away from the closed position.</p>
    *
    * @return {@code true} if calibration is complete.
    *       If {@code false} is returned, this method will be called again before any other commands are given to the gripper.
    */
   boolean updateCalibration();

   /**
    * Set the goal position.
    * 0.0 = closed, 1.0 = open.
    *
    * @param goalPosition The goal position.
    */
   void setGoalPosition(float goalPosition);

   float getGoalPosition();

   /**
    * Set the maximum effort to put into achieving the goal position.
    * 0.0 = no effort (fingers will not move), 1.0 = maximum effort (can quickly overheat actuator).
    * 0.3 is a reasonable normal value, and it is recommended not to exceed 0.8.
    *
    * @param maxEffort Maximum effort to put into achieving the goal position.
    */
   void setMaxEffort(float maxEffort);

   float getMaxEffort();

   /**
    * Set whether to turn torque on.
    * Keeping the torque off when not needed can hlp keep the gripper's temperature down.
    *
    * @param on Whether to turn the torque on.
    */
   void setTorqueOn(boolean on);

   boolean getTorqueOnCommand();

   // STATE METHODS //
   float getCurrentPosition();

   void setCurrentPosition(float currentPosition);

   float getCurrentEffort();

   void setCurrentEffort(float currentEffort);

   byte getTemperature();

   void setCurrentTemperature(byte currentTemperature);

   int getRealtimeTick();

   void setRealtimeTick(int realtimeTick);

   byte getErrorCode();

   void setErrorCode(byte errorCode);
}
