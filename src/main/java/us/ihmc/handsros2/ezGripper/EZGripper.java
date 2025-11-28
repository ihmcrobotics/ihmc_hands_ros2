package us.ihmc.handsros2.ezGripper;

import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import static us.ihmc.handsros2.ezGripper.EZGripperModel.EZGripperJointName.*;
import us.ihmc.handsros2.ezGripper.EZGripperModel.EZGripperJointName;

/**
 * Generic implementation for a SAKE EZGripper using YoVariables.
 * Only the calibration is left to be implemented.
 */
public class EZGripper implements HandInterface
{
   public static final int RAW_RANGE_OF_MOTION = 2500;

   private final String identifier;
   private final RobotSide robotSide;

   // Low level control
   private final YoDouble goalPosition;
   private final YoDouble maxEffort;
   private final YoBoolean torqueOn;

   // State
   private final YoDouble currentPosition;
   private final YoDouble currentEffort;
   private final YoInteger temperature;
   private final YoInteger realtimeTick;
   private final YoInteger errorCode;

   public EZGripper(String identifier, RobotSide robotSide)
   {
      this(new YoRegistry("EZGripper_" + identifier + "_" + robotSide.name()), identifier, robotSide);
   }

   public EZGripper(YoRegistry registry, String identifier, RobotSide robotSide)
   {
      this.identifier = identifier;
      this.robotSide = robotSide;

      String prefix = robotSide.name() + "EZGripper";

      goalPosition = new YoDouble(prefix + "GoalPosition", registry);
      maxEffort = new YoDouble(prefix + "MaxEffort", registry);
      torqueOn = new YoBoolean(prefix + "TorqueOn", registry);

      currentPosition = new YoDouble(prefix + "CurrentPosition", registry);
      currentEffort = new YoDouble(prefix + "CurrentEffort", registry);
      temperature = new YoInteger(prefix + "Temperature", registry);
      realtimeTick = new YoInteger(prefix + "RealtimeTick", registry);
      errorCode = new YoInteger(prefix + "ErrorCode", registry);
   }

   public String getIdentifier()
   {
      return identifier;
   }

   public RobotSide getSide()
   {
      return robotSide;
   }

   public HandType getType()
   {
      return HandType.EZ_GRIPPER;
   }

   // COMMAND METHODS //

   /**
    * Start the calibration process.
    *
    * Generally the grippers can be calibrated by closing until the fingers collide,
    * then recording the position as the fully closed position. The open position is
    * 2500 raw position units away from the closed position.
    *
    * @return {@code true} if calibration is complete.
    * If {@code false} is returned, this method will be called again before any other commands are given to the gripper.
    */
   public boolean updateCalibration()
   {
      return false;
   }

   /**
    * Set the goal position.
    * 0.0 = closed, 1.0 = open.
    *
    * @param goalPosition The goal position.
    */
   public void setGoalPosition(float goalPosition)
   {
      this.goalPosition.set(goalPosition);
   }

   public float getGoalPosition()
   {
      return (float) goalPosition.getValue();
   }

   /**
    * Set the maximum effort to put into achieving the goal position.
    * 0.0 = no effort (fingers will not move), 1.0 = maximum effort (can quickly overheat actuator).
    * 0.3 is a reasonable normal value, and it is recommended not to exceed 0.8.
    *
    * @param maxEffort Maximum effort to put into achieving the goal position.
    */
   public void setMaxEffort(float maxEffort)
   {
      this.maxEffort.set(maxEffort);
   }

   public float getMaxEffort()
   {
      return (float) maxEffort.getValue();
   }

   /**
    * Set whether to turn torque on.
    * Keeping the torque off when not needed can help keep the gripper's temperature down.
    *
    * @param on Whether to turn the torque on.
    */
   public void setTorqueOn(boolean on)
   {
      torqueOn.set(on);
   }

   public boolean getTorqueOnCommand()
   {
      return torqueOn.getValue();
   }

   // STATE METHODS //

   public float getCurrentPosition()
   {
      return (float) currentPosition.getValue();
   }

   public void setCurrentPosition(float currentPosition)
   {
      this.currentPosition.set(currentPosition);
   }

   public float getCurrentEffort()
   {
      return (float) currentEffort.getValue();
   }

   public void setCurrentEffort(float currentEffort)
   {
      this.currentEffort.set(currentEffort);
   }

   public byte getTemperature()
   {
      return (byte) temperature.getValue();
   }

   public void setCurrentTemperature(byte currentTemperature)
   {
      temperature.set(Byte.toUnsignedInt(currentTemperature));
   }

   public int getRealtimeTick()
   {
      return realtimeTick.getValue();
   }

   public void setRealtimeTick(int realtimeTick)
   {
      this.realtimeTick.set(realtimeTick);
   }

   public byte getErrorCode()
   {
      return (byte) errorCode.getValue();
   }

   public void setErrorCode(byte errorCode)
   {
      this.errorCode.set(Byte.toUnsignedInt(errorCode));
   }

   public void readJointAngles(double[] jointAngles)
   {
      double angle = JOINT_RANGE * (1.0f - getCurrentPosition());
      jointAngles[KNUCKLE_PALM_L1_1.getIndex(getSide())] = angle;
      jointAngles[KNUCKLE_PALM_L1_2.getIndex(getSide())] = angle;
      // We can't know the angle of the fingertips
      jointAngles[KNUCKLE_L1_L2_1.getIndex(getSide())] = 0.0;
      jointAngles[KNUCKLE_L1_L2_2.getIndex(getSide())] = 0.0;
   }

   public int getJointCount()
   {
      return EZGripperJointName.values.length;
   }
}
