package us.ihmc.handsros2.ezGripper;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * A partial implementation of the {@link EZGripperInterface} using YoVariables.
 * Only the calibration is left to be implemented.
 */
public abstract class YoEZGripper implements EZGripperInterface
{
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

   public YoEZGripper(YoRegistry registry, String identifier, RobotSide robotSide)
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

   @Override
   public String getIdentifier()
   {
      return identifier;
   }

   @Override
   public RobotSide getSide()
   {
      return robotSide;
   }

   @Override
   public void setGoalPosition(float goalPosition)
   {
      this.goalPosition.set(goalPosition);
   }

   @Override
   public float getGoalPosition()
   {
      return (float) goalPosition.getValue();
   }

   @Override
   public void setMaxEffort(float maxEffort)
   {
      this.maxEffort.set(maxEffort);
   }

   @Override
   public float getMaxEffort()
   {
      return (float) maxEffort.getValue();
   }

   @Override
   public void setTorqueOn(boolean on)
   {
      torqueOn.set(on);
   }

   @Override
   public boolean getTorqueOnCommand()
   {
      return torqueOn.getValue();
   }

   @Override
   public float getCurrentPosition()
   {
      return (float) currentPosition.getValue();
   }

   @Override
   public void setCurrentPosition(float currentPosition)
   {
      this.currentPosition.set(currentPosition);
   }

   @Override
   public float getCurrentEffort()
   {
      return (float) currentEffort.getValue();
   }

   @Override
   public void setCurrentEffort(float currentEffort)
   {
      this.currentEffort.set(currentEffort);
   }

   @Override
   public byte getTemperature()
   {
      return (byte) temperature.getValue();
   }

   @Override
   public void setCurrentTemperature(byte currentTemperature)
   {
      temperature.set(Byte.toUnsignedInt(currentTemperature));
   }

   @Override
   public int getRealtimeTick()
   {
      return realtimeTick.getValue();
   }

   @Override
   public void setRealtimeTick(int realtimeTick)
   {
      this.realtimeTick.set(realtimeTick);
   }

   @Override
   public byte getErrorCode()
   {
      return (byte) errorCode.getValue();
   }

   @Override
   public void setErrorCode(byte errorCode)
   {
      this.errorCode.set(Byte.toUnsignedInt(errorCode));
   }
}
