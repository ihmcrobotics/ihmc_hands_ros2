package us.ihmc.handsros2.ezGripper;

import us.ihmc.robotics.robotSide.RobotSide;

class TestEZGripper implements EZGripperInterface
{
   private final RobotSide side;

   private float goalPosition = 0.1f;
   private float maxEffort = 0.3f;
   private boolean torqueOn = false;

   private float currentPosition = 0.0f;
   private float currentEffort = 0.0f;
   private byte temperature = 0;
   private int realtimeTick = 0;
   private byte errorCode = 0;

   public TestEZGripper(RobotSide side)
   {
      this.side = side;
   }

   @Override
   public RobotSide getRobotSide()
   {
      return side;
   }

   @Override
   public boolean updateCalibration()
   {
      return true;
   }

   @Override
   public void setGoalPosition(float goalPosition)
   {
      this.goalPosition = goalPosition;
   }

   @Override
   public float getGoalPosition()
   {
      return goalPosition;
   }

   @Override
   public void setMaxEffort(float maxEffort)
   {
      this.maxEffort = maxEffort;
   }

   @Override
   public float getMaxEffort()
   {
      return maxEffort;
   }

   @Override
   public void setTorqueOn(boolean on)
   {
      this.torqueOn = on;
   }

   @Override
   public boolean getTorqueOnCommand()
   {
      return torqueOn;
   }

   @Override
   public float getCurrentPosition()
   {
      return currentPosition;
   }

   @Override
   public void setCurrentPosition(float currentPosition)
   {
      this.currentPosition = currentPosition;
   }

   @Override
   public float getCurrentEffort()
   {
      return currentEffort;
   }

   @Override
   public void setCurrentEffort(float currentEffort)
   {
      this.currentEffort = currentEffort;
   }

   @Override
   public byte getTemperature()
   {
      return temperature;
   }

   @Override
   public void setCurrentTemperature(byte currentTemperature)
   {
      temperature = currentTemperature;
   }

   @Override
   public int getRealtimeTick()
   {
      return realtimeTick;
   }

   @Override
   public void setRealtimeTick(int realtimeTick)
   {
      this.realtimeTick = realtimeTick;
   }

   @Override
   public byte getErrorCode()
   {
      return errorCode;
   }

   @Override
   public void setErrorCode(byte errorCode)
   {
      this.errorCode = errorCode;
   }
}
