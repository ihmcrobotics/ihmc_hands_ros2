package us.ihmc.handsros2.ezGripper;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TestYoEZGripper extends YoEZGripper
{
   public TestYoEZGripper(YoRegistry registry, RobotSide robotSide)
   {
      super(registry, robotSide);
   }

   @Override
   public boolean updateCalibration()
   {
      return true;
   }
}
