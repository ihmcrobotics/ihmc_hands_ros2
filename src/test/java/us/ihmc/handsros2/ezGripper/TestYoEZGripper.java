package us.ihmc.handsros2.ezGripper;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TestYoEZGripper extends YoEZGripper
{
   public TestYoEZGripper(YoRegistry registry, String identifier, RobotSide robotSide)
   {
      super(registry, identifier, robotSide);
   }

   @Override
   public boolean updateCalibration()
   {
      return true;
   }
}
