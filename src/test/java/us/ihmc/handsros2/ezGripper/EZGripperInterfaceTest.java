package us.ihmc.handsros2.ezGripper;

import org.junit.jupiter.api.Test;
import us.ihmc.handsros2.HandType;
import us.ihmc.robotics.robotSide.RobotSide;

import static org.junit.jupiter.api.Assertions.*;

public class EZGripperInterfaceTest
{
   @Test
   public void testEZGripperType()
   {
      TestEZGripper testEZGripper = new TestEZGripper("EZGripper", RobotSide.LEFT);
      assertEquals(HandType.EZ_GRIPPER, testEZGripper.getType());
   }
}