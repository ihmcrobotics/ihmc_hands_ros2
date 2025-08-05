package us.ihmc.handsros2.abilityHand;

import org.junit.jupiter.api.Test;
import us.ihmc.handsros2.HandType;
import us.ihmc.robotics.robotSide.RobotSide;

import static org.junit.jupiter.api.Assertions.*;

public class AbilityHandInterfaceTest
{
   @Test
   public void testEZGripperType()
   {
      TestAbilityHand testEZGripper = new TestAbilityHand("24ABH001", RobotSide.LEFT);
      assertEquals(HandType.ABILITY_HAND, testEZGripper.getType());
   }
}
