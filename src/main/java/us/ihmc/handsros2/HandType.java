package us.ihmc.handsros2;

import java.util.EnumSet;

public enum HandType
{
   ABILITY_HAND, EZ_GRIPPER;

   public static HandType[] values = values();
   public static EnumSet<HandType> set = EnumSet.allOf(HandType.class);
}
