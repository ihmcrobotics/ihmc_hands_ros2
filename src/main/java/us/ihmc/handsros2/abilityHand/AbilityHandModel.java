package us.ihmc.handsros2.abilityHand;

import us.ihmc.handsros2.HandModel;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.partNames.HandJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class AbilityHandModel implements HandModel
{
   public enum AbilityHandJointName implements HandJointName
   {
      INDEX_Q1, INDEX_Q2,
      MIDDLE_Q1, MIDDLE_Q2,
      RING_Q1, RING_Q2,
      PINKY_Q1, PINKY_Q2,
      THUMB_Q1, THUMB_Q2;

      public static final AbilityHandJointName[] values = values();

      // Values taken from the Ability Hand URDF
      public static final double Q2_JOINT_MULTIPLIER = 1.05851325;
      public static final double Q2_JOINT_OFFSET = 0.72349796;

      @Override
      public FingerName getFinger(RobotSide robotSide)
      {
         return switch (this)
         {
            case INDEX_Q1, INDEX_Q2 -> FingerName.INDEX;
            case MIDDLE_Q1, MIDDLE_Q2 -> FingerName.MIDDLE;
            case RING_Q1, RING_Q2 -> FingerName.RING;
            case PINKY_Q1, PINKY_Q2 -> FingerName.PINKY;
            case THUMB_Q1, THUMB_Q2 -> FingerName.THUMB;
         };
      }

      @Override
      public int getIndex(RobotSide robotSide)
      {
         return ordinal();
      }

      @Override
      public HandJointName[] getValues()
      {
         return values();
      }

      @Override
      public String getJointName(RobotSide robotSide)
      {
         return name().toLowerCase();
      }

      public static AbilityHandJointName getJoint(int index)
      {
         return values[index];
      }
   }

   @Override
   public HandJointName[] getHandJointNames()
   {
      return AbilityHandJointName.values;
   }
}
