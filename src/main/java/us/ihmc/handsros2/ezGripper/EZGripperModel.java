package us.ihmc.handsros2.ezGripper;

import us.ihmc.handsros2.HandModel;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.partNames.HandJointName;
import us.ihmc.robotics.robotSide.RobotSide;

@SuppressWarnings("ClassCanBeRecord")
public class EZGripperModel implements HandModel
{
   public enum EZGripperJointName implements HandJointName
   {
      // The urdf for the gripper does not mirror the fingers; 1 is the left finger and 2 is the right finger on both hands.
      // The inner finger is the thumb, and the outer finger is the index.
      // There is no functional difference to any of these angles since the angles of each joint are always identical.
      KNUCKLE_PALM_L1_1, KNUCKLE_PALM_L1_2, KNUCKLE_L1_L2_1, KNUCKLE_L1_L2_2;

      public static final EZGripperJointName[] values = EZGripperJointName.values();
      public static final EZGripperJointName[] nonFingertipValues = new EZGripperJointName[] {KNUCKLE_PALM_L1_1, KNUCKLE_PALM_L1_2};

      // Taken from the EZGripper URDF
      public static final double JOINT_RANGE = 1.94;

      @Override
      public int getIndex(RobotSide robotSide)
      {
         return ordinal();
      }

      @Override
      public FingerName getFinger(RobotSide robotSide)
      {
         return switch (this)
         {
            case KNUCKLE_PALM_L1_1, KNUCKLE_L1_L2_1 -> robotSide == RobotSide.LEFT ? FingerName.INDEX : FingerName.THUMB;
            case KNUCKLE_PALM_L1_2, KNUCKLE_L1_L2_2 -> robotSide == RobotSide.LEFT ? FingerName.THUMB : FingerName.INDEX;
         };
      }

      @Override
      public String getJointName(RobotSide robotSide)
      {
         return robotSide.getLowerCaseName() + "_ezgripper_" + name().toLowerCase();
      }

      @Override
      public HandJointName[] getValues()
      {
         return values;
      }
   }

   private final boolean includeFingertips;

   public EZGripperModel(boolean includeFingertips)
   {
      this.includeFingertips = includeFingertips;
   }

   @Override
   public HandJointName[] getHandJointNames()
   {
      return includeFingertips ? EZGripperJointName.values : EZGripperJointName.nonFingertipValues;
   }

   public boolean hasFingertips()
   {
      return includeFingertips;
   }
}
