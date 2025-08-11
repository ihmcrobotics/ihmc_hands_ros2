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
      // The sdf for the gripper does not mirror the fingers; X1 is the left finger and X2 is the right finger on both hands.
      // The inner finger is the thumb, and the outer finger is the index.
      // There is no functional difference to any of these angles since the angles of each joint are always identical.
      GRIPPER_X1, GRIPPER_X1_TIP, GRIPPER_X2, GRIPPER_X2_TIP;

      public static final EZGripperJointName[] values = EZGripperJointName.values();
      public static final EZGripperJointName[] nonFingertipValues = new EZGripperJointName[] {GRIPPER_X1, GRIPPER_X2};

      // Taken from the EZGripper URDF
      public static final double JOINT_RANGE = 1.94;

      @Override
      public int getIndex(RobotSide robotSide)
      {
         return switch (this)
         {
            case GRIPPER_X1 -> robotSide == RobotSide.LEFT ? 1 : 0;
            case GRIPPER_X2 -> robotSide == RobotSide.LEFT ? 0 : 1;
            case GRIPPER_X1_TIP -> robotSide == RobotSide.LEFT ? 3 : 2;
            case GRIPPER_X2_TIP -> robotSide == RobotSide.LEFT ? 2 : 3;
         };
      }

      @Override
      public FingerName getFinger(RobotSide robotSide)
      {
         return switch (this)
         {
            case GRIPPER_X1 -> robotSide == RobotSide.LEFT ? FingerName.INDEX : FingerName.THUMB;
            case GRIPPER_X2 -> robotSide == RobotSide.LEFT ? FingerName.THUMB : FingerName.INDEX;
            default -> null;
         };
      }

      @Override
      public String getJointName(RobotSide robotSide)
      {
         return robotSide.getSideNameInAllCaps() + "_" + this;
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
