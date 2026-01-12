package us.ihmc.handsros2;

import us.ihmc.robotics.robotSide.RobotSide;

public interface HandInterface
{
   /**
    * Get this hand's side.
    *
    * @return This hand's side.
    */
   RobotSide getSide();

   /**
    * Get the type of this hand.
    *
    * @return This hand's type.
    */
   HandType getType();

   /**
    * Updates the state or configuration of the hand.
    * This method typically handles tasks such as refreshing the internal state,
    * recalculating values, or processing any changes necessary to ensure the hand's behavior
    * or data remains consistent and up-to-date.
    */
   void update();

   /**
    * Read the current joint angles into the passed in array.
    *
    * @param jointAngles Array to pack the joint angles into.
    */
   void readJointAngles(double[] jointAngles);

   /**
    * Get the number of joints in the hand.
    * The length of the joint angle array passed into
    * {@link #readJointAngles(double[])} should be at least this long.
    *
    * @return The number of joints in the hand.
    */
   int getJointCount();
}
