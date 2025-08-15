package us.ihmc.handsros2;

import us.ihmc.robotics.robotSide.RobotSide;

public interface HandInterface
{
   /**
    * Get this hand's unique identifier.
    * This may be the hand's serial number, ID number,
    * or a custom defined identifier.
    *
    * @return This hand's unique identifier.
    */
   String getIdentifier();

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

   /**
    * Create a simple identifier.
    * <p>
    * The identifier will be formatted as: prefix_side_type.
    *
    * @param prefix   A prefix that will be prepended to the identifier.
    * @param handSide The hand's side.
    * @param handType The hand's type.
    * @return A simple identifier.
    */
   static String getSimpleIdentifier(String prefix, RobotSide handSide, HandType handType)
   {
      return prefix + "_" + handSide.getLowerCaseName() + "_" + handType.name().toLowerCase();
   }
}
