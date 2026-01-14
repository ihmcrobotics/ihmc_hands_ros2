package us.ihmc.handsros2;

import us.ihmc.robotics.partNames.HandJointName;
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
    * Get the position of a joint. 0.0 is returned if unknown.
    *
    * @param jointName Name of the joint to query.
    * @return Position of the queried joint. 0.0 when unknown.
    */
   default double getJointPosition(HandJointName jointName)
   {
      return 0.0;
   }

   /**
    * Get the velocity of the joint at the specified index.
    * 0.0 is returned if unknown.
    *
    * @param jointIndex Index of the joint to query.
    * @return Velocity of the queried joint. 0.0 when unknown.
    */
   default double getJointVelocity(HandJointName jointIndex)
   {
      return 0.0;
   }
}
