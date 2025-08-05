package us.ihmc.handsros2;

public interface HandManager<T extends HandInterface>
{
   /**
    * Get the hand object this manager manages.
    *
    * @return The hand this manager manages.
    */
   T getHand();

   /**
    * Updates the hand commands based on the desired values set in this manager. Should be called periodically.
    */
   void update();
}
