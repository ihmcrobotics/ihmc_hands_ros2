package us.ihmc.handsros2;

/**
 * <p>Interface for hardware side ROS 2 communication for hands. Communicates with an external controller.</p>
 * <p>Subscribes to command messages and publishes state messages.</p>
 *
 * @param <T> Type of the hand manager.
 */
public interface HandROS2ControllerCommunication<T extends HandManager<? extends HandInterface>>
{
   /**
    * Update the hand manager with the latest command.
    *
    * @param handManager The hand manager to update.
    */
   void readCommand(T handManager);

   /**
    * Publish the hand's state.
    *
    * @param handManager Manager of the hand to publish.
    */
   void publishState(T handManager);

   /**
    * Initialize the communication.
    */
   void start();

   /**
    * Shut the communication down. {@link #start()} cannot be called again after this method.
    */
   void shutdown();
}
