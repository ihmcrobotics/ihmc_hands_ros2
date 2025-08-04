package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * <p>Controller side ROS 2 communication for the {@link EZGripperInterface}. Communicates with low-level hardware control process.</p>
 * <p>Subscribes to {@link EZGripperState} messages and publishes {@link EZGripperCommand} messages.</p>
 */
public class EZGripperROS2HardwareCommunication
{
   private final RealtimeROS2Node node;

   private final EZGripperMessageListener<EZGripperState> stateListener;
   private final ROS2Subscription<EZGripperState> stateSubscription;

   private final SideDependentList<EZGripperCommand> commandMessages;
   private final ROS2Publisher<EZGripperCommand> commandPublisher;

   public EZGripperROS2HardwareCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public EZGripperROS2HardwareCommunication(String nodeName, int domainId)
   {
      commandMessages = new SideDependentList<>();

      ROS2NodeBuilder nodeBuilder = new ROS2NodeBuilder();
      if (domainId >= 0)
         nodeBuilder.domainId(domainId);
      node = nodeBuilder.buildRealtime(nodeName);

      stateListener = new EZGripperMessageListener<>(EZGripperState::new);
      stateListener.onNewHandRegistered(this::registerNewHand);
      stateSubscription = node.createSubscription(EZGripperROS2API.STATE_TOPIC, stateListener);

      commandPublisher = node.createPublisher(EZGripperROS2API.COMMAND_TOPIC);
   }

   private void registerNewHand(RobotSide newGripperSide)
   {
      EZGripperCommand commandMessage = new EZGripperCommand();
      commandMessage.setRobotSide(newGripperSide.toByte());
      commandMessages.put(newGripperSide, commandMessage);
   }

   /**
    * <p>Get the robot sides of the available grippers.</p>
    * <p>Treat the array as read-only.</p>
    *
    * @return Array of robot sides of the available grippers.
    */
   public RobotSide[] getAvailableGripperSides()
   {
      return commandMessages.sides();
   }

   /**
    * Read the latest state of the specified gripper.
    *
    * @param messageToPack Message to pack with the latest state.
    * @return {@code true} if a message was read. {@code false} if no message was available.
    */
   public boolean readState(RobotSide gripperSide, EZGripperState messageToPack)
   {
      return stateListener.readLatestMessage(gripperSide, messageToPack);
   }

   /**
    * Read the latest state message of the specified gripper.
    *
    * @param gripperSide Robot side specifying the gripper.
    * @return A copy of the latest state message.
    */
   public EZGripperState readState(RobotSide gripperSide)
   {
      EZGripperState stateMessage = new EZGripperState();
      if (readState(gripperSide, stateMessage))
         return stateMessage;

      return null;
   }

   /**
    * <p>Get the command message for the specified gripper.</p>
    * <p>Use this method to set the desired command values.
    * Then publish the command using {@link #publishCommand(RobotSide)}.</p>
    *
    * @param gripperSide Robot side specifying the gripper.
    * @return A reference to the command message for the specified gripper.
    */
   public EZGripperCommand getCommand(RobotSide gripperSide)
   {
      return commandMessages.get(gripperSide);
   }

   /**
    * Publish the command for the specified gripper.
    *
    * @param gripperSide Robot side specifying the gripper.
    * @return {@code true} if the message was published. {@code false} if the gripper specified wasn't found.
    */
   public boolean publishCommand(RobotSide gripperSide)
   {
      EZGripperCommand commandMessage = commandMessages.get(gripperSide);
      if (commandMessage == null)
         return false;

      commandPublisher.publish(commandMessage);
      return true;
   }

   /**
    * Start the communication.
    */
   public void start()
   {
      node.spin();
   }

   /**
    * Stop the communication. {@link #start()} can be called again after this method to re-start communication.
    */
   public void stop()
   {
      node.stopSpinning();
   }

   /**
    * Shut the communication down. {@link #start()} cannot be called again after this method.
    */
   public void shutdown()
   {
      stop();

      commandPublisher.remove();
      stateSubscription.remove();

      node.destroy();
   }
}
