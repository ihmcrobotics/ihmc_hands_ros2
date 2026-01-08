package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.QueuedROS2Subscription;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * <p>High level ROS 2 communication for the {@link AbilityHand}. Communicates with low-level hardware control process.</p>
 * <p>Subscribes to {@link AbilityHandState} messages and publishes {@link AbilityHandCommand} messages.</p>
 */
public class AbilityHandROS2HardwareCommunication
{
   private final RealtimeROS2Node node;

   private final SideDependentList<QueuedROS2Subscription<AbilityHandState>> stateSubscriptions;
   private final SideDependentList<ROS2Publisher<AbilityHandCommand>> commandPublishers;

   public AbilityHandROS2HardwareCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public AbilityHandROS2HardwareCommunication(String nodeName, int domainId)
   {
      ROS2NodeBuilder nodeBuilder = new ROS2NodeBuilder();
      if (domainId >= 0)
         nodeBuilder.domainId(domainId);
      node = nodeBuilder.buildRealtime(nodeName);

      stateSubscriptions = new SideDependentList<>(side -> node.createQueuedSubscription(AbilityHandROS2API.STATE_TOPICS.get(side)));
      commandPublishers = new SideDependentList<>(side -> node.createPublisher(AbilityHandROS2API.COMMAND_TOPICS.get(side)));
   }

   /**
    * Read the latest state message of the specified hand.
    *
    * @param side          Side specifying the hand.
    * @param messageToPack Message to pack with the latest state.
    * @return {@code true} if a new state message was available.
    */
   public boolean readState(RobotSide side, AbilityHandState messageToPack)
   {
      return stateSubscriptions.get(side).flushAndGetLatest(messageToPack);
   }

   /**
    * Read the latest state message of the specified hand.
    *
    * @param side Side specifying the hand.
    * @return A copy of the latest state message, if a new one was received.
    *       {@code null} if no new message has been received since the last read.
    */
   public AbilityHandState readState(RobotSide side)
   {
      AbilityHandState stateMessage = new AbilityHandState();
      if (readState(side, stateMessage))
         return stateMessage;

      return null;
   }

   /**
    * Publish the command for the specified hand.
    *
    * @param side Side specifying the hand.
    * @return {@code true} if the message was published.
    */
   public boolean publishCommand(RobotSide side, AbilityHandCommand command)
   {
      return commandPublishers.get(side).publish(command);
   }

   /**
    * Start the communication.
    */
   public void start()
   {
      node.spin();
   }

   /**
    * Shut the communication down. {@link #start()} cannot be called again after this method.
    */
   public void shutdown()
   {
      node.stopSpinning();

      for (RobotSide side : RobotSide.values)
      {
         commandPublishers.get(side).remove();
         stateSubscriptions.get(side).remove();
      }

      node.destroy();
   }
}
