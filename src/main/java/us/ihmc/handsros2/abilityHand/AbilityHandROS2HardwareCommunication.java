package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.handsros2.LatestMessageSubscription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.RealtimeROS2Node;

import java.util.function.LongSupplier;

/**
 * <p>High level ROS 2 communication for the {@link AbilityHand}. Communicates with low-level hardware control process.</p>
 * <p>Subscribes to {@link AbilityHandState} messages and publishes {@link AbilityHandCommand} messages.</p>
 */
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class AbilityHandROS2HardwareCommunication
{
   private final RealtimeROS2Node node;

   private final SideDependentList<LatestMessageSubscription<AbilityHandState>> stateSubscriptions;
   private final SideDependentList<ROS2Publisher<AbilityHandCommand>> commandPublishers;

   private final AbilityHandState stateMessage = new AbilityHandState();

   public AbilityHandROS2HardwareCommunication(String nodeName, int domainId)
   {
      this(nodeName, domainId, System::currentTimeMillis);
   }

   public AbilityHandROS2HardwareCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public AbilityHandROS2HardwareCommunication(String nodeName, int domainId, LongSupplier epochMillisSupplier)
   {
      ROS2NodeBuilder nodeBuilder = new ROS2NodeBuilder();
      if (domainId >= 0)
         nodeBuilder.domainId(domainId);
      node = nodeBuilder.buildRealtime(nodeName);

      stateSubscriptions = new SideDependentList<>(side -> new LatestMessageSubscription<>(node,
                                                                                           AbilityHandROS2API.STATE_TOPICS.get(side),
                                                                                           AbilityHandState::new,
                                                                                           epochMillisSupplier));
      commandPublishers = new SideDependentList<>(side -> node.createPublisher(AbilityHandROS2API.COMMAND_TOPICS.get(side)));
   }

   public boolean isHandConnected(RobotSide side, long timeoutMillis)
   {
      return isHandConnected(side, timeoutMillis, System.currentTimeMillis());
   }

   public boolean isHandConnected(RobotSide side, long timeoutMillis, long epochMillis)
   {
      return epochMillis - stateSubscriptions.get(side).getLatestMessageTimestamp() < timeoutMillis;
   }

   /**
    * Read the latest state message of the specified hand.
    *
    * @param side        Side specifying the hand.
    * @param stateToPack State message to pack with the latest state.
    * @return {@code true} if a state message was available.
    */
   public boolean readState(RobotSide side, AbilityHandState stateToPack)
   {
      if (stateSubscriptions.get(side).hasReceivedAMessage())
      {
         stateSubscriptions.get(side).readLatestMessage(stateToPack);
         return true;
      }

      return false;
   }

   /**
    * Read the latest state message of the specified hand.
    *
    * @param side Side specifying the hand.
    * @return A copy of the latest state message. {@code null} if no message has been received.
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
