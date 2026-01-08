package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.handsros2.LatestMessageSubscription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.RealtimeROS2Node;

import java.util.function.LongSupplier;

/**
 * <p>Controller side ROS 2 communication for the {@link EZGripper}. Communicates with low-level hardware control process.</p>
 * <p>Subscribes to {@link EZGripperState} messages and publishes {@link EZGripperCommand} messages.</p>
 */
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class EZGripperROS2HardwareCommunication
{
   private final RealtimeROS2Node node;

   private final SideDependentList<LatestMessageSubscription<EZGripperState>> stateSubscriptions;
   private final SideDependentList<ROS2Publisher<EZGripperCommand>> commandPublishers;

   private final EZGripperState stateMessage = new EZGripperState();

   public EZGripperROS2HardwareCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public EZGripperROS2HardwareCommunication(String nodeName, int domainId)
   {
      this(nodeName, domainId, System::currentTimeMillis);
   }

   public EZGripperROS2HardwareCommunication(String nodeName, int domainId, LongSupplier epochMillisSupplier)
   {
      ROS2NodeBuilder nodeBuilder = new ROS2NodeBuilder();
      if (domainId >= 0)
         nodeBuilder.domainId(domainId);
      node = nodeBuilder.buildRealtime(nodeName);

      stateSubscriptions = new SideDependentList<>(side -> new LatestMessageSubscription<>(node,
                                                                                           EZGripperROS2API.STATE_TOPICS.get(side),
                                                                                           EZGripperState::new,
                                                                                           epochMillisSupplier));
      commandPublishers = new SideDependentList<>(side -> node.createPublisher(EZGripperROS2API.COMMAND_TOPICS.get(side)));
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
    * @return {@code true} if a new state message was available.
    */
   public boolean readState(RobotSide side, EZGripperState stateToPack)
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
    * @return A copy of the latest state message, if a new one was received.
    *       {@code null} if no new message has been received since the last read.
    */
   public EZGripperState readState(RobotSide side)
   {
      EZGripperState stateMessage = new EZGripperState();
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
   public boolean publishCommand(RobotSide side, EZGripperCommand command)
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
