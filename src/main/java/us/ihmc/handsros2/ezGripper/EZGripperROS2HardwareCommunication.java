package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.EZGripperCommand;
import ihmc_hands_ros2.EZGripperState;
import us.ihmc.handsros2.LatestMessageSubscription;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.function.LongSupplier;

/**
 * <p>Controller side ROS 2 communication for the {@link EZGripper}. Communicates with low-level hardware control process.</p>
 * <p>Subscribes to {@link EZGripperState} messages and publishes {@link EZGripperCommand} messages.</p>
 */
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class EZGripperROS2HardwareCommunication
{
   private final AsyncROS2Node node;

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
      node = domainId >= 0 ? new AsyncROS2Node(nodeName, domainId) : new AsyncROS2Node(nodeName);

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
    * @return {@code true} if a state message was available.
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
    * @return A copy of the latest state message. {@code null} if no message has been received.
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
      commandPublishers.get(side).publish(command);
      return true;
   }

   /**
    * Start the communication.
    */
   public void start()
   {
   }

   /**
    * Shut the communication down. {@link #start()} cannot be called again after this method.
    */
   public void shutdown()
   {
      for (RobotSide side : RobotSide.values)
      {
         node.destroyPublisher(commandPublishers.get(side));
         stateSubscriptions.get(side).remove();
      }

      node.close();
   }
}
