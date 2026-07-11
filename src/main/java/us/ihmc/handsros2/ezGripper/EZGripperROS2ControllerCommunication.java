package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.EZGripperCommand;
import ihmc_hands_ros2.EZGripperState;
import us.ihmc.handsros2.LatestMessageSubscription;
import us.ihmc.handsros2.ezGripper.EZGripper.OperationMode;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * <p>Hardware side ROS 2 communication for the {@link EZGripper}. Communicates with external controller.</p>
 * <p>Subscribes to {@link EZGripperCommand} messages and publishes {@link EZGripperState} messages.</p>
 */
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class EZGripperROS2ControllerCommunication
{
   private final AsyncROS2Node node;

   private final EZGripperState stateMessage;
   private final SideDependentList<ROS2Publisher<EZGripperState>> statePublishers;

   private final EZGripperCommand commandMessage;
   private final SideDependentList<LatestMessageSubscription<EZGripperCommand>> commandSubscriptions;

   public EZGripperROS2ControllerCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public EZGripperROS2ControllerCommunication(String nodeName, int domainId)
   {
      node = domainId >= 0 ? new AsyncROS2Node(nodeName, domainId) : new AsyncROS2Node(nodeName);

      stateMessage = new EZGripperState();
      statePublishers = new SideDependentList<>(side -> node.createPublisher(EZGripperROS2API.STATE_TOPICS.get(side)));

      commandMessage = new EZGripperCommand();
      commandSubscriptions = new SideDependentList<>(side -> new LatestMessageSubscription<>(node,
                                                                                             EZGripperROS2API.COMMAND_TOPICS.get(side),
                                                                                             EZGripperCommand::new));
   }

   /**
    * Update the hand with the latest command.
    *
    * @param gripper The hand to update.
    */
   public void readCommand(EZGripper gripper)
   {
      if (commandSubscriptions.get(gripper.getSide()).readLatestMessage(commandMessage))
      {
         gripper.setOperationMode(OperationMode.fromByte(commandMessage.getOperationMode()));

         gripper.setTemperatureLimit(commandMessage.getTemperatureLimit());
         gripper.setGoalPosition(commandMessage.getGoalPosition());
         gripper.setMaxEffort(commandMessage.getMaxEffort());
         gripper.setTorqueOn(commandMessage.getTorqueOn());
      }
   }

   /**
    * Publish the hand's state.
    *
    * @param hand The hand to publish.
    */
   public void publishState(EZGripper hand)
   {
      stateMessage.setOperationMode(hand.getOperationMode().toByte());
      stateMessage.setTemperature(hand.getTemperature());
      stateMessage.setCurrentPosition(hand.getCurrentPosition());
      stateMessage.setCurrentEffort(hand.getCurrentEffort());
      stateMessage.setErrorCode(hand.getErrorCode());
      stateMessage.setRealtimeTick(hand.getRealtimeTick());
      stateMessage.setIsCalibrated(hand.isCalibrated());

      statePublishers.get(hand.getSide()).publish(stateMessage);
   }

   /**
    * Initialize the communication.
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
         node.destroyPublisher(statePublishers.get(side));
         commandSubscriptions.get(side).remove();
      }

      node.close();
   }
}
