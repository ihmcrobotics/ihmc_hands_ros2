package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.handsros2.ezGripper.EZGripper.OperationMode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.QueuedROS2Subscription;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * <p>Hardware side ROS 2 communication for the {@link EZGripper}. Communicates with external controller.</p>
 * <p>Subscribes to {@link EZGripperCommand} messages and publishes {@link EZGripperState} messages.</p>
 */
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class EZGripperROS2ControllerCommunication
{
   private final RealtimeROS2Node node;

   private final EZGripperState stateMessage;
   private final SideDependentList<ROS2Publisher<EZGripperState>> statePublishers;

   private final EZGripperCommand commandMessage;
   private final SideDependentList<QueuedROS2Subscription<EZGripperCommand>> commandSubscriptions;

   public EZGripperROS2ControllerCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public EZGripperROS2ControllerCommunication(String nodeName, int domainId)
   {
      ROS2NodeBuilder nodeBuilder = new ROS2NodeBuilder();
      if (domainId >= 0)
         nodeBuilder.domainId(domainId);
      node = nodeBuilder.buildRealtime(nodeName);

      stateMessage = new EZGripperState();
      statePublishers = new SideDependentList<>(side -> node.createPublisher(EZGripperROS2API.STATE_TOPICS.get(side)));

      commandMessage = new EZGripperCommand();
      commandSubscriptions = new SideDependentList<>(side -> node.createQueuedSubscription(EZGripperROS2API.COMMAND_TOPICS.get(side)));
   }

   /**
    * Update the hand with the latest command.
    *
    * @param gripper The hand to update.
    */
   public void readCommand(EZGripper gripper)
   {
      if (commandSubscriptions.get(gripper.getSide()).flushAndGetLatest(commandMessage))
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
         statePublishers.get(side).remove();
         commandSubscriptions.get(side).remove();
      }

      node.destroy();
   }
}
