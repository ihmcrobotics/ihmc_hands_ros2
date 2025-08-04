package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.handsros2.ezGripper.EZGripperManager.OperationMode;

/**
 * <p>Hardware side ROS 2 communication for the {@link EZGripperInterface}. Communicates with external controller.</p>
 * <p>Subscribes to {@link EZGripperCommand} messages and publishes {@link EZGripperState} messages.</p>
 */
public class EZGripperROS2ControllerCommunication
{
   private final RealtimeROS2Node node;

   private final EZGripperState stateMessage;
   private final ROS2Publisher<EZGripperState> statePublisher;

   private final EZGripperCommand commandMessage;
   private final EZGripperMessageListener<EZGripperCommand> commandListener;
   private final ROS2Subscription<EZGripperCommand> commandSubscription;

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
      statePublisher = node.createPublisher(EZGripperROS2API.STATE_TOPIC);

      commandMessage = new EZGripperCommand();
      commandListener = new EZGripperMessageListener<>(EZGripperCommand::new);
      commandSubscription = node.createSubscription(EZGripperROS2API.COMMAND_TOPIC, commandListener);
   }

   /**
    * Read the latest command into the gripper manager object.
    *
    * @param gripperManager Gripper manager to update using the latest command.
    */
   public void readCommand(EZGripperManager gripperManager)
   {
      if (commandListener.readLatestMessage(gripperManager.getGripper().getRobotSide(), commandMessage))
      {
         gripperManager.setOperationMode(OperationMode.fromByte(commandMessage.getOperationMode()));

         gripperManager.setTemperatureLimit(commandMessage.getTemperatureLimit());
         gripperManager.setGoalPosition(commandMessage.getGoalPosition());
         gripperManager.setMaxEffort(commandMessage.getMaxEffort());
         gripperManager.setTorqueOn(commandMessage.getTorqueOn());
      }
   }

   /**
    * Publish the hand's state.
    *
    * @param managerToPublish The manager of the hand to publish.
    */
   public void publishState(EZGripperManager managerToPublish)
   {
      stateMessage.setRobotSide(managerToPublish.getGripper().getRobotSide().toByte());
      stateMessage.setOperationMode(managerToPublish.getOperationMode().toByte());
      stateMessage.setTemperature(managerToPublish.getGripper().getTemperature());
      stateMessage.setCurrentPosition(managerToPublish.getGripper().getCurrentPosition());
      stateMessage.setCurrentEffort(managerToPublish.getGripper().getCurrentEffort());
      stateMessage.setErrorCode(managerToPublish.getGripper().getErrorCode());
      stateMessage.setRealtimeTick(managerToPublish.getGripper().getRealtimeTick());
      stateMessage.setIsCalibrated(managerToPublish.isCalibrated());

      statePublisher.publish(stateMessage);
   }

   /**
    * Initialize the communication. No messages will be received or published until this method is called.
    */
   public void start()
   {
      node.spin();
   }

   /**
    * Shut the communications down. Messages will no longer be received or published.
    */
   public void shutdown()
   {
      node.stopSpinning();

      statePublisher.remove();
      commandSubscription.remove();

      node.destroy();
   }
}
