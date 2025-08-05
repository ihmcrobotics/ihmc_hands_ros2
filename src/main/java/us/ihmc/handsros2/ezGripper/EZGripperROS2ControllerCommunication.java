package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.handsros2.HandMessageListener;
import us.ihmc.handsros2.HandROS2ControllerCommunication;
import us.ihmc.handsros2.ezGripper.EZGripperManager.OperationMode;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * <p>Hardware side ROS 2 communication for the {@link EZGripperInterface}. Communicates with external controller.</p>
 * <p>Subscribes to {@link EZGripperCommand} messages and publishes {@link EZGripperState} messages.</p>
 */
public class EZGripperROS2ControllerCommunication implements HandROS2ControllerCommunication<EZGripperManager>
{
   private final RealtimeROS2Node node;

   private final EZGripperState stateMessage;
   private final ROS2Publisher<EZGripperState> statePublisher;

   private final EZGripperCommand commandMessage;
   private final HandMessageListener<EZGripperCommand> commandListener;
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
      commandListener = new HandMessageListener<>(EZGripperCommand::new);
      commandSubscription = node.createSubscription(EZGripperROS2API.COMMAND_TOPIC, commandListener);
   }

   /** {@inheritDoc} */
   @Override
   public void readCommand(EZGripperManager gripperManager)
   {
      if (commandListener.readLatestMessage(gripperManager.getHand().getIdentifier(), commandMessage))
      {
         gripperManager.setOperationMode(OperationMode.fromByte(commandMessage.getOperationMode()));

         gripperManager.setTemperatureLimit(commandMessage.getTemperatureLimit());
         gripperManager.setGoalPosition(commandMessage.getGoalPosition());
         gripperManager.setMaxEffort(commandMessage.getMaxEffort());
         gripperManager.setTorqueOn(commandMessage.getTorqueOn());
      }
   }

   /** {@inheritDoc} */
   @Override
   public void publishState(EZGripperManager managerToPublish)
   {
      stateMessage.setIdentifier(managerToPublish.getHand().getIdentifier());
      stateMessage.setRobotSide(managerToPublish.getHand().getSide().toByte());
      stateMessage.setOperationMode(managerToPublish.getOperationMode().toByte());
      stateMessage.setTemperature(managerToPublish.getHand().getTemperature());
      stateMessage.setCurrentPosition(managerToPublish.getHand().getCurrentPosition());
      stateMessage.setCurrentEffort(managerToPublish.getHand().getCurrentEffort());
      stateMessage.setErrorCode(managerToPublish.getHand().getErrorCode());
      stateMessage.setRealtimeTick(managerToPublish.getHand().getRealtimeTick());
      stateMessage.setIsCalibrated(managerToPublish.isCalibrated());

      statePublisher.publish(stateMessage);
   }

   /** {@inheritDoc} */
   @Override
   public void start()
   {
      node.spin();
   }

   /** {@inheritDoc} */
   @Override
   public void shutdown()
   {
      node.stopSpinning();

      statePublisher.remove();
      commandSubscription.remove();

      node.destroy();
   }
}
