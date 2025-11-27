package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.handsros2.HandMessageListener;
import us.ihmc.handsros2.HandROS2ControllerCommunication;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * <p>Hardware side ROS 2 communication for the {@link AbilityHandInterface}. Communicates with external controller.</p>
 * <p>Subscribes to {@link AbilityHandCommand} messages and publishes {@link AbilityHandState} messages.</p>
 */
public class AbilityHandROS2ControllerCommunication implements HandROS2ControllerCommunication<AbilityHandManager>
{
   private final RealtimeROS2Node node;

   private final AbilityHandState stateMessage;
   private final ROS2Publisher<AbilityHandState> statePublisher;

   private final AbilityHandCommand commandMessage;
   private final HandMessageListener<AbilityHandCommand> commandListener;
   private final ROS2Subscription<AbilityHandCommand> commandSubscription;

   public AbilityHandROS2ControllerCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public AbilityHandROS2ControllerCommunication(String nodeName, int domainId)
   {
      ROS2NodeBuilder nodeBuilder = new ROS2NodeBuilder();
      if (domainId >= 0)
         nodeBuilder.domainId(domainId);
      node = nodeBuilder.buildRealtime(nodeName);

      stateMessage = new AbilityHandState();
      statePublisher = node.createPublisher(AbilityHandROS2API.STATE_TOPIC);

      commandMessage = new AbilityHandCommand();
      commandListener = new HandMessageListener<>(AbilityHandCommand::new);
      commandSubscription = node.createSubscription(AbilityHandROS2API.COMMAND_TOPIC, commandListener);
   }

   /** {@inheritDoc} */
   @Override
   public void readCommand(AbilityHandManager managerToUpdate)
   {
      if (commandListener.readLatestMessage(managerToUpdate.getHand().getIdentifier(), commandMessage))
      {
         managerToUpdate.setControlMode(AbilityHandControlMode.fromByte(commandMessage.getControlMode()));
         managerToUpdate.setGrip(AbilityHandGrip.fromByte(commandMessage.getGrip()));
         managerToUpdate.setGoalPositions(commandMessage.getGoalPositions());
         managerToUpdate.setGoalVelocities(commandMessage.getGoalVelocities());
      }
   }

   /** {@inheritDoc} */
   @Override
   public void publishState(AbilityHandManager managerToPublish)
   {
      stateMessage.setIdentifier(managerToPublish.getHand().getIdentifier());
      stateMessage.setHandSide(managerToPublish.getHand().getSide().toByte());
      for (int i = 0; i < AbilityHandInterface.ACTUATOR_COUNT; ++i)
      {
         stateMessage.getActuatorPositions()[i] = managerToPublish.getHand().getActuatorPosition(i);
         stateMessage.getActuatorVelocities()[i] = managerToPublish.getHand().getActuatorVelocity(i);
         stateMessage.getActuatorCurrents()[i] = managerToPublish.getHand().getActuatorCurrent(i);
         stateMessage.getGoalPositions()[i] = managerToPublish.getGoalPosition(i);
         stateMessage.getGoalVelocities()[i] = managerToPublish.getGoalVelocity(i);
      }
      stateMessage.setGripStage(managerToPublish.getGripStage());
      for (int i = 0; i < AbilityHandInterface.TOUCH_SENSOR_COUNT; ++i)
         stateMessage.getTouchSensorReadings()[i] = managerToPublish.getHand().getSensedPressure(i);

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
