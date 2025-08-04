package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.Grip;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * <p>Hardware side ROS 2 communication for the {@link AbilityHandInterface}. Communicates with external controller.</p>
 * <p>Subscribes to {@link AbilityHandCommand} messages and publishes {@link AbilityHandState} messages.</p>
 */
public class AbilityHandROS2ControllerCommunication
{
   private final RealtimeROS2Node node;

   private final AbilityHandState stateMessage;
   private final ROS2Publisher<AbilityHandState> statePublisher;

   private final AbilityHandCommand commandMessage;
   private final AbilityHandMessageListener<AbilityHandCommand> commandListener;
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
      commandListener = new AbilityHandMessageListener<>(AbilityHandCommand::new);
      commandSubscription = node.createSubscription(AbilityHandROS2API.COMMAND_TOPIC, commandListener);
   }

   /**
    * Read the latest command into the hand manager object.
    *
    * @param managerToUpdate Hand manager to update using the latest command.
    */
   public void readCommand(AbilityHandManager managerToUpdate)
   {
      if (commandListener.readLatestMessage(managerToUpdate.getHand().getSerialNumber(), commandMessage))
      {
         managerToUpdate.setControlMode(ControlMode.fromByte(commandMessage.getControlMode()));
         managerToUpdate.setGrip(Grip.fromByte(commandMessage.getGrip()));
         managerToUpdate.setGoalPositions(commandMessage.getGoalPositions());
         managerToUpdate.setGoalVelocities(commandMessage.getGoalVelocities());
      }
   }

   /**
    * Publish the hand's state.
    *
    * @param managerToPublish The manager of the hand to publish.
    */
   public void publishState(AbilityHandManager managerToPublish)
   {
      stateMessage.setSerialNumber(managerToPublish.getHand().getSerialNumber());
      stateMessage.setHandSide(managerToPublish.getHand().getHandSide().toByte());
      for (int i = 0; i < AbilityHandInterface.ACTUATOR_COUNT; ++i)
         stateMessage.getActuatorPositions()[i] = managerToPublish.getHand().getActuatorPosition(i);
      for (int i = 0; i < AbilityHandInterface.TOUCH_SENSOR_COUNT; ++i)
         stateMessage.getTouchSensorReadings()[i] = managerToPublish.getHand().getSensedPressure(i);

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
