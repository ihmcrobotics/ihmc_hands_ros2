package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.handsros2.HandMessageListener;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * <p>Hardware side ROS 2 communication for the {@link AbilityHand}. Communicates with external controller.</p>
 * <p>Subscribes to {@link AbilityHandCommand} messages and publishes {@link AbilityHandState} messages.</p>
 */
public class AbilityHandROS2ControllerCommunication
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

   /**
    * Update the hand manager with the latest command.
    *
    * @param hand The hand manager to update.
    */
   public void readCommand(AbilityHand hand)
   {
      if (commandListener.readLatestMessage(hand.getIdentifier(), commandMessage))
      {
         hand.setControlMode(AbilityHandControlMode.fromByte(commandMessage.getControlMode()));
         hand.setGrip(AbilityHandGrip.fromByte(commandMessage.getGrip()));
         hand.setGoalPositions(commandMessage.getGoalPositions());
         hand.setGoalVelocities(commandMessage.getGoalVelocities());
      }
   }

   /**
    * Publish the hand's state.
    *
    * @param hand Manager of the hand to publish.
    */
   public void publishState(AbilityHand hand)
   {
      stateMessage.setIdentifier(hand.getIdentifier());
      stateMessage.setHandSide(hand.getSide().toByte());
      for (int i = 0; i < AbilityHand.ACTUATOR_COUNT; ++i)
      {
         stateMessage.getActuatorPositions()[i] = hand.getActuatorPosition(i);
         stateMessage.getActuatorVelocities()[i] = hand.getFingerVelocityDegPerSec(i);
         stateMessage.getActuatorCurrents()[i] = hand.getActuatorCurrent(i);
         stateMessage.getGoalPositions()[i] = hand.getGoalPosition(i);
         stateMessage.getGoalVelocities()[i] = hand.getGoalVelocity(i);
      }
      stateMessage.setGripStage(hand.getGripStage());
      for (int i = 0; i < AbilityHand.TOUCH_SENSOR_COUNT; ++i)
         stateMessage.getTouchSensorReadings()[i] = hand.getSensedPressure(i);

      statePublisher.publish(stateMessage);
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

      statePublisher.remove();
      commandSubscription.remove();

      node.destroy();
   }
}
