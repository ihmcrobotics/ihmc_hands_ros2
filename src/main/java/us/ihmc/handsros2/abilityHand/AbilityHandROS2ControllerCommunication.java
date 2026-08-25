package us.ihmc.handsros2.abilityHand;

import controller_msgs.HandConfigurationCommandMessage;
import ihmc_hands_ros2.AbilityHandCommand;
import ihmc_hands_ros2.AbilityHandState;
import us.ihmc.handsros2.LatestMessageSubscription;
import us.ihmc.jros2.AsyncROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * <p>Hardware side ROS 2 communication for the {@link AbilityHand}. Communicates with external controller.</p>
 * <p>Subscribes to {@link AbilityHandCommand} messages and generic {@link HandConfigurationCommandMessage}s,
 * and publishes {@link AbilityHandState} messages.</p>
 */
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class AbilityHandROS2ControllerCommunication
{
   private static final float DEFAULT_VELOCITY_DEG_PER_SEC = 180.0f;

   private final AsyncROS2Node node;

   private final AbilityHandState stateMessage;
   private final SideDependentList<ROS2Publisher<AbilityHandState>> statePublishers;

   private final AbilityHandCommand commandMessage;
   private final SideDependentList<LatestMessageSubscription<AbilityHandCommand>> commandSubscriptions;

   private final HandConfigurationCommandMessage configurationMessage;
   private final SideDependentList<LatestMessageSubscription<HandConfigurationCommandMessage>> configurationSubscriptions;
   private final float[] goalVelocities = new float[AbilityHand.ACTUATOR_COUNT];

   public AbilityHandROS2ControllerCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public AbilityHandROS2ControllerCommunication(String nodeName, int domainId)
   {
      node = domainId >= 0 ? new AsyncROS2Node(nodeName, domainId) : new AsyncROS2Node(nodeName);

      stateMessage = new AbilityHandState();
      statePublishers = new SideDependentList<>(side -> node.createPublisher(AbilityHandROS2API.STATE_TOPICS.get(side)));

      commandMessage = new AbilityHandCommand();
      commandSubscriptions = new SideDependentList<>(side -> new LatestMessageSubscription<>(node,
                                                                                             AbilityHandROS2API.COMMAND_TOPICS.get(side),
                                                                                             AbilityHandCommand::new));

      configurationMessage = new HandConfigurationCommandMessage();
      configurationSubscriptions = new SideDependentList<>(side -> new LatestMessageSubscription<>(node,
                                                                                                   handConfigurationTopic(side),
                                                                                                   HandConfigurationCommandMessage::new));
   }

   /**
    * Update the hand with the latest command.
    *
    * @param hand The hand to update.
    */
   public void readCommand(AbilityHand hand)
   {
      try
      {
         if (commandSubscriptions.get(hand.getSide()).readLatestMessage(commandMessage))
            applyAbilityHandCommand(hand);

         if (configurationSubscriptions.get(hand.getSide()).readLatestMessage(configurationMessage))
            applyConfiguration(hand, configurationMessage.getConfiguration());
      }
      catch (Exception ignored)
      {
         // Invalid or unmatched hand messages must not propagate into the EtherCAT cycle.
      }
   }

   /**
    * Drop queued command / configuration messages without applying them. Used when no Psyonic is
    * answering so a later connect cannot replay a stale grip into the EtherCAT PDOs.
    */
   public void discardPendingCommands(RobotSide side)
   {
      try
      {
         commandSubscriptions.get(side).readLatestMessage(commandMessage);
         configurationSubscriptions.get(side).readLatestMessage(configurationMessage);
      }
      catch (Exception ignored)
      {
         // Discard must never take down the EtherCAT cycle.
      }
   }

   private void applyAbilityHandCommand(AbilityHand hand)
   {
      byte controlModeOrdinal = commandMessage.getControlMode();
      if (controlModeOrdinal < 0 || controlModeOrdinal >= AbilityHandControlMode.values.length)
         return;

      AbilityHandControlMode controlMode = AbilityHandControlMode.fromByte(controlModeOrdinal);
      hand.setControlMode(controlMode);
      if (controlMode == AbilityHandControlMode.POSITION)
         hand.setGoalPositions(commandMessage.getGoalPositions());
      if (controlMode == AbilityHandControlMode.GRIP)
      {
         byte gripOrdinal = commandMessage.getGrip();
         if (gripOrdinal >= 0 && gripOrdinal < AbilityHandGrip.values.length)
            hand.setGrip(AbilityHandGrip.fromByte(gripOrdinal));
      }
      hand.setGoalVelocities(commandMessage.getGoalVelocities());
   }

   private void applyConfiguration(AbilityHand hand, int configuration)
   {
      if (configuration <= 0 || configuration >= AbilityHandGrip.values.length)
         return;

      AbilityHandGrip grip = AbilityHandGrip.fromByte((byte) configuration);
      hand.setControlMode(AbilityHandControlMode.GRIP);
      hand.setGrip(grip);
      for (int i = 0; i < goalVelocities.length; i++)
         goalVelocities[i] = DEFAULT_VELOCITY_DEG_PER_SEC;
      hand.setGoalVelocities(goalVelocities);
      LogTools.info("Ability Hand {} configuration {} -> grip {}", hand.getSide().getLowerCaseName(), configuration, grip);
   }

   /**
    * Must match {@code us.ihmc.communication.HandConfigurationAPI#getCommandTopic(RobotSide)}.
    */
   private static ROS2Topic<HandConfigurationCommandMessage> handConfigurationTopic(RobotSide robotSide)
   {
      return new ROS2Topic<>("/ihmc/hand_configuration").appendedWith(robotSide.getLowerCaseName())
                                                        .appendedWith("hand_configuration_command")
                                                        .withType(HandConfigurationCommandMessage.class)
                                                        .withQoS(ROS2QoSProfile.RELIABLE);
   }

   /**
    * Publish the hand's state.
    *
    * @param hand Hand to publish.
    */
   public void publishState(AbilityHand hand)
   {
      try
      {
         for (int i = 0; i < AbilityHand.ACTUATOR_COUNT; ++i)
         {
            stateMessage.getActuatorPositions()[i] = hand.getActuatorPosition(i);
            stateMessage.getActuatorVelocities()[i] = hand.getFilteredActuatorVelocity(i);
            stateMessage.getActuatorCurrents()[i] = hand.getActuatorCurrent(i);
            stateMessage.getGoalPositions()[i] = hand.getGoalPosition(i);
            stateMessage.getGoalVelocities()[i] = hand.getGoalVelocity(i);
         }
         stateMessage.setGripStage(hand.getGripStage());
         for (int i = 0; i < AbilityHand.TOUCH_SENSOR_COUNT; ++i)
            stateMessage.getTouchSensorReadings()[i] = hand.getSensedPressure(i);

         statePublishers.get(hand.getSide()).publish(stateMessage);
      }
      catch (Exception ignored)
      {
         // Hand state publish must not propagate into the EtherCAT cycle.
      }
   }

   /**
    * Shut the communication down. The communication cannot be used after this method.
    */
   public void shutdown()
   {
      for (RobotSide side : RobotSide.values)
      {
         node.destroyPublisher(statePublishers.get(side));
         commandSubscriptions.get(side).remove();
         configurationSubscriptions.get(side).remove();
      }

      node.close();
   }
}
