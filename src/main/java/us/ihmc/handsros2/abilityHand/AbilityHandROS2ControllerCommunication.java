package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.handsros2.LatestMessageSubscription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.RealtimeROS2Node;

/**
 * <p>Hardware side ROS 2 communication for the {@link AbilityHand}. Communicates with external controller.</p>
 * <p>Subscribes to {@link AbilityHandCommand} messages and publishes {@link AbilityHandState} messages.</p>
 */
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class AbilityHandROS2ControllerCommunication
{
   private final RealtimeROS2Node node;

   private final AbilityHandState stateMessage;
   private final SideDependentList<ROS2Publisher<AbilityHandState>> statePublishers;

   private final AbilityHandCommand commandMessage;
   private final SideDependentList<LatestMessageSubscription<AbilityHandCommand>> commandSubscriptions;

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
      statePublishers = new SideDependentList<>(side -> node.createPublisher(AbilityHandROS2API.STATE_TOPICS.get(side)));

      commandMessage = new AbilityHandCommand();
      commandSubscriptions = new SideDependentList<>(side -> new LatestMessageSubscription<>(node,
                                                                                             AbilityHandROS2API.COMMAND_TOPICS.get(side),
                                                                                             AbilityHandCommand::new));
   }

   /**
    * Update the hand with the latest command.
    *
    * @param hand The hand to update.
    */
   public void readCommand(AbilityHand hand)
   {
      if (commandSubscriptions.get(hand.getSide()).hasReceivedAMessage())
      {
         commandSubscriptions.get(hand.getSide()).readLatestMessage(commandMessage);
         AbilityHandControlMode controlMode = AbilityHandControlMode.fromByte(commandMessage.getControlMode());
         hand.setControlMode(controlMode);
         if (controlMode == AbilityHandControlMode.POSITION)
            hand.setGoalPositions(commandMessage.getGoalPositions());
         if (controlMode == AbilityHandControlMode.GRIP)
            hand.setGrip(AbilityHandGrip.fromByte(commandMessage.getGrip()));
         hand.setGoalVelocities(commandMessage.getGoalVelocities());
      }
   }

   /**
    * Publish the hand's state.
    *
    * @param hand Hand to publish.
    */
   public void publishState(AbilityHand hand)
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
