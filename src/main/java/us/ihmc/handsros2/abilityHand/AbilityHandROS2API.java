package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.AbilityHandCommand;
import ihmc_hands_ros2.AbilityHandState;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Collection of the ROS 2 topics for communicating with the {@code AbilityHandROS2*Communication} classes.
 */
public class AbilityHandROS2API
{
   public static final ROS2Topic<?> ROOT_TOPIC = new ROS2Topic<>("/ability_hand");

   public static final ROS2Topic<?> LEFT_TOPIC = ROOT_TOPIC.appendedWith("left");
   public static final ROS2Topic<AbilityHandState> LEFT_STATE_TOPIC = LEFT_TOPIC.appendedWith("state").withType(AbilityHandState.class);
   public static final ROS2Topic<AbilityHandCommand> LEFT_COMMAND_TOPIC = LEFT_TOPIC.appendedWith("command").withType(AbilityHandCommand.class);

   public static final ROS2Topic<?> RIGHT_TOPIC = ROOT_TOPIC.appendedWith("right");
   public static final ROS2Topic<AbilityHandState> RIGHT_STATE_TOPIC = RIGHT_TOPIC.appendedWith("state").withType(AbilityHandState.class);
   public static final ROS2Topic<AbilityHandCommand> RIGHT_COMMAND_TOPIC = RIGHT_TOPIC.appendedWith("command").withType(AbilityHandCommand.class);

   public static final SideDependentList<ROS2Topic<AbilityHandState>> STATE_TOPICS = new SideDependentList<>(LEFT_STATE_TOPIC, RIGHT_STATE_TOPIC);
   public static final SideDependentList<ROS2Topic<AbilityHandCommand>> COMMAND_TOPICS = new SideDependentList<>(LEFT_COMMAND_TOPIC, RIGHT_COMMAND_TOPIC);
}
