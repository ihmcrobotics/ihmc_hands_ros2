package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;

/**
 * Collection of the ROS 2 topics for communicating with the {@code AbilityHandROS2*Communication} classes.
 */
public class AbilityHandROS2API
{
   public static final ROS2Topic<?> ROOT_TOPIC = new ROS2Topic<>().withPrefix("ability_hand");

   public static final ROS2Topic<?> LEFT_TOPIC = ROOT_TOPIC.withModule("left");
   public static final ROS2Topic<AbilityHandState> LEFT_STATE_TOPIC = LEFT_TOPIC.withSuffix("state").withType(AbilityHandState.class);
   public static final ROS2Topic<AbilityHandCommand> LEFT_COMMAND_TOPIC = LEFT_TOPIC.withSuffix("command").withType(AbilityHandCommand.class);

   public static final ROS2Topic<?> RIGHT_TOPIC = ROOT_TOPIC.withModule("right");
   public static final ROS2Topic<AbilityHandState> RIGHT_STATE_TOPIC = RIGHT_TOPIC.withSuffix("state").withType(AbilityHandState.class);
   public static final ROS2Topic<AbilityHandCommand> RIGHT_COMMAND_TOPIC = RIGHT_TOPIC.withSuffix("command").withType(AbilityHandCommand.class);

   public static final SideDependentList<ROS2Topic<AbilityHandState>> STATE_TOPICS = new SideDependentList<>(LEFT_STATE_TOPIC, RIGHT_STATE_TOPIC);
   public static final SideDependentList<ROS2Topic<AbilityHandCommand>> COMMAND_TOPICS = new SideDependentList<>(LEFT_COMMAND_TOPIC, RIGHT_COMMAND_TOPIC);
}
