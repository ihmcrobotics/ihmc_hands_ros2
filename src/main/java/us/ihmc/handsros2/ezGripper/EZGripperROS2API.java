package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Topic;

/**
 * Collection of the ROS 2 topics for communicating with the {@code EZGripper*Communication} classes.
 */
public class EZGripperROS2API
{
   public static final ROS2Topic<?> ROOT_TOPIC = new ROS2Topic<>().withPrefix("ezgripper");

   public static final ROS2Topic<?> LEFT_TOPIC = ROOT_TOPIC.withModule("left");
   public static final ROS2Topic<EZGripperState> LEFT_STATE_TOPIC = LEFT_TOPIC.withSuffix("state").withType(EZGripperState.class);
   public static final ROS2Topic<EZGripperCommand> LEFT_COMMAND_TOPIC = LEFT_TOPIC.withSuffix("command").withType(EZGripperCommand.class);

   public static final ROS2Topic<?> RIGHT_TOPIC = ROOT_TOPIC.withModule("right");
   public static final ROS2Topic<EZGripperState> RIGHT_STATE_TOPIC = RIGHT_TOPIC.withSuffix("state").withType(EZGripperState.class);
   public static final ROS2Topic<EZGripperCommand> RIGHT_COMMAND_TOPIC = RIGHT_TOPIC.withSuffix("command").withType(EZGripperCommand.class);

   public static final SideDependentList<ROS2Topic<EZGripperState>> STATE_TOPICS = new SideDependentList<>(LEFT_STATE_TOPIC, RIGHT_STATE_TOPIC);
   public static final SideDependentList<ROS2Topic<EZGripperCommand>> COMMAND_TOPICS = new SideDependentList<>(LEFT_COMMAND_TOPIC, RIGHT_COMMAND_TOPIC);
}
