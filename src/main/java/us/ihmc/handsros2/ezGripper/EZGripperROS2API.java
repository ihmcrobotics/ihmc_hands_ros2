package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.ros2.ROS2Topic;

/**
 * Collection of the ROS 2 topics for communicating with the {@code EZGripper*Communication} classes.
 */
public class EZGripperROS2API
{
   public static final ROS2Topic<?> ROOT_TOPIC = new ROS2Topic<>().withModule("ezgripper");
   public static final ROS2Topic<EZGripperState> STATE_TOPIC = ROOT_TOPIC.withSuffix("state").withType(EZGripperState.class);
   public static final ROS2Topic<EZGripperCommand> COMMAND_TOPIC = ROOT_TOPIC.withSuffix("command").withType(EZGripperCommand.class);
}
