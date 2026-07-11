package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.EZGripperCommand;
import ihmc_hands_ros2.EZGripperState;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Collection of the ROS 2 topics for communicating with the {@code EZGripper*Communication} classes.
 */
public class EZGripperROS2API
{
   public static final ROS2Topic<?> ROOT_TOPIC = new ROS2Topic<>("/ezgripper");

   public static final ROS2Topic<?> LEFT_TOPIC = ROOT_TOPIC.appendedWith("left");
   public static final ROS2Topic<EZGripperState> LEFT_STATE_TOPIC = LEFT_TOPIC.appendedWith("state").withType(EZGripperState.class);
   public static final ROS2Topic<EZGripperCommand> LEFT_COMMAND_TOPIC = LEFT_TOPIC.appendedWith("command").withType(EZGripperCommand.class);

   public static final ROS2Topic<?> RIGHT_TOPIC = ROOT_TOPIC.appendedWith("right");
   public static final ROS2Topic<EZGripperState> RIGHT_STATE_TOPIC = RIGHT_TOPIC.appendedWith("state").withType(EZGripperState.class);
   public static final ROS2Topic<EZGripperCommand> RIGHT_COMMAND_TOPIC = RIGHT_TOPIC.appendedWith("command").withType(EZGripperCommand.class);

   public static final SideDependentList<ROS2Topic<EZGripperState>> STATE_TOPICS = new SideDependentList<>(LEFT_STATE_TOPIC, RIGHT_STATE_TOPIC);
   public static final SideDependentList<ROS2Topic<EZGripperCommand>> COMMAND_TOPICS = new SideDependentList<>(LEFT_COMMAND_TOPIC, RIGHT_COMMAND_TOPIC);
}
