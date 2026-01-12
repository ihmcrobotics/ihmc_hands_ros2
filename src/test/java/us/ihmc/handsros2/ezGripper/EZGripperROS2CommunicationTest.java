package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import org.junit.jupiter.api.Test;
import us.ihmc.handsros2.ezGripper.EZGripper.OperationMode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.LockSupport;

import static org.junit.jupiter.api.Assertions.*;

public class EZGripperROS2CommunicationTest
{
   private static final AtomicInteger nextDomainId = new AtomicInteger(0);

   @Test
   public void testControllerCommunication() throws InterruptedException
   {
      final int domainId = nextDomainId.getAndIncrement();
      final RobotSide GRIPPER_SIDE = RobotSide.LEFT;
      final OperationMode OPERATION_MODE = OperationMode.POSITION_CONTROL;
      final float GOAL_POSITION = 0.7f;
      final float MAX_EFFORT = 0.3f;
      final boolean TORQUE_ON = true;
      final float CURRENT_POSITION = 0.3f;
      final float CURRENT_EFFORT = 0.1f;
      final byte TEMPERATURE = 40;
      final int REALTIME_TICK = 5000;

      // Create a node
      ROS2Node node = new ROS2NodeBuilder().domainId(domainId).build("ezgripperTestNode");

      // Create a command message and its publisher
      EZGripperCommand command = new EZGripperCommand();
      command.setOperationMode(OPERATION_MODE.toByte());
      command.setGoalPosition(GOAL_POSITION);
      command.setMaxEffort(MAX_EFFORT);
      command.setTorqueOn(TORQUE_ON);
      ROS2Publisher<EZGripperCommand> publisher = node.createPublisher(EZGripperROS2API.COMMAND_TOPICS.get(GRIPPER_SIDE));

      // Create a stats subscription
      AtomicBoolean received = new AtomicBoolean(false);
      EZGripperState stateReceived = new EZGripperState();
      ROS2Subscription<EZGripperState> subscription = node.createSubscription2(EZGripperROS2API.STATE_TOPICS.get(GRIPPER_SIDE), stateMessage ->
      {
         stateReceived.set(stateMessage);

         synchronized (received)
         {
            received.set(true);
            received.notify();
         }
      });

      // Initialize a test gripper
      EZGripper testGripper = new EZGripper(GRIPPER_SIDE);

      // Set the state of the gripper
      testGripper.setCurrentPosition(CURRENT_POSITION);
      testGripper.setCurrentEffort(CURRENT_EFFORT);
      testGripper.setCurrentTemperature(TEMPERATURE);
      testGripper.setRealtimeTick(REALTIME_TICK);

      // Create an instance of the communication class
      EZGripperROS2ControllerCommunication controllerCommunication = new EZGripperROS2ControllerCommunication("test_controller_comm", domainId);

      // Publish before starting. Nothing should happen
      controllerCommunication.publishState(testGripper);

      // Assert that no messages were received
      assertFalse(received.get());

      // Start and publish again. Should receive message
      controllerCommunication.start();
      LockSupport.parkNanos((long) 1E8);

      publisher.publish(command);
      controllerCommunication.publishState(testGripper);

      // Wait for the state message to be received
      synchronized (received)
      {
         if (!received.get())
            received.wait(1000);
      }

      // Read values
      controllerCommunication.readCommand(testGripper);
      testGripper.update();

      // Assert that the messages were received
      assertTrue(received.get());
      assertEquals(testGripper.getCurrentPosition(), stateReceived.getCurrentPosition());
      assertEquals(testGripper.getCurrentEffort(), stateReceived.getCurrentEffort());
      assertEquals(testGripper.getTemperature(), stateReceived.getTemperature());
      assertEquals(testGripper.getRealtimeTick(), stateReceived.getRealtimeTick());

      // Shut things down
      controllerCommunication.shutdown();
      subscription.remove();
      publisher.remove();
      node.destroy();
   }

   @Test
   public void testHardwareCommunication() throws InterruptedException
   {
      final int domainId = nextDomainId.getAndIncrement();
      final RobotSide GRIPPER_SIDE = RobotSide.LEFT;
      final OperationMode OPERATION_MODE = OperationMode.POSITION_CONTROL;
      final float GOAL_POSITION = 0.7f;
      final float MAX_EFFORT = 0.3f;
      final boolean TORQUE_ON = true;
      final float CURRENT_POSITION = 0.3f;
      final float CURRENT_EFFORT = 0.1f;
      final byte TEMPERATURE = 40;

      // Create a node
      ROS2Node node = new ROS2NodeBuilder().domainId(domainId).build("abilityTestNode");

      // Create a state message and its publisher
      EZGripperState state = new EZGripperState();
      state.setCurrentPosition(CURRENT_POSITION);
      state.setCurrentEffort(CURRENT_EFFORT);
      state.setTemperature(TEMPERATURE);
      ROS2Publisher<EZGripperState> statePublisher = node.createPublisher(EZGripperROS2API.STATE_TOPICS.get(GRIPPER_SIDE));

      // Create a subscription to command messages
      AtomicBoolean received = new AtomicBoolean(false);
      EZGripperCommand commandReceived = new EZGripperCommand();
      ROS2Subscription<EZGripperCommand> subscription = node.createSubscription2(EZGripperROS2API.COMMAND_TOPICS.get(GRIPPER_SIDE), commandMessage ->
      {
         commandReceived.set(commandMessage);

         synchronized (received)
         {
            received.set(true);
            received.notify();
         }
      });

      // Create the communication instance
      EZGripperROS2HardwareCommunication communication = new EZGripperROS2HardwareCommunication("test_hardware_comm", domainId, () -> 0);
      communication.start();
      LockSupport.parkNanos((long) 1E8);

      // Publish a state message
      statePublisher.publish(state);
      LockSupport.parkNanos((long) 1E8);

      // Assert the state received is correct
      EZGripperState stateReceived = communication.readState(GRIPPER_SIDE);
      assertNotNull(stateReceived);
      assertTrue(communication.isHandConnected(GRIPPER_SIDE, 1000, 500));
      assertFalse(communication.isHandConnected(GRIPPER_SIDE, 1000, 1500));
      assertEquals(CURRENT_POSITION, stateReceived.getCurrentPosition());
      assertEquals(CURRENT_EFFORT, stateReceived.getCurrentEffort());
      assertEquals(TEMPERATURE, stateReceived.getTemperature());

      // Publish a command message
      EZGripperCommand command = new EZGripperCommand();
      command.setOperationMode(OPERATION_MODE.toByte());
      command.setGoalPosition(GOAL_POSITION);
      command.setMaxEffort(MAX_EFFORT);
      command.setTorqueOn(TORQUE_ON);
      boolean published = communication.publishCommand(GRIPPER_SIDE, command);
      assertTrue(published);

      // Wait for the subscription to receive it
      synchronized (received)
      {
         if (!received.get())
            received.wait(1000);
      }

      // Make sure subscription received it correctly
      assertTrue(received.get());
      assertEquals(OPERATION_MODE, OperationMode.fromByte(commandReceived.getOperationMode()));
      assertEquals(GOAL_POSITION, commandReceived.getGoalPosition());
      assertEquals(MAX_EFFORT, commandReceived.getMaxEffort());
      assertEquals(TORQUE_ON, commandReceived.getTorqueOn());

      // Shut things down
      communication.shutdown();
      statePublisher.remove();
      subscription.remove();
      node.destroy();
   }
}
