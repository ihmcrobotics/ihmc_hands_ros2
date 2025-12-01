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
      final String IDENTIFIER = GRIPPER_SIDE.name() + "EZGripper";

      // Create a node
      ROS2Node node = new ROS2NodeBuilder().domainId(domainId).build("ezgripperTestNode");

      // Create a command message and its publisher
      EZGripperCommand command = new EZGripperCommand();
      command.setIdentifier(IDENTIFIER);
      command.setOperationMode(OPERATION_MODE.toByte());
      command.setGoalPosition(GOAL_POSITION);
      command.setMaxEffort(MAX_EFFORT);
      command.setTorqueOn(TORQUE_ON);
      ROS2Publisher<EZGripperCommand> publisher = node.createPublisher(EZGripperROS2API.COMMAND_TOPIC);

      // Create a stats subscription
      AtomicBoolean received = new AtomicBoolean(false);
      EZGripperState stateReceived = new EZGripperState();
      ROS2Subscription<EZGripperState> subscription = node.createSubscription2(EZGripperROS2API.STATE_TOPIC, stateMessage ->
      {
         stateReceived.set(stateMessage);

         synchronized (received)
         {
            received.set(true);
            received.notify();
         }
      });

      // Initialize a test gripper
      EZGripper testGripper = new EZGripper(IDENTIFIER, GRIPPER_SIDE);

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
      assertEquals(GRIPPER_SIDE, RobotSide.fromByte(stateReceived.getRobotSide()));
      assertEquals(IDENTIFIER, stateReceived.getIdentifierAsString());

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
      final String IDENTIFIER = GRIPPER_SIDE.name() + "EZGripper1";

      // Create a node
      ROS2Node node = new ROS2NodeBuilder().domainId(domainId).build("abilityTestNode");

      // Create a state message and its publisher
      EZGripperState state = new EZGripperState();
      state.setIdentifier(IDENTIFIER);
      state.setRobotSide(GRIPPER_SIDE.toByte());
      state.setCurrentPosition(CURRENT_POSITION);
      state.setCurrentEffort(CURRENT_EFFORT);
      state.setTemperature(TEMPERATURE);
      ROS2Publisher<EZGripperState> statePublisher = node.createPublisher(EZGripperROS2API.STATE_TOPIC);

      // Create a subscription to command messages
      AtomicBoolean received = new AtomicBoolean(false);
      EZGripperCommand commandReceived = new EZGripperCommand();
      ROS2Subscription<EZGripperCommand> subscription = node.createSubscription2(EZGripperROS2API.COMMAND_TOPIC, commandMessage ->
      {
         commandReceived.set(commandMessage);

         synchronized (received)
         {
            received.set(true);
            received.notify();
         }
      });

      // Create the communication instance
      EZGripperROS2HardwareCommunication communication = new EZGripperROS2HardwareCommunication("test_hardware_comm", domainId);
      communication.start();
      LockSupport.parkNanos((long) 1E8);

      // Publish a state message
      statePublisher.publish(state);
      LockSupport.parkNanos((long) 1E8);

      // Now the communications class should have received the state message
      assertEquals(1, communication.getAvailableHands().size());
      assertEquals(IDENTIFIER, communication.getAvailableHandList().get(0));

      // Assert the state received is correct
      EZGripperState stateReceived = communication.readState(IDENTIFIER);
      assertNotNull(stateReceived);
      assertEquals(GRIPPER_SIDE, RobotSide.fromByte(stateReceived.getRobotSide()));
      assertEquals(CURRENT_POSITION, stateReceived.getCurrentPosition());
      assertEquals(CURRENT_EFFORT, stateReceived.getCurrentEffort());
      assertEquals(TEMPERATURE, stateReceived.getTemperature());

      // Make sure the communications created a command message for the gripper
      assertNotNull(communication.getCommand(IDENTIFIER));
      assertEquals(IDENTIFIER, communication.getCommand(IDENTIFIER).getIdentifierAsString());

      // Set the command message and publish it
      communication.getCommand(IDENTIFIER).setOperationMode(OPERATION_MODE.toByte());
      communication.getCommand(IDENTIFIER).setGoalPosition(GOAL_POSITION);
      communication.getCommand(IDENTIFIER).setMaxEffort(MAX_EFFORT);
      communication.getCommand(IDENTIFIER).setTorqueOn(TORQUE_ON);
      communication.publishCommand(IDENTIFIER);

      // Wait for the subscription to receive it
      synchronized (received)
      {
         if (!received.get())
            received.wait(1000);
      }

      // Make sure subscription received it correctly
      assertTrue(received.get());
      assertEquals(IDENTIFIER, commandReceived.getIdentifierAsString());
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
