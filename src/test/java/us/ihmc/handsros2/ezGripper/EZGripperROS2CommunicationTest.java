package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.EZGripperCommand;
import ihmc_hands_ros2.EZGripperState;
import org.junit.jupiter.api.Test;
import us.ihmc.handsros2.ezGripper.EZGripper.OperationMode;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.robotics.robotSide.RobotSide;

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

      ROS2Node node = new ROS2Node("ezgripperTestNode", domainId);

      EZGripperCommand command = new EZGripperCommand();
      command.setOperationMode(OPERATION_MODE.toByte());
      command.setGoalPosition(GOAL_POSITION);
      command.setMaxEffort(MAX_EFFORT);
      command.setTorqueOn(TORQUE_ON);
      ROS2Publisher<EZGripperCommand> publisher = node.createPublisher(EZGripperROS2API.COMMAND_TOPICS.get(GRIPPER_SIDE));

      AtomicBoolean received = new AtomicBoolean(false);
      EZGripperState stateReceived = new EZGripperState();
      ROS2Subscription<EZGripperState> subscription = node.createSubscription(EZGripperROS2API.STATE_TOPICS.get(GRIPPER_SIDE), reader ->
      {
         EZGripperState stateMessage = reader.read();
         if (stateMessage == null)
            return;

         stateReceived.set(stateMessage);

         synchronized (received)
         {
            received.set(true);
            received.notify();
         }
      });

      EZGripper testGripper = new EZGripper(GRIPPER_SIDE);

      testGripper.setCurrentPosition(CURRENT_POSITION);
      testGripper.setCurrentEffort(CURRENT_EFFORT);
      testGripper.setCurrentTemperature(TEMPERATURE);
      testGripper.setRealtimeTick(REALTIME_TICK);

      EZGripperROS2ControllerCommunication controllerCommunication = new EZGripperROS2ControllerCommunication("test_controller_comm", domainId);

      controllerCommunication.publishState(testGripper);

      assertFalse(received.get());

      controllerCommunication.start();
      LockSupport.parkNanos((long) 1E8);

      publisher.publish(command);
      controllerCommunication.publishState(testGripper);

      synchronized (received)
      {
         if (!received.get())
            received.wait(1000);
      }

      controllerCommunication.readCommand(testGripper);
      testGripper.update();

      assertTrue(received.get());
      assertEquals(testGripper.getCurrentPosition(), stateReceived.getCurrentPosition());
      assertEquals(testGripper.getCurrentEffort(), stateReceived.getCurrentEffort());
      assertEquals(testGripper.getTemperature(), stateReceived.getTemperature());
      assertEquals(testGripper.getRealtimeTick(), stateReceived.getRealtimeTick());

      controllerCommunication.shutdown();
      node.destroySubscription(subscription);
      node.destroyPublisher(publisher);
      node.close();
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

      ROS2Node node = new ROS2Node("abilityTestNode", domainId);

      EZGripperState state = new EZGripperState();
      state.setCurrentPosition(CURRENT_POSITION);
      state.setCurrentEffort(CURRENT_EFFORT);
      state.setTemperature(TEMPERATURE);
      ROS2Publisher<EZGripperState> statePublisher = node.createPublisher(EZGripperROS2API.STATE_TOPICS.get(GRIPPER_SIDE));

      AtomicBoolean received = new AtomicBoolean(false);
      EZGripperCommand commandReceived = new EZGripperCommand();
      ROS2Subscription<EZGripperCommand> subscription = node.createSubscription(EZGripperROS2API.COMMAND_TOPICS.get(GRIPPER_SIDE), reader ->
      {
         EZGripperCommand commandMessage = reader.read();
         if (commandMessage == null)
            return;

         commandReceived.set(commandMessage);

         synchronized (received)
         {
            received.set(true);
            received.notify();
         }
      });

      EZGripperROS2HardwareCommunication communication = new EZGripperROS2HardwareCommunication("test_hardware_comm", domainId, () -> 0);
      communication.start();
      LockSupport.parkNanos((long) 1E8);

      statePublisher.publish(state);
      LockSupport.parkNanos((long) 1E8);

      EZGripperState stateReceived = communication.readState(GRIPPER_SIDE);
      assertNotNull(stateReceived);
      assertTrue(communication.isHandConnected(GRIPPER_SIDE, 1000, 500));
      assertFalse(communication.isHandConnected(GRIPPER_SIDE, 1000, 1500));
      assertEquals(CURRENT_POSITION, stateReceived.getCurrentPosition());
      assertEquals(CURRENT_EFFORT, stateReceived.getCurrentEffort());
      assertEquals(TEMPERATURE, stateReceived.getTemperature());

      EZGripperCommand command = new EZGripperCommand();
      command.setOperationMode(OPERATION_MODE.toByte());
      command.setGoalPosition(GOAL_POSITION);
      command.setMaxEffort(MAX_EFFORT);
      command.setTorqueOn(TORQUE_ON);
      boolean published = communication.publishCommand(GRIPPER_SIDE, command);
      assertTrue(published);

      synchronized (received)
      {
         if (!received.get())
            received.wait(1000);
      }

      assertTrue(received.get());
      assertEquals(OPERATION_MODE, OperationMode.fromByte(commandReceived.getOperationMode()));
      assertEquals(GOAL_POSITION, commandReceived.getGoalPosition());
      assertEquals(MAX_EFFORT, commandReceived.getMaxEffort());
      assertEquals(TORQUE_ON, commandReceived.getTorqueOn());

      communication.shutdown();
      node.destroyPublisher(statePublisher);
      node.destroySubscription(subscription);
      node.close();
   }
}
