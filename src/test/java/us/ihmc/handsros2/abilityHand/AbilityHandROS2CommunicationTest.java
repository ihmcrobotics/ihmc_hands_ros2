package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import org.junit.jupiter.api.Test;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.LockSupport;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.handsros2.abilityHand.AbilityHand.ACTUATOR_COUNT;
import static us.ihmc.handsros2.abilityHand.AbilityHand.TOUCH_SENSOR_COUNT;

public class AbilityHandROS2CommunicationTest
{
   private static final AtomicInteger nextDomainId = new AtomicInteger(0);

   @Test
   public void testControllerCommunication() throws InterruptedException
   {
      final int domainId = nextDomainId.getAndIncrement();
      final RobotSide HAND_SIDE = RobotSide.LEFT;
      final AbilityHandCommandType COMMAND_TYPE = AbilityHandCommandType.POSITION;
      final float[] COMMAND_VALUES = new float[] {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, -5.0f};
      final float[] ACTUATOR_POSITIONS = new float[] {5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.0f};
      final int[] TOUCH_SENSOR_READINGS = new int[30];

      for (int i = 0; i < TOUCH_SENSOR_COUNT; ++i)
         TOUCH_SENSOR_READINGS[i] = i;

      // Create a node
      ROS2Node node = new ROS2NodeBuilder().domainId(domainId).build("abilityTestNode");

      // Create a command message and its publisher
      AbilityHandCommand command = new AbilityHandCommand();
      command.setControlMode(AbilityHandControlMode.POSITION.toByte());
      System.arraycopy(COMMAND_VALUES, 0, command.getGoalPositions(), 0, ACTUATOR_COUNT);
      for (int i = 0; i < 6; ++i)
         command.getGoalVelocities()[i] = 30.0f;
      ROS2Publisher<AbilityHandCommand> publisher = node.createPublisher(AbilityHandROS2API.COMMAND_TOPICS.get(HAND_SIDE));

      // Create a stats subscription
      AtomicBoolean received = new AtomicBoolean(false);
      AbilityHandState stateReceived = new AbilityHandState();
      ROS2Subscription<AbilityHandState> subscription = node.createSubscription2(AbilityHandROS2API.STATE_TOPICS.get(HAND_SIDE), stateMessage ->
      {
         stateReceived.set(stateMessage);

         synchronized (received)
         {
            received.set(true);
            received.notify();
         }
      });

      // Initialize a test hand
      AbilityHand hand = new AbilityHand(HAND_SIDE);
      hand.setActuatorPositions(ACTUATOR_POSITIONS); // Set the state of the hand
      hand.setRawFSRValues(TOUCH_SENSOR_READINGS);

      // Create an instance of the communication class
      AbilityHandROS2ControllerCommunication controllerCommunication = new AbilityHandROS2ControllerCommunication("test_controller_comm", domainId);

      // Publish before starting. Nothing should happen
      controllerCommunication.publishState(hand);

      // Assert that no messages were received
      assertFalse(received.get());

      // Start and publish again. Should receive message
      controllerCommunication.start();
      LockSupport.parkNanos((long) 1E8);

      publisher.publish(command);
      controllerCommunication.publishState(hand);

      // Wait for the state message to be received
      synchronized (received)
      {
         if (!received.get())
            received.wait(1000);
      }

      // Read values
      controllerCommunication.readCommand(hand);

      for (int i = 0; i < ACTUATOR_COUNT; ++i)
      {
         float expectedPos = hand.getActuatorPosition(i);
         float actualPos = stateReceived.getActuatorPositions()[i];
         System.out.printf("Asserting actuator %d position: expected=%.3f actual=%.3f%n", i, expectedPos, actualPos);
         assertEquals(expectedPos, actualPos);

         float expectedVelDeg = hand.getFilteredActuatorVelocity(i);
         float actualVelDeg = stateReceived.getActuatorVelocities()[i];
         System.out.printf("Asserting actuator %d velocity (deg/s): expected=%.3f actual=%.3f%n", i, expectedVelDeg, actualVelDeg);
         assertEquals(expectedVelDeg, actualVelDeg);
      }

      float[] prevPos = new float[ACTUATOR_COUNT];
      for (int i = 0; i < 50; ++i)
      {
         float dt = 0.01f;
         hand.update(dt);

         for (int f = 0; f < ACTUATOR_COUNT; f++)
         {
            hand.setActuatorPosition(f, hand.getCommandValue(f));
            if (i > 0)
            {
               float velocity = (hand.getCommandValue(f) - prevPos[f]) * (1.0f / dt);
               hand.setActuatorVelocity(f, velocity);
            }
            prevPos[f] = hand.getActuatorPosition(f);
         }

         System.out.printf("Hand state - Position: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, Command: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f%n",
                           hand.getActuatorPosition(0),
                           hand.getActuatorPosition(1),
                           hand.getActuatorPosition(2),
                           hand.getActuatorPosition(3),
                           hand.getActuatorPosition(4),
                           hand.getActuatorPosition(5),
                           hand.getCommandValue(0),
                           hand.getCommandValue(1),
                           hand.getCommandValue(2),
                           hand.getCommandValue(3),
                           hand.getCommandValue(4),
                           hand.getCommandValue(5));
      }

      // Assert that the messages were received
      System.out.printf("Asserting that a state message was received: received=%s%n", received.get());
      assertTrue(received.get());

      for (int i = 0; i < ACTUATOR_COUNT; ++i)
      {
         float expectedCurrent = hand.getActuatorCurrent(i);
         float actualCurrent = stateReceived.getActuatorCurrents()[i];
         System.out.printf("Asserting actuator %d current: expected=%.3f actual=%.3f%n", i, expectedCurrent, actualCurrent);
         assertEquals(expectedCurrent, actualCurrent);

         float expectedCommand = COMMAND_VALUES[i];
         float actualCommand = hand.getCommandValue(i);
         System.out.printf("Asserting actuator %d command value: expected=%.3f actual=%.3f tolerance=%.3f%n", i, expectedCommand, actualCommand, 1.0);
         assertEquals(expectedCommand, actualCommand, 1.0);
      }

      for (int i = 0; i < TOUCH_SENSOR_COUNT; ++i)
      {
         float expectedPressure = hand.getSensedPressure(i);
         float actualPressure = stateReceived.getTouchSensorReadings()[i];
         System.out.printf("Asserting touch sensor %d pressure: expected=%.3f actual=%.3f%n", i, expectedPressure, actualPressure);
         assertEquals(expectedPressure, actualPressure);
      }

      System.out.printf("Asserting command type: expected=%s actual=%s%n", COMMAND_TYPE, hand.getCommandType());
      assertEquals(COMMAND_TYPE, hand.getCommandType());

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
      final RobotSide HAND_SIDE = RobotSide.LEFT;
      final AbilityHandControlMode CONTROL_MODE = AbilityHandControlMode.POSITION;
      final float[] GOAL_POSITIONS = new float[] {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, -5.0f};
      final float[] ACTUATOR_POSITIONS = new float[] {5.0f, 4.0f, 3.0f, 2.0f, 1.0f, 0.0f};
      final float[] ACTUATOR_VELOCITIES = new float[] {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
      final float[] ACTUATOR_CURRENTS = new float[] {7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f};
      final float[] TOUCH_SENSOR_READINGS = new float[30];

      for (int i = 0; i < TOUCH_SENSOR_COUNT; ++i)
         TOUCH_SENSOR_READINGS[i] = TOUCH_SENSOR_COUNT - i;

      // Create a node
      ROS2Node node = new ROS2NodeBuilder().domainId(domainId).build("abilityTestNode");

      // Create a state message and its publisher
      AbilityHandState state = new AbilityHandState();
      System.arraycopy(ACTUATOR_POSITIONS, 0, state.getActuatorPositions(), 0, ACTUATOR_COUNT);
      System.arraycopy(ACTUATOR_VELOCITIES, 0, state.getActuatorVelocities(), 0, ACTUATOR_COUNT);
      System.arraycopy(ACTUATOR_CURRENTS, 0, state.getActuatorCurrents(), 0, ACTUATOR_COUNT);
      System.arraycopy(TOUCH_SENSOR_READINGS, 0, state.getTouchSensorReadings(), 0, TOUCH_SENSOR_COUNT);
      ROS2Publisher<AbilityHandState> statePublisher = node.createPublisher(AbilityHandROS2API.STATE_TOPICS.get(HAND_SIDE));

      // Create a subscription to command messages
      AtomicBoolean received = new AtomicBoolean(false);
      AbilityHandCommand commandReceived = new AbilityHandCommand();
      ROS2Subscription<AbilityHandCommand> subscription = node.createSubscription2(AbilityHandROS2API.COMMAND_TOPICS.get(HAND_SIDE), commandMessage ->
      {
         commandReceived.set(commandMessage);

         synchronized (received)
         {
            received.set(true);
            received.notify();
         }
      });

      // Create the communication instance
      AbilityHandROS2HardwareCommunication communication = new AbilityHandROS2HardwareCommunication("test_hardware_comm", domainId, () -> 0);
      communication.start();
      LockSupport.parkNanos((long) 1E8);

      // Publish a state message
      statePublisher.publish(state);
      LockSupport.parkNanos((long) 1E8);

      // Assert the state received is correct
      AbilityHandState stateReceived = communication.readState(HAND_SIDE);
      assertNotNull(stateReceived);
      assertTrue(communication.isHandConnected(HAND_SIDE, 1000, 500));
      assertFalse(communication.isHandConnected(HAND_SIDE, 1000, 1500));
      assertArrayEquals(ACTUATOR_POSITIONS, stateReceived.getActuatorPositions());
      assertArrayEquals(ACTUATOR_VELOCITIES, stateReceived.getActuatorVelocities());
      assertArrayEquals(ACTUATOR_CURRENTS, stateReceived.getActuatorCurrents());
      assertArrayEquals(TOUCH_SENSOR_READINGS, stateReceived.getTouchSensorReadings());

      // Publish a command
      AbilityHandCommand command = new AbilityHandCommand();
      command.setControlMode(CONTROL_MODE.toByte());
      System.arraycopy(GOAL_POSITIONS, 0, command.getGoalPositions(), 0, ACTUATOR_COUNT);
      boolean published = communication.publishCommand(HAND_SIDE, command);
      assertTrue(published);

      // Wait for the subscription to receive it
      synchronized (received)
      {
         if (!received.get())
            received.wait(1000);
      }

      // Make sure subscription received it correctly
      assertTrue(received.get());
      assertEquals(CONTROL_MODE, AbilityHandControlMode.fromByte(commandReceived.getControlMode()));
      assertArrayEquals(GOAL_POSITIONS, commandReceived.getGoalPositions());

      // Shut things down
      communication.shutdown();
      statePublisher.remove();
      subscription.remove();
      node.destroy();
   }
}
