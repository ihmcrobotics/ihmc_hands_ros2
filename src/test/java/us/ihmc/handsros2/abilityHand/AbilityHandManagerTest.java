package us.ihmc.handsros2.abilityHand;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.Grip;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.handsros2.abilityHand.AbilityHandInterface.ACTUATOR_COUNT;

public class AbilityHandManagerTest
{
   private static Stream<Arguments> getControllers()
   {
      TestAbilityHand abilityHand = new TestAbilityHand("24ABH000", RobotSide.LEFT);
      AbilityHandManager manager = new AbilityHandManager(abilityHand);

      YoAbilityHand yoAbilityHand = new YoAbilityHand(null, "24ABH001", RobotSide.RIGHT);
      YoAbilityHandManager yoManager = new YoAbilityHandManager(null, yoAbilityHand);

      return Stream.of(Arguments.of(manager, abilityHand), Arguments.of(yoManager, yoAbilityHand));
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testPositionControl(AbilityHandManager manager, AbilityHandInterface hand)
   {
      float[] targetPositions = {10f, 20f, 30f, 40f, 50f, -10f};
      manager.setControlMode(ControlMode.POSITION);
      manager.setGoalPositions(targetPositions);

      int numberOfSteps = 200;
      float timeStep = 0.01f;

      float[] times = new float[numberOfSteps];
      float[][] commandedPositions = new float[ACTUATOR_COUNT][numberOfSteps];

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         manager.update(timeStep);

         times[stepIndex] = currentTime;
         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
            commandedPositions[fingerIndex][stepIndex] = hand.getCommandValue(fingerIndex);

         currentTime += timeStep;
      }

      // Compute global min/max across all fingers for normalization
      float globalMin = Float.POSITIVE_INFINITY;
      float globalMax = Float.NEGATIVE_INFINITY;
      for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
      {
         for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
         {
            float value = commandedPositions[fingerIndex][stepIndex];
            globalMin = Math.min(globalMin, value);
            globalMax = Math.max(globalMax, value);
         }
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of all finger positions in POSITION mode:");
      asciiPlotAllFingers(times, commandedPositions, globalMin, globalMax, plotWidth);

      assertEquals(AbilityHandCommandType.POSITION, hand.getCommandType());
      
      for (int i = 0; i < targetPositions.length; i++)
         System.out.printf("Finger %d: target=%.3f actual=%.3f%n", i, targetPositions[i], hand.getCommandValue(i));
      for (int i = 0; i < targetPositions.length; i++)
         assertEquals(targetPositions[i], hand.getCommandValue(i), 1e-3f);
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testVelocityControl(AbilityHandManager manager, AbilityHandInterface hand)
   {
      float[] velocities = {1f, 2f, 3f, 4f, 5f, -5f};
      manager.setControlMode(ControlMode.VELOCITY);
      manager.setGoalVelocities(velocities);

      for (int i = 0; i < 200; i++)
         manager.update(0.01f);

      assertEquals(AbilityHandCommandType.VELOCITY, hand.getCommandType());
      for (int i = 0; i < velocities.length; i++)
      {
         assertEquals(velocities[i], hand.getCommandValue(i), 1e-6f);
      }
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testVelToPosControl(AbilityHandManager manager, AbilityHandInterface hand)
   {
      // current < goal => positive, current > goal => negative
      float[] current = {5f, 50f, 0f, 0f, 0f, 0f};
      float[] goals = {10f, 20f, 0f, 0f, 0f, 0f};
      float[] speeds = {2f, 3f, 0f, 0f, 0f, 0f};

      hand.setActuatorPositions(current);
      manager.setControlMode(ControlMode.VEL_TO_POS);
      manager.setGoalPositions(goals);
      manager.setGoalVelocities(speeds);

      for (int i = 0; i < 200; i++)
         manager.update(0.01f);

      assertEquals(AbilityHandCommandType.VELOCITY, hand.getCommandType());
      assertTrue(hand.getCommandValue(0) > 0, "Index 0 should move positively");
      assertTrue(hand.getCommandValue(1) < 0, "Index 1 should move negatively");
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testGripInitialThumbStage(AbilityHandManager manager, AbilityHandInterface hand)
   {
      // Thumb (index 4) must clear first: simulate it at the clear position already
      float[] current = {30f, 30f, 30f, 30f, 30f, 30f};
      hand.setActuatorPositions(current);

      manager.setGoalVelocities(new float[] {5f, 5f, 5f, 5f, 5f, 5f});
      manager.setControlMode(ControlMode.GRIP);
      manager.setGrip(Grip.POWER);

      for (int i = 0; i < 200; i++)
         manager.update(0.01f);

      assertEquals(AbilityHandCommandType.VELOCITY, hand.getCommandType());

      // Thumb index 4 should be zero (already clear) and stage 0 should run:
      for (int i = 0; i < 4; i++)
         assertTrue(hand.getCommandValue(i) > 0, "Finger " + i + " should start closing");
      assertEquals(0f, hand.getCommandValue(4), 1e-6f, "Thumb should not move on clear");
      assertEquals(0f, hand.getCommandValue(5), 1e-6f, "Pinky shouldn't move in stage 0");
   }

   /**
    * ASCII plot for 6 fingers at once.
    * Each time step is a line; each finger is plotted with a different marker.
    */
   private void asciiPlotAllFingers(float[] times,
                                    float[][] fingerValues,
                                    float minimumValue,
                                    float maximumValue,
                                    int plotWidth)
   {
      char[] markers = {'0', '1', '2', '3', '4', '5'};

      float valueRange = maximumValue - minimumValue;
      if (valueRange < 1e-6f)
         valueRange = 1e-6f; // avoid division by zero

      StringBuilder header = new StringBuilder();
      header.append("Step | Time   | ");
      header.append(" ".repeat(Math.max(0, plotWidth)));
      header.append(" | Values (0-5 fingers)");
      System.out.println(header);

      for (int stepIndex = 0; stepIndex < times.length; stepIndex++)
      {
         float time = times[stepIndex];

         // Start line and blank canvas
         StringBuilder line = new StringBuilder();
         line.append(String.format("%4d | %6.3f | ", stepIndex, time));
         char[] canvas = new char[plotWidth];
         for (int i = 0; i < plotWidth; i++)
            canvas[i] = ' ';

         // Place each finger marker
         for (int fingerIndex = 0; fingerIndex < fingerValues.length; fingerIndex++)
         {
            float value = fingerValues[fingerIndex][stepIndex];
            float normalized = (value - minimumValue) / valueRange; // 0..1
            int markerIndex = Math.round(normalized * (plotWidth - 1));
            if (markerIndex < 0) markerIndex = 0;
            if (markerIndex >= plotWidth) markerIndex = plotWidth - 1;

            // If multiple fingers map to the same column, last one wins; this is fine for debugging
            canvas[markerIndex] = markers[fingerIndex];
         }

         line.append(new String(canvas));
         line.append(" |");
         System.out.println(line);
      }
   }
}
