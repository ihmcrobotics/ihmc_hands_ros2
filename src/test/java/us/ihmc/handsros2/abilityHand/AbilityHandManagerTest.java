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
import static us.ihmc.handsros2.abilityHand.AbilityHandManager.THUMB_CLEAR_POSITION;

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
      float[][] fingerVelocities = new float[ACTUATOR_COUNT][numberOfSteps];

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         manager.update(timeStep);

         times[stepIndex] = currentTime;
         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
         {
            commandedPositions[fingerIndex][stepIndex] = hand.getCommandValue(fingerIndex);
            fingerVelocities[fingerIndex][stepIndex] = hand.getCommandValue(fingerIndex);
         }

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
      asciiPlotAllFingers(times, commandedPositions, fingerVelocities, globalMin, globalMax, plotWidth);

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
      float[] targetVelocities = {1f, 2f, 3f, 4f, 5f, -5f};
      manager.setControlMode(ControlMode.VELOCITY);
      manager.setGoalVelocities(targetVelocities);

      int numberOfSteps = 200;
      float timeStep = 0.01f;

      float[] times = new float[numberOfSteps];
      float[][] fingerPositions = new float[ACTUATOR_COUNT][numberOfSteps];
      float[][] fingerVelocities = new float[ACTUATOR_COUNT][numberOfSteps];

      // Local simulated positions for the virtual hand
      float[] simulatedPositions = new float[ACTUATOR_COUNT];
      for (int i = 0; i < ACTUATOR_COUNT; i++)
         simulatedPositions[i] = hand.getActuatorPosition(i);

      java.util.Random random = new java.util.Random(12345L);
      float noiseStdDev = 0.001f; // small Gaussian noise on position integration

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         // Run controller: this sets command velocities in the hand
         manager.update(timeStep);

         // Integrate commanded velocities into simulated positions with noise
         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
         {
            float commandedVelocity = hand.getCommandValue(fingerIndex);
            float noise = (float) (random.nextGaussian() * noiseStdDev);
            simulatedPositions[fingerIndex] += (commandedVelocity + noise) * timeStep;
         }

         // Push simulated positions back into the virtual hand as if they were measured
         hand.setActuatorPositions(simulatedPositions);

         // Log for plotting
         times[stepIndex] = currentTime;
         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
         {
            fingerPositions[fingerIndex][stepIndex] = simulatedPositions[fingerIndex];
            fingerVelocities[fingerIndex][stepIndex] = hand.getCommandValue(fingerIndex);
         }

         currentTime += timeStep;
      }

      // Compute global min/max across all fingers for normalization
      float globalMin = Float.POSITIVE_INFINITY;
      float globalMax = Float.NEGATIVE_INFINITY;
      for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
      {
         for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
         {
            float value = fingerPositions[fingerIndex][stepIndex];
            globalMin = Math.min(globalMin, value);
            globalMax = Math.max(globalMax, value);
         }
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of all finger positions in VELOCITY mode:");
      asciiPlotAllFingers(times, fingerPositions, fingerVelocities, globalMin, globalMax, plotWidth);

      assertEquals(AbilityHandCommandType.VELOCITY, hand.getCommandType());
      for (int i = 0; i < targetVelocities.length; i++)
         assertEquals(targetVelocities[i], hand.getCommandValue(i), 1e-6f);
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testVelToPosControl(AbilityHandManager manager, AbilityHandInterface hand)
   {
      // current < goal => positive, current > goal => negative
      float[] initialPositions = {5f, 50f, 0f, 0f, 0f, 0f};
      float[] goalPositions    = {10f, 20f, 0f, 0f, 0f, 0f};
      float[] speeds           = {20f, 30f, 0f, 0f, 0f, 0f};

      // Initialize simulated plant state
      float[] truePositions = new float[ACTUATOR_COUNT];
      float[] trueVelocities = new float[ACTUATOR_COUNT];
      System.arraycopy(initialPositions, 0, truePositions, 0, ACTUATOR_COUNT);

      // Push initial state into the virtual hand
      hand.setActuatorPositions(truePositions);
      hand.setActuatorVelocities(trueVelocities);

      manager.setControlMode(ControlMode.VEL_TO_POS);
      manager.setGoalPositions(goalPositions);
      manager.setGoalVelocities(speeds);

      int numberOfSteps = 110;
      float timeStep = 0.01f;

      float[] times = new float[numberOfSteps];
      float[][] fingerPositions = new float[ACTUATOR_COUNT][numberOfSteps];
      float[][] fingerVelocities = new float[ACTUATOR_COUNT][numberOfSteps];

      java.util.Random random = new java.util.Random(12345L);
      float noiseStdDev = 0.001f;

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         // Before calling update, provide the current measured state to the manager
         hand.setActuatorPositions(truePositions);
         hand.setActuatorVelocities(trueVelocities);

         // Run controller (uses current positions/velocities and writes velocity commands)
         manager.update(timeStep);

         // Integrate commanded velocities into the true plant state with a bit of noise
         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
         {
            float commandedVelocity = hand.getCommandValue(fingerIndex);
            float noise = (float) (random.nextGaussian() * noiseStdDev);

            trueVelocities[fingerIndex] = commandedVelocity;                  // simple velocity servo
            truePositions[fingerIndex] += (commandedVelocity + noise) * timeStep;
         }

         // Log for plotting
         times[stepIndex] = currentTime;
         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
         {
            fingerPositions[fingerIndex][stepIndex] = truePositions[fingerIndex];
            fingerVelocities[fingerIndex][stepIndex] = hand.getCommandValue(fingerIndex);
         }

         currentTime += timeStep;
      }

      // Normalize for plotting (based on positions)
      float globalMin = Float.POSITIVE_INFINITY;
      float globalMax = Float.NEGATIVE_INFINITY;
      for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
      {
         for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
         {
            float value = fingerPositions[fingerIndex][stepIndex];
            globalMin = Math.min(globalMin, value);
            globalMax = Math.max(globalMax, value);
         }
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of all finger positions in VEL_TO_POS mode:");
      asciiPlotAllFingers(times, fingerPositions, fingerVelocities, globalMin, globalMax, plotWidth);

      assertEquals(AbilityHandCommandType.VELOCITY, hand.getCommandType(),
                   "Expected VELOCITY command type but got " + hand.getCommandType());

      assertTrue(Math.abs(truePositions[0] - goalPositions[0]) < 1.5f,
                 String.format("Finger 0 position not near goal. pos=%.3f goal=%.3f vel=%.3f",
                               truePositions[0], goalPositions[0], trueVelocities[0]));

      assertTrue(Math.abs(trueVelocities[0]) < 1.5f,
                 String.format("Finger 0 velocity not near zero. vel=%.3f pos=%.3f goal=%.3f",
                               trueVelocities[0], truePositions[0], goalPositions[0]));

      assertTrue(truePositions[1] < 50.0f,
                 String.format("Finger 1 did not move down from initial. pos=%.3f init=50.0 vel=%.3f goal=%.3f",
                               truePositions[1], trueVelocities[1], goalPositions[1]));

      assertTrue(truePositions[1] > 20.0f,
                 String.format("Finger 1 overshot or did not approach goal correctly. pos=%.3f goal=%.3f vel=%.3f",
                               truePositions[1], goalPositions[1], trueVelocities[1]));

      for (int i = 2; i < ACTUATOR_COUNT; i++)
      {
         assertTrue(Math.abs(truePositions[i]) < 1.0f,
                    String.format("Finger %d position drifted. pos=%.3f expected≈0 vel=%.3f",
                                  i, truePositions[i], trueVelocities[i]));
         assertTrue(Math.abs(trueVelocities[i]) < 1.0f,
                    String.format("Finger %d velocity too large. vel=%.3f pos=%.3f",
                                  i, trueVelocities[i], truePositions[i]));
      }
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testPowerGripWithInitialThumbStage(AbilityHandManager manager, AbilityHandInterface hand)
   {
      // Thumb (index 4) must clear first: simulate it at the clear position already
      float[] initialPositions = {0f, 2f, 7f, 15f, 90f, 10f};
      hand.setActuatorPositions(initialPositions);

      manager.setGoalVelocities(new float[] {80f, 80f, 80f, 80f, 80f, 80f});
      manager.setControlMode(ControlMode.GRIP);
      manager.setGrip(Grip.POWER);

      int numberOfSteps = 800;
      float timeStep = 0.01f;

      float[] times = new float[numberOfSteps];
      float[][] fingerPositions = new float[ACTUATOR_COUNT][numberOfSteps];

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         // Manager computes desired positions in GRIP mode
         manager.update(timeStep);

         times[stepIndex] = currentTime;

         // For this test, treat commanded positions as the actual positions
         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
         {
            float commandedPosition = hand.getCommandValue(fingerIndex);
            fingerPositions[fingerIndex][stepIndex] = commandedPosition;
         }

         currentTime += timeStep;
      }

      // Normalize for plotting
      float globalMin = Float.POSITIVE_INFINITY;
      float globalMax = Float.NEGATIVE_INFINITY;
      for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
      {
         for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
         {
            float value = fingerPositions[fingerIndex][stepIndex];
            globalMin = Math.min(globalMin, value);
            globalMax = Math.max(globalMax, value);
         }
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of all finger positions in GRIP mode (initial thumb stage):");
      // We do not care about velocities here, so pass null for velocity array
      asciiPlotAllFingers(times, fingerPositions, null, globalMin, globalMax, plotWidth);

      // GRIP mode should be issuing POSITION commands
      assertEquals(AbilityHandCommandType.POSITION, hand.getCommandType(),
                   "Expected POSITION command type but got " + hand.getCommandType());

      // Thumb index 4 should stay near its clear position
      float thumbPosition = hand.getCommandValue(4);
      assertTrue(Math.abs(thumbPosition - THUMB_CLEAR_POSITION) < 2.0f,
                 String.format("Thumb position not near clear position. pos=%.3f clear=%.3f",
                               thumbPosition, THUMB_CLEAR_POSITION));

      // Stage 0 fingers for POWER grip: indices 0–3 should have moved upward from initial 30
      for (int i = 0; i < 4; i++)
      {
         float fingerPos = hand.getCommandValue(i);
         assertTrue(fingerPos > initialPositions[i],
                    String.format("Finger %d should have started closing. initial=%.3f pos=%.3f",
                                  i, initialPositions[i], fingerPos));
      }

      // Pinky (index 5) should not move in stage 0 (remain near initial)
      float pinkyPosition = hand.getCommandValue(5);
      assertTrue(Math.abs(pinkyPosition - initialPositions[5]) < 2.0f,
                 String.format("Pinky should not move in stage 0. initial=%.3f pos=%.3f",
                               initialPositions[5], pinkyPosition));
   }

   /**
    * ASCII plot for 6 fingers at once.
    * Each time step is a line; each finger is plotted with a different marker,
    * and at the end of the line the numeric position and velocity for each finger
    * are printed concisely.
    */
   private void asciiPlotAllFingers(float[] times,
                                    float[][] fingerPositions,
                                    float[][] fingerVelocities,
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
      header.append(" | pos/vel per finger [0..5]");
      if (fingerVelocities != null)
         header.append(" | pos/vel per finger [0..5]");
      else
         header.append(" | pos per finger [0..5]");
      System.out.println(header);

      for (int stepIndex = 0; stepIndex < times.length; stepIndex++)
      {
         float time = times[stepIndex];

         StringBuilder line = new StringBuilder();
         line.append(String.format("%4d | %6.3f | ", stepIndex, time));

         // Canvas for markers
         char[] canvas = new char[plotWidth];
         for (int i = 0; i < plotWidth; i++)
            canvas[i] = ' ';

         // Place each finger marker based on position
         for (int fingerIndex = 0; fingerIndex < fingerPositions.length; fingerIndex++)
         {
            float value = fingerPositions[fingerIndex][stepIndex];
            float normalized = (value - minimumValue) / valueRange; // 0..1
            int markerIndex = Math.round(normalized * (plotWidth - 1));
            if (markerIndex < 0) markerIndex = 0;
            if (markerIndex >= plotWidth) markerIndex = plotWidth - 1;

            canvas[markerIndex] = markers[fingerIndex];
         }

         line.append(new String(canvas));
         line.append(" | ");

         // Append compact per-finger pos/vel: P0=xx.x,V0=yy.y;...
         for (int fingerIndex = 0; fingerIndex < fingerPositions.length; fingerIndex++)
         {
            float pos = fingerPositions[fingerIndex][stepIndex];
            if (fingerVelocities != null)
            {
               float vel = fingerVelocities[fingerIndex][stepIndex];
               line.append(String.format("F%d(%.3f,%.3f)", fingerIndex, pos, vel));
            }
            else
            {
               line.append(String.format("F%d(%.3f)", fingerIndex, pos));
            }
            if (fingerIndex < fingerPositions.length - 1)
               line.append(" ");
         }

         System.out.println(line);
      }
   }

}
