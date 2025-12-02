package us.ihmc.handsros2.abilityHand;

import org.junit.jupiter.api.Test;
import us.ihmc.handsros2.HandType;
import us.ihmc.robotics.robotSide.RobotSide;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.handsros2.abilityHand.AbilityHand.ACTUATOR_COUNT;

public class AbilityHandTest
{
   @Test
   public void testPositionControl()
   {
      AbilityHand hand = new AbilityHand("", RobotSide.LEFT);
      hand.setControlMode(AbilityHandControlMode.POSITION);
      hand.setActuatorPositions(new float[] {30f, 30f, 30f, 30f, 30f, -30f});
      float[] goalPositions = {10f, 20f, 30f, 40f, 50f, -10f};
      hand.setGoalPositions(goalPositions);
      hand.setGoalVelocities(new float[] {30f, 30f, 30f, 30f, 30f, 30f});

      int numberOfSteps = 100;
      float timeStep = 0.01f;

      float[][] actuatorPositions = new float[ACTUATOR_COUNT][numberOfSteps];
      float[][] fingerVelocities = new float[ACTUATOR_COUNT][numberOfSteps];

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         hand.update(timeStep);

         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
         {
            float command = hand.getCommandValue(fingerIndex);
            hand.setActuatorPosition(fingerIndex, command);

            if (stepIndex > 0)
            {
               float prevPos = actuatorPositions[fingerIndex][stepIndex - 1];
               float velocity = (command - prevPos) * (1.0f / timeStep);
               hand.setActuatorVelocity(fingerIndex, velocity);
            }

            actuatorPositions[fingerIndex][stepIndex] = hand.getActuatorPosition(fingerIndex);
            fingerVelocities[fingerIndex][stepIndex] = hand.getActuatorVelocity(fingerIndex);
         }

         currentTime += timeStep;
      }

      // Compute global min/max for positions and velocities
      float minPos = Float.POSITIVE_INFINITY;
      float maxPos = Float.NEGATIVE_INFINITY;
      float minVel = Float.POSITIVE_INFINITY;
      float maxVel = Float.NEGATIVE_INFINITY;

      for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
      {
         for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
         {
            float p = actuatorPositions[fingerIndex][stepIndex];
            float v = fingerVelocities[fingerIndex][stepIndex];
            if (p < minPos) minPos = p;
            if (p > maxPos) maxPos = p;
            if (v < minVel) minVel = v;
            if (v > maxVel) maxVel = v;
         }
      }

      int plotWidth = 60;

      float tolerance = 1e-3f;

      System.out.printf("Asserting command type is POSITION, actual=%s%n", hand.getCommandType());
      assertEquals(AbilityHandCommandType.POSITION, hand.getCommandType());

      for (int i = 0; i < goalPositions.length; i++)
      {
         float target = goalPositions[i];
         float actual = hand.getCommandValue(i);
         System.out.printf("Asserting finger %d: target=%.3f actual=%.3f tol=%.4f%n",
                           i, target, actual, tolerance);
         assertEquals(target, actual, tolerance);
      }
   }

   @Test
   public void testVelocityControl()
   {
      AbilityHand hand = new AbilityHand("", RobotSide.LEFT);
      hand.setActuatorPositions(new float[] {30f, 30f, 30f, 30f, 30f, -30f});
      float[] goalPositions = {10f, 20f, 30f, 40f, 50f, -10f};
      hand.setGoalPositions(goalPositions);

      float[] targetVelocities = {10f, 20f, 30f, 40f, 50f, -60f};
      hand.setControlMode(AbilityHandControlMode.VELOCITY);
      hand.setGoalVelocities(targetVelocities);

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
         hand.update(timeStep);

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

      // Compute global min/max across all fingers for positions and velocities
      float minPos = Float.POSITIVE_INFINITY;
      float maxPos = Float.NEGATIVE_INFINITY;
      float minVel = Float.POSITIVE_INFINITY;
      float maxVel = Float.NEGATIVE_INFINITY;
      for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
      {
         for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
         {
            float p = fingerPositions[fingerIndex][stepIndex];
            float v = fingerVelocities[fingerIndex][stepIndex];
            if (p < minPos) minPos = p;
            if (p > maxPos) maxPos = p;
            if (v < minVel) minVel = v;
            if (v > maxVel) maxVel = v;
         }
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of all finger positions/velocities in VELOCITY mode:");
      asciiPlotAllFingers(times, fingerPositions, fingerVelocities, minPos, maxPos, minVel, maxVel, plotWidth);

      float tolerance = 1e-6f;

      System.out.printf("Asserting command type is VELOCITY, actual=%s%n", hand.getCommandType());
      assertEquals(AbilityHandCommandType.VELOCITY, hand.getCommandType());

      for (int i = 0; i < targetVelocities.length; i++)
      {
         float expected = targetVelocities[i];
         float actual = hand.getCommandValue(i);
         System.out.printf("Asserting finger %d velocity: expected=%.6f actual=%.6f tol=%.1e%n",
                           i, expected, actual, tolerance);
         assertEquals(expected, actual, tolerance);
      }
   }

   @Test
   public void testCloseGrip()
   {
      AbilityHand hand = new AbilityHand("24ABH000", RobotSide.LEFT);

      // Thumb (index 4) must clear first: simulate it at the clear position already
      float[] initialPositions = {0f, 2f, 7f, 15f, 90f, 10f};
      hand.setActuatorPositions(initialPositions);

      hand.setGoalVelocities(new float[] {80f, 80f, 80f, 80f, 80f, 80f});
      hand.setControlMode(AbilityHandControlMode.GRIP);
      hand.setGrip(AbilityHandGrip.CLOSE);

      int numberOfSteps = 170;
      float timeStep = 0.01f;

      float[] times = new float[numberOfSteps];
      float[][] actuatorPositions = new float[ACTUATOR_COUNT][numberOfSteps];
      float[][] fingerVelocities = new float[ACTUATOR_COUNT][numberOfSteps];

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         if (stepIndex == 130)
         {
            // Stage 0 fingers for POWER grip: indices 0–3 should have moved upward from initial positions
            for (int i = 0; i < 4; i++)
            {
               float fingerPos = hand.getCommandValue(i);
               assertTrue(fingerPos > initialPositions[i],
                          String.format("Finger %d should have started closing. initial=%.3f pos=%.3f",
                                        i, initialPositions[i], fingerPos));
            }
         }

         // Manager computes desired positions in GRIP mode
         hand.update(timeStep);

         times[stepIndex] = currentTime;

         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
         {
            float command = hand.getCommandValue(fingerIndex);
            hand.setActuatorPosition(fingerIndex, command);

            if (stepIndex > 0)
            {
               float prevPos = actuatorPositions[fingerIndex][stepIndex - 1];
               float velocity = (command - prevPos) * (1.0f / timeStep);
               hand.setActuatorVelocity(fingerIndex, velocity);
            }

            actuatorPositions[fingerIndex][stepIndex] = hand.getActuatorPosition(fingerIndex);
            fingerVelocities[fingerIndex][stepIndex] = hand.getActuatorVelocity(fingerIndex);
         }

         currentTime += timeStep;
      }

      // Normalize for plotting
      float minPos = Float.POSITIVE_INFINITY;
      float maxPos = Float.NEGATIVE_INFINITY;
      float minVel = Float.POSITIVE_INFINITY;
      float maxVel = Float.NEGATIVE_INFINITY;
      for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
      {
         for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
         {
            float p = actuatorPositions[fingerIndex][stepIndex];
            float v = fingerVelocities[fingerIndex][stepIndex];
            if (p < minPos) minPos = p;
            if (p > maxPos) maxPos = p;
            if (v < minVel) minVel = v;
            if (v > maxVel) maxVel = v;
         }
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of all finger positions/velocities in CLOSE grip:");
      asciiPlotAllFingers(times, actuatorPositions, fingerVelocities, minPos, maxPos, minVel, maxVel, plotWidth);

      // GRIP mode should be issuing POSITION commands
      System.out.printf("Asserting GRIP mode uses POSITION command type, actual=%s%n", hand.getCommandType());
      assertEquals(AbilityHandCommandType.POSITION, hand.getCommandType(),
                   "Expected POSITION command type but got " + hand.getCommandType());

      // Assert CLOSE grip final-stage target positions are achieved at the end
      float tolerance = 1e-1f;
      AbilityHandGrip grip = AbilityHandGrip.CLOSE;
      int lastStage = grip.stages.length - 1;
      int[] stageActuators = grip.stages[lastStage];
      float[] stageTargets = grip.positions[lastStage];

      int finalStep = numberOfSteps - 1;
      System.out.printf("Asserting final positions for CLOSE grip at step %d%n", finalStep);

      for (int i = 0; i < stageActuators.length; i++)
      {
         int actuatorIndex = stageActuators[i];
         float target = stageTargets[i];
         float actual = actuatorPositions[actuatorIndex][finalStep];

         System.out.printf("CLOSE grip, finger %d: target=%.3f actual=%.3f tol=%.3f%n",
                           actuatorIndex, target, actual, tolerance);
         assertEquals(target, actual, tolerance);
      }
   }

   @Test
   public void testMultipleGripsSequence()
   {
      AbilityHand hand = new AbilityHand("24ABH000", RobotSide.LEFT);
      // Start from some nontrivial configuration
      float[] initialPositions = {0f, 10f, 20f, 30f, 0f, 0f};
      hand.setActuatorPositions(initialPositions);

      hand.setGoalVelocities(new float[] {80f, 80f, 80f, 80f, 80f, 80f});
      hand.setControlMode(AbilityHandControlMode.GRIP);

      // Sequence of grips to test
      AbilityHandGrip[] gripSequence = new AbilityHandGrip[] {
            AbilityHandGrip.OPEN, AbilityHandGrip.RELAX, AbilityHandGrip.HOOK, AbilityHandGrip.CLOSE};
      int[] gripSwitchSteps = new int[] {0, 150, 400, 500}; // when to switch grips

      int numberOfSteps = 650;
      float timeStep = 0.01f;

      float[] times = new float[numberOfSteps];
      float[][] actuatorPositions = new float[ACTUATOR_COUNT][numberOfSteps];
      float[][] fingerVelocities = new float[ACTUATOR_COUNT][numberOfSteps];

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         // Choose grip based on schedule
         for (int seqIndex = gripSequence.length - 1; seqIndex >= 0; seqIndex--)
         {
            if (stepIndex >= gripSwitchSteps[seqIndex])
            {
               hand.setGrip(gripSequence[seqIndex]);
               break;
            }
         }

         // Run controller in GRIP mode
         hand.update(timeStep);

         times[stepIndex] = currentTime;

         for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
         {
            float command = hand.getCommandValue(fingerIndex);
            hand.setActuatorPosition(fingerIndex, command);

            if (stepIndex > 0)
            {
               float prevPos = actuatorPositions[fingerIndex][stepIndex - 1];
               float velocity = (command - prevPos) * (1.0f / timeStep);
               hand.setActuatorVelocity(fingerIndex, velocity);
            }

            actuatorPositions[fingerIndex][stepIndex] = hand.getActuatorPosition(fingerIndex);
            fingerVelocities[fingerIndex][stepIndex] = hand.getActuatorVelocity(fingerIndex);
         }

         currentTime += timeStep;
      }

      // Normalize for plotting
      float minPos = Float.POSITIVE_INFINITY;
      float maxPos = Float.NEGATIVE_INFINITY;
      float minVel = Float.POSITIVE_INFINITY;
      float maxVel = Float.NEGATIVE_INFINITY;
      for (int fingerIndex = 0; fingerIndex < ACTUATOR_COUNT; fingerIndex++)
      {
         for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
         {
            float p = actuatorPositions[fingerIndex][stepIndex];
            float v = fingerVelocities[fingerIndex][stepIndex];
            if (p < minPos) minPos = p;
            if (p > maxPos) maxPos = p;
            if (v < minVel) minVel = v;
            if (v > maxVel) maxVel = v;
         }
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of all finger positions/velocities across multiple grips:");
      asciiPlotAllFingers(times, actuatorPositions, fingerVelocities, minPos, maxPos, minVel, maxVel, plotWidth);

      // GRIP mode should be issuing POSITION commands at the end
      System.out.printf("Asserting GRIP mode uses POSITION command type at end, actual=%s%n", hand.getCommandType());
      assertEquals(AbilityHandCommandType.POSITION, hand.getCommandType(),
                   "Expected POSITION command type but got " + hand.getCommandType());

      // New: assert that at the end of each grip segment, the final stage target positions are reached
      float tolerance = 1e-1f; // looser tolerance to allow some discretization error

      for (int seqIndex = 0; seqIndex < gripSequence.length; seqIndex++)
      {
         AbilityHandGrip grip = gripSequence[seqIndex];
         int segmentEndStep;

         if (seqIndex < gripSequence.length - 1)
            segmentEndStep = gripSwitchSteps[seqIndex + 1] - 1;
         else
            segmentEndStep = numberOfSteps - 1;

         // Use the last stage of the grip (all fingers should be at these positions by the end)
         int lastStage = grip.stages.length - 1;
         int[] stageActuators = grip.stages[lastStage];
         float[] stageTargets = grip.positions[lastStage];

         System.out.printf("Asserting final positions for grip %s at step %d%n", grip, segmentEndStep);

         for (int i = 0; i < stageActuators.length; i++)
         {
            int actuatorIndex = stageActuators[i];
            float target = stageTargets[i];
            float actual = actuatorPositions[actuatorIndex][segmentEndStep];

            System.out.printf("Grip %s, finger %d: target=%.3f actual=%.3f tol=%.3f%n",
                              grip, actuatorIndex, target, actual, tolerance);
            assertEquals(target, actual, tolerance);
         }
      }
   }


   /**
    * ASCII plot for 6 fingers at once.
    * Each time step is a line; each finger's position is plotted on the left,
    * and each finger's velocity is plotted on the right, followed by numeric
    * positions and velocities.
    */
   private void asciiPlotAllFingers(float[] times,
                                    float[][] fingerPositions,
                                    float[][] fingerVelocities,
                                    float minPosition,
                                    float maxPosition,
                                    float minVelocity,
                                    float maxVelocity,
                                    int plotWidth)
   {
      char[] markers = {'0', '1', '2', '3', '4', '5'};

      float posRange = maxPosition - minPosition;
      if (posRange < 1e-6f)
         posRange = 1e-6f;

      float velRange = maxVelocity - minVelocity;
      if (velRange < 1e-6f)
         velRange = 1e-6f;

      String header = "Step | Time   | " + String.format("%-" + plotWidth + "s", "Position") + " | " + String.format("%-" + plotWidth + "s", "Velocity")
                      + " | pos[deg], vel[deg/s]";
      System.out.println(header);

      for (int stepIndex = 0; stepIndex < times.length; stepIndex++)
      {
         float time = times[stepIndex];

         StringBuilder line = new StringBuilder();
         line.append(String.format("%4d | %6.3f | ", stepIndex, time));

         // Position canvas
         char[] posCanvas = new char[plotWidth];
         for (int i = 0; i < plotWidth; i++)
            posCanvas[i] = ' ';

         // Velocity canvas
         char[] velCanvas = new char[plotWidth];
         for (int i = 0; i < plotWidth; i++)
            velCanvas[i] = ' ';

         // Place each finger marker based on position
         for (int fingerIndex = 0; fingerIndex < fingerPositions.length; fingerIndex++)
         {
            float pos = fingerPositions[fingerIndex][stepIndex];
            float posNorm = (pos - minPosition) / posRange; // 0..1
            int posIdx = Math.round(posNorm * (plotWidth - 1));
            if (posIdx < 0)
               posIdx = 0;
            if (posIdx >= plotWidth)
               posIdx = plotWidth - 1;
            posCanvas[posIdx] = markers[fingerIndex];
         }

         // Place each finger marker based on velocity
         for (int fingerIndex = 0; fingerIndex < fingerVelocities.length; fingerIndex++)
         {
            float vel = fingerVelocities[fingerIndex][stepIndex];
            float velNorm = (vel - minVelocity) / velRange; // 0..1
            int velIdx = Math.round(velNorm * (plotWidth - 1));
            if (velIdx < 0)
               velIdx = 0;
            if (velIdx >= plotWidth)
               velIdx = plotWidth - 1;
            velCanvas[velIdx] = markers[fingerIndex];
         }

         line.append(new String(posCanvas));
         line.append(" | ");
         line.append(new String(velCanvas));
         line.append(" | ");

         // Append numeric positions and velocities with %.1f precision
         for (int fingerIndex = 0; fingerIndex < fingerPositions.length; fingerIndex++)
         {
            float pos = fingerPositions[fingerIndex][stepIndex];
            float vel = fingerVelocities[fingerIndex][stepIndex];
            line.append(String.format("%d:%.1f/%.1f ", fingerIndex, pos, vel));
         }

         System.out.println(line);
      }
   }

   @Test
   public void testType()
   {
      AbilityHand testEZGripper = new AbilityHand("24ABH001", RobotSide.LEFT);
      assertEquals(HandType.ABILITY_HAND, testEZGripper.getType());
   }
}
