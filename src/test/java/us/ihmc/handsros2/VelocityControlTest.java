package us.ihmc.handsros2;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class VelocityControlTest
{
   @Test
   public void testNoisyPositionControlledByVelocityCommand()
   {
      // Goal and limits
      float goalPosition = 90.0f;
      float maximumVelocity = 30.0f;
      float maximumAcceleration = 200.0f;
      float deadzone = 1.0f;

      float timeStep = 0.01f;
      int numberOfSteps = 330;

      // Plant state (true, noise-free)
      float truePosition = 0.0f;
      float trueVelocity = 0.0f;

      // Logging arrays
      float[] times = new float[numberOfSteps];
      float[] measuredPositions = new float[numberOfSteps];
      float[] truePositions = new float[numberOfSteps];
      float[] commandedVelocities = new float[numberOfSteps];

      Random random = new Random(12345L);
      float noiseStandardDeviation = 0.005f; // small Gaussian noise on position

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         // Generate noisy measurement of position
         float noise = (float) (random.nextGaussian() * noiseStandardDeviation);
         float measuredPosition = truePosition + noise;

         // Compute velocity command from noisy position
         float velocityCommand = VelocityControlTools.computeVelocityCommand(measuredPosition,
                                                                             trueVelocity,
                                                                             goalPosition,
                                                                             maximumVelocity,
                                                                             maximumAcceleration,
                                                                             timeStep,
                                                                             deadzone);

         // Simple plant integration: apply commanded velocity directly
         // (you could also low-pass or use a separate accel/vel model if desired)
         trueVelocity = velocityCommand;
         truePosition += trueVelocity * timeStep;

         // Log
         times[stepIndex] = currentTime;
         measuredPositions[stepIndex] = measuredPosition;
         truePositions[stepIndex] = truePosition;
         commandedVelocities[stepIndex] = velocityCommand;

         currentTime += timeStep;
      }

      // ASCII plots
      float minTruePos = Float.POSITIVE_INFINITY, maxTruePos = Float.NEGATIVE_INFINITY;
      float minMeasPos = Float.POSITIVE_INFINITY, maxMeasPos = Float.NEGATIVE_INFINITY;
      float minVel = Float.POSITIVE_INFINITY, maxVel = Float.NEGATIVE_INFINITY;

      for (int i = 0; i < numberOfSteps; i++)
      {
         minTruePos = Math.min(minTruePos, truePositions[i]);
         maxTruePos = Math.max(maxTruePos, truePositions[i]);
         minMeasPos = Math.min(minMeasPos, measuredPositions[i]);
         maxMeasPos = Math.max(maxMeasPos, measuredPositions[i]);
         minVel = Math.min(minVel, commandedVelocities[i]);
         maxVel = Math.max(maxVel, commandedVelocities[i]);
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of true position (VelocityControlTest):");
      asciiPlot(times, truePositions, minTruePos, maxTruePos, plotWidth);

      System.out.println();
      System.out.println("ASCII plot of measured (noisy) position (VelocityControlTest):");
      asciiPlot(times, measuredPositions, minMeasPos, maxMeasPos, plotWidth);

      System.out.println();
      System.out.println("ASCII plot of commanded velocity (VelocityControlTest):");
      asciiPlot(times, commandedVelocities, minVel, maxVel, plotWidth);

      // Basic sanity checks: final true position should be near goal, and velocity near zero
      assertTrue(Math.abs(truePosition - goalPosition) < 5e-2f,
                 "Final true position should be close to the goal");
      assertTrue(Math.abs(trueVelocity) < 5e-2f,
                 "Final true velocity should be near zero");
   }

   @Test
   public void testMultipleGoalChangesWithNoisyPosition()
   {
      // Initial conditions and limits
      float goalPosition = 90.0f;
      float maximumVelocity = 70.0f;
      float maximumAcceleration = 200.0f;
      float deadzone = 1.0f;

      float timeStep = 0.01f;
      int numberOfSteps = 525; // 0–99: 50, 100–199: -50, 200–299: 30

      // Plant state (true, noise-free)
      float truePosition = 50.0f;  // start at 50
      float trueVelocity = 0.0f;

      float[] times = new float[numberOfSteps];
      float[] measuredPositions = new float[numberOfSteps];
      float[] truePositions = new float[numberOfSteps];
      float[] commandedVelocities = new float[numberOfSteps];

      Random random = new Random(98765L);
      float noiseStandardDeviation = 0.005f;

      float currentTime = 0.0f;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         // Change goal at specific ticks
         if (stepIndex == 100)
            goalPosition = -50.0f;
         else if (stepIndex == 300)
            goalPosition = 30.0f;
         else if (stepIndex == 400)
            goalPosition = 0.0f;

         float noise = (float) (random.nextGaussian() * noiseStandardDeviation);
         float measuredPosition = truePosition + noise;

         float velocityCommand = VelocityControlTools.computeVelocityCommand(
               measuredPosition,
               trueVelocity,
               goalPosition,
               maximumVelocity,
               maximumAcceleration,
               timeStep,
               deadzone);

         trueVelocity = velocityCommand;
         truePosition += trueVelocity * timeStep;

         times[stepIndex] = currentTime;
         measuredPositions[stepIndex] = measuredPosition;
         truePositions[stepIndex] = truePosition;
         commandedVelocities[stepIndex] = velocityCommand;

         currentTime += timeStep;
      }

      // Ranges for plotting
      float minTruePos = Float.POSITIVE_INFINITY, maxTruePos = Float.NEGATIVE_INFINITY;
      float minMeasPos = Float.POSITIVE_INFINITY, maxMeasPos = Float.NEGATIVE_INFINITY;
      float minVel = Float.POSITIVE_INFINITY, maxVel = Float.NEGATIVE_INFINITY;

      for (int i = 0; i < numberOfSteps; i++)
      {
         minTruePos = Math.min(minTruePos, truePositions[i]);
         maxTruePos = Math.max(maxTruePos, truePositions[i]);
         minMeasPos = Math.min(minMeasPos, measuredPositions[i]);
         maxMeasPos = Math.max(maxMeasPos, measuredPositions[i]);
         minVel = Math.min(minVel, commandedVelocities[i]);
         maxVel = Math.max(maxVel, commandedVelocities[i]);
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of true position (multiple goals):");
      asciiPlot(times, truePositions, minTruePos, maxTruePos, plotWidth);

      System.out.println();
      System.out.println("ASCII plot of measured (noisy) position (multiple goals):");
      asciiPlot(times, measuredPositions, minMeasPos, maxMeasPos, plotWidth);

      System.out.println();
      System.out.println("ASCII plot of commanded velocity (multiple goals):");
      asciiPlot(times, commandedVelocities, minVel, maxVel, plotWidth);

      // Sanity: final goal is 0; truePosition should be reasonably close and nearly stopped
      assertTrue(Math.abs(truePosition - 0.0f) < 1.0f,
                 "Final true position should be close to last goal (30)");
      assertTrue(Math.abs(trueVelocity) < 1.0f,
                 "Final true velocity should be near zero");
   }

   private void asciiPlot(float[] times,
                          float[] values,
                          float minimumValue,
                          float maximumValue,
                          int plotWidth)
   {
      float valueRange = maximumValue - minimumValue;
      if (valueRange < 1e-6f)
         valueRange = 1e-6f; // avoid division by zero

      StringBuilder headerLine = new StringBuilder();
      headerLine.append("Step | Time   | ");
      headerLine.append(" ".repeat(Math.max(0, plotWidth)));
      headerLine.append(" | Value");
      System.out.println(headerLine);

      for (int sampleIndex = 0; sampleIndex < times.length; sampleIndex++)
      {
         float time = times[sampleIndex];
         float value = values[sampleIndex];

         float normalized = (value - minimumValue) / valueRange; // 0..1
         int markerIndex = Math.round(normalized * (plotWidth - 1));

         StringBuilder line = new StringBuilder();
         line.append(String.format("%4d | %6.3f | ", sampleIndex, time));

         for (int columnIndex = 0; columnIndex < plotWidth; columnIndex++)
         {
            if (columnIndex == markerIndex)
               line.append('*');
            else
               line.append(' ');
         }

         line.append(String.format(" | %.5f", value));
         System.out.println(line);
      }
   }
}
