package us.ihmc.handsros2;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TrapezoidalTrajectory1DTest
{
   private static final float EPSILON = 1e-4f;

   @Test
   public void testSimpleMoveFromRest()
   {
      TrapezoidalTrajectory1D trajectory = new TrapezoidalTrajectory1D(0.0f, 0.5f, 1.0f);
      trajectory.setGoal(1.0f, 0.5f);

      float timeStep = 0.02f;
      int numberOfSteps = 200;

      float[] times = new float[numberOfSteps];
      float[] positions = new float[numberOfSteps];
      float[] velocities = new float[numberOfSteps];

      float currentTime = 0.0f;
      float lastPosition = trajectory.getCurrentPosition();

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         float position = trajectory.update(timeStep);
         float velocity = trajectory.getCurrentVelocity();

         // Basic correctness checks
         assertTrue(position >= lastPosition - 1e-5f, "Position should be non-decreasing for positive move");
         assertTrue(Math.abs(velocity) <= 0.5f + 1e-3f, "Velocity should be bounded by maximum velocity");

         times[stepIndex] = currentTime;
         positions[stepIndex] = position;
         velocities[stepIndex] = velocity;

         lastPosition = position;
         currentTime += timeStep;
      }

      // End-state checks
      assertEquals(1.0f, trajectory.getCurrentPosition(), 5e-2f);
      assertEquals(0.0f, trajectory.getCurrentVelocity(), 5e-2f);

      // ASCII plots
      float minimumPosition = Float.POSITIVE_INFINITY;
      float maximumPosition = Float.NEGATIVE_INFINITY;
      float minimumVelocity = Float.POSITIVE_INFINITY;
      float maximumVelocity = Float.NEGATIVE_INFINITY;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         minimumPosition = Math.min(minimumPosition, positions[stepIndex]);
         maximumPosition = Math.max(maximumPosition, positions[stepIndex]);
         minimumVelocity = Math.min(minimumVelocity, velocities[stepIndex]);
         maximumVelocity = Math.max(maximumVelocity, velocities[stepIndex]);
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of position (testSimpleMoveFromRest):");
      asciiPlot(times, positions, minimumPosition, maximumPosition, plotWidth);

      System.out.println();
      System.out.println("ASCII plot of velocity (testSimpleMoveFromRest):");
      asciiPlot(times, velocities, minimumVelocity, maximumVelocity, plotWidth);
   }

   @Test
   public void testReverseDirectionMove()
   {
      TrapezoidalTrajectory1D trajectory = new TrapezoidalTrajectory1D(1.0f, 1.0f, 2.0f);
      trajectory.setGoal(0.0f, 1.0f);

      float timeStep = 0.01f;
      int numberOfSteps = 170;

      float[] times = new float[numberOfSteps];
      float[] positions = new float[numberOfSteps];
      float[] velocities = new float[numberOfSteps];

      float currentTime = 0.0f;
      float lastPosition = trajectory.getCurrentPosition();

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         float position = trajectory.update(timeStep);
         float velocity = trajectory.getCurrentVelocity();

         // Assertions
         assertTrue(position <= lastPosition + 1e-5f, "Position should be non-increasing for negative move");
         if (Math.abs(velocity) > EPSILON)
            assertTrue(velocity < 0.0f, "Velocity should be negative while moving toward smaller goal");

         times[stepIndex] = currentTime;
         positions[stepIndex] = position;
         velocities[stepIndex] = velocity;

         lastPosition = position;
         currentTime += timeStep;
      }

      assertEquals(0.0f, trajectory.getCurrentPosition(), 5e-2f);
      assertEquals(0.0f, trajectory.getCurrentVelocity(), 5e-2f);

      // ASCII plots
      float minimumPosition = Float.POSITIVE_INFINITY;
      float maximumPosition = Float.NEGATIVE_INFINITY;
      float minimumVelocity = Float.POSITIVE_INFINITY;
      float maximumVelocity = Float.NEGATIVE_INFINITY;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         minimumPosition = Math.min(minimumPosition, positions[stepIndex]);
         maximumPosition = Math.max(maximumPosition, positions[stepIndex]);
         minimumVelocity = Math.min(minimumVelocity, velocities[stepIndex]);
         maximumVelocity = Math.max(maximumVelocity, velocities[stepIndex]);
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of position (testReverseDirectionMove):");
      asciiPlot(times, positions, minimumPosition, maximumPosition, plotWidth);

      System.out.println();
      System.out.println("ASCII plot of velocity (testReverseDirectionMove):");
      asciiPlot(times, velocities, minimumVelocity, maximumVelocity, plotWidth);
   }

   @Test
   public void testReplanningMidFlight()
   {
      TrapezoidalTrajectory1D trajectory = new TrapezoidalTrajectory1D(0.0f, 0.5f, 1.0f);
      trajectory.setGoal(5.0f, 0.5f);

      float timeStep = 0.01f;
      int totalSteps = 300; // first 100 steps toward 1.0, then 1100 after replanning

      float[] times = new float[totalSteps];
      float[] positions = new float[totalSteps];
      float[] velocities = new float[totalSteps];

      float currentTime = 0.0f;

      // Run for some time toward 1.0
      for (int stepIndex = 0; stepIndex < 100; stepIndex++)
      {
         float position = trajectory.update(timeStep);
         float velocity = trajectory.getCurrentVelocity();

         times[stepIndex] = currentTime;
         positions[stepIndex] = position;
         velocities[stepIndex] = velocity;

         currentTime += timeStep;
      }

      float positionBeforeReplan = trajectory.getCurrentPosition();
      float velocityBeforeReplan = trajectory.getCurrentVelocity();

      trajectory.setGoal(0.1f, 0.5f);

      assertEquals(positionBeforeReplan, trajectory.getCurrentPosition(), EPSILON);
      assertEquals(velocityBeforeReplan, trajectory.getCurrentVelocity(), EPSILON);

      // Continue after replanning
      for (int stepIndex = 100; stepIndex < totalSteps; stepIndex++)
      {
         float position = trajectory.update(timeStep);
         float velocity = trajectory.getCurrentVelocity();

         times[stepIndex] = currentTime;
         positions[stepIndex] = position;
         velocities[stepIndex] = velocity;

         currentTime += timeStep;
      }

      assertEquals(0.1f, trajectory.getCurrentPosition(), 5e-2f);
      assertEquals(0.0f, trajectory.getCurrentVelocity(), 5e-2f);

      // ASCII plots
      float minimumPosition = Float.POSITIVE_INFINITY;
      float maximumPosition = Float.NEGATIVE_INFINITY;
      float minimumVelocity = Float.POSITIVE_INFINITY;
      float maximumVelocity = Float.NEGATIVE_INFINITY;

      for (int stepIndex = 0; stepIndex < totalSteps; stepIndex++)
      {
         minimumPosition = Math.min(minimumPosition, positions[stepIndex]);
         maximumPosition = Math.max(maximumPosition, positions[stepIndex]);
         minimumVelocity = Math.min(minimumVelocity, velocities[stepIndex]);
         maximumVelocity = Math.max(maximumVelocity, velocities[stepIndex]);
      }

      int plotWidth = 60;

      System.out.println("ASCII plot of position (testReplanningMidFlight):");
      asciiPlot(times, positions, minimumPosition, maximumPosition, plotWidth);

      System.out.println();
      System.out.println("ASCII plot of velocity (testReplanningMidFlight):");
      asciiPlot(times, velocities, minimumVelocity, maximumVelocity, plotWidth);
   }


   @Test
   public void testAlreadyAtGoal()
   {
      TrapezoidalTrajectory1D trajectory = new TrapezoidalTrajectory1D(0.0f, 1.0f, 1.0f);
      trajectory.setGoal(0.0f, 1.0f);

      float timeStep = 0.01f;

      for (int stepIndex = 0; stepIndex < 100; stepIndex++)
      {
         float position = trajectory.update(timeStep);
         float velocity = trajectory.getCurrentVelocity();

         assertEquals(0.0f, position, EPSILON);
         assertEquals(0.0f, velocity, EPSILON);
      }
   }

   /**
    * ASCII "plot" of position and velocity over time.
    * Each line is a time sample; '*' shows the normalized value.
    */
   @Test
   public void asciiPlotSimpleTrajectory()
   {
      TrapezoidalTrajectory1D trajectory = new TrapezoidalTrajectory1D(0.0f, 0.5f, 1.0f);
      trajectory.setGoal(1.0f, 0.5f);

      float timeStep = 0.02f;
      int numberOfSteps = 200;

      float[] times = new float[numberOfSteps];
      float[] positions = new float[numberOfSteps];
      float[] velocities = new float[numberOfSteps];

      float currentTime = 0.0f;
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         float position = trajectory.update(timeStep);
         float velocity = trajectory.getCurrentVelocity();

         times[stepIndex] = currentTime;
         positions[stepIndex] = position;
         velocities[stepIndex] = velocity;

         currentTime += timeStep;
      }

      // Find ranges for normalization
      float minimumPosition = Float.POSITIVE_INFINITY;
      float maximumPosition = Float.NEGATIVE_INFINITY;
      float minimumVelocity = Float.POSITIVE_INFINITY;
      float maximumVelocity = Float.NEGATIVE_INFINITY;

      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         minimumPosition = Math.min(minimumPosition, positions[stepIndex]);
         maximumPosition = Math.max(maximumPosition, positions[stepIndex]);
         minimumVelocity = Math.min(minimumVelocity, velocities[stepIndex]);
         maximumVelocity = Math.max(maximumVelocity, velocities[stepIndex]);
      }

      int plotWidth = 60; // characters per line for each plot

      System.out.println("ASCII plot of position:");
      asciiPlot(times, positions, minimumPosition, maximumPosition, plotWidth);

      System.out.println();
      System.out.println("ASCII plot of velocity:");
      asciiPlot(times, velocities, minimumVelocity, maximumVelocity, plotWidth);
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
