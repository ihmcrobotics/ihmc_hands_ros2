package us.ihmc.handsros2;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TrapezoidalTrajectory1DTest
{
   private static final float EPS = 1e-4f;

   @Test
   public void testSimpleMoveFromRest()
   {
      TrapezoidalTrajectory1D traj = new TrapezoidalTrajectory1D(0.0f, 0.5f, 1.0f);
      traj.setGoal(1.0f, 0.5f);

      float dt = 0.02f;
      int steps = 200;

      float[] times = new float[steps];
      float[] positions = new float[steps];
      float[] velocities = new float[steps];

      float t = 0.0f;
      float lastPos = traj.getCurrentPos();

      for (int i = 0; i < steps; i++)
      {
         float pos = traj.update(dt);
         float vel = traj.getCurrentVel();

         // Basic correctness checks
         assertTrue(pos >= lastPos - 1e-5f, "Position should be non-decreasing for positive move");
         assertTrue(Math.abs(vel) <= 0.5f + 1e-3f, "Velocity should be bounded by maxVel");

         times[i] = t;
         positions[i] = pos;
         velocities[i] = vel;

         lastPos = pos;
         t += dt;
      }

      // End-state checks
      assertEquals(1.0f, traj.getCurrentPos(), 5e-2f);
      assertEquals(0.0f, traj.getCurrentVel(), 5e-2f);

      // ASCII plots
      float minPos = Float.POSITIVE_INFINITY, maxPos = Float.NEGATIVE_INFINITY;
      float minVel = Float.POSITIVE_INFINITY, maxVel = Float.NEGATIVE_INFINITY;

      for (int i = 0; i < steps; i++)
      {
         minPos = Math.min(minPos, positions[i]);
         maxPos = Math.max(maxPos, positions[i]);
         minVel = Math.min(minVel, velocities[i]);
         maxVel = Math.max(maxVel, velocities[i]);
      }

      int width = 60;

      System.out.println("ASCII plot of position (testSimpleMoveFromRest):");
      asciiPlot(times, positions, minPos, maxPos, width);

      System.out.println();
      System.out.println("ASCII plot of velocity (testSimpleMoveFromRest):");
      asciiPlot(times, velocities, minVel, maxVel, width);
   }

   @Test
   public void testReverseDirectionMove()
   {
      TrapezoidalTrajectory1D traj = new TrapezoidalTrajectory1D(1.0f, 1.0f, 2.0f);
      traj.setGoal(0.0f, 1.0f);

      float dt = 0.01f;
      float lastPos = traj.getCurrentPos();

      for (int i = 0; i < 1000; i++)
      {
         float pos = traj.update(dt);
         float vel = traj.getCurrentVel();

         assertTrue(pos <= lastPos + 1e-5f, "Position should be non-increasing for negative move");
         if (Math.abs(vel) > EPS)
            assertTrue(vel < 0.0f, "Velocity should be negative while moving toward smaller goal");

         lastPos = pos;
      }

      assertEquals(0.0f, traj.getCurrentPos(), 5e-2f);
      assertEquals(0.0f, traj.getCurrentVel(), 5e-2f);
   }

   @Test
   public void testReplanningMidFlight()
   {
      TrapezoidalTrajectory1D traj = new TrapezoidalTrajectory1D(0.0f, 0.5f, 1.0f);
      traj.setGoal(1.0f, 0.5f);

      float dt = 0.01f;

      for (int i = 0; i < 100; i++)
         traj.update(dt);

      float posBeforeReplan = traj.getCurrentPos();
      float velBeforeReplan = traj.getCurrentVel();

      traj.setGoal(0.5f, 0.5f);

      assertEquals(posBeforeReplan, traj.getCurrentPos(), EPS);
      assertEquals(velBeforeReplan, traj.getCurrentVel(), EPS);

      for (int i = 0; i < 1000; i++)
         traj.update(dt);

      assertEquals(0.5f, traj.getCurrentPos(), 5e-2f);
      assertEquals(0.0f, traj.getCurrentVel(), 5e-2f);
   }

   @Test
   public void testAlreadyAtGoal()
   {
      TrapezoidalTrajectory1D traj = new TrapezoidalTrajectory1D(0.0f, 1.0f, 1.0f);
      traj.setGoal(0.0f, 1.0f);

      float dt = 0.01f;

      for (int i = 0; i < 100; i++)
      {
         float pos = traj.update(dt);
         float vel = traj.getCurrentVel();

         assertEquals(0.0f, pos, EPS);
         assertEquals(0.0f, vel, EPS);
      }
   }

   /**
    * ASCII "plot" of position and velocity over time.
    * Each line is a time sample; '*' shows the normalized value.
    */
   @Test
   public void asciiPlotSimpleTrajectory()
   {
      TrapezoidalTrajectory1D traj = new TrapezoidalTrajectory1D(0.0f, 0.5f, 1.0f);
      traj.setGoal(1.0f, 0.5f);

      float dt = 0.02f;
      int steps = 200;

      float[] times = new float[steps];
      float[] positions = new float[steps];
      float[] velocities = new float[steps];

      float t = 0.0f;
      for (int i = 0; i < steps; i++)
      {
         float pos = traj.update(dt);
         float vel = traj.getCurrentVel();

         times[i] = t;
         positions[i] = pos;
         velocities[i] = vel;

         t += dt;
      }

      // Find ranges for normalization
      float minPos = Float.POSITIVE_INFINITY, maxPos = Float.NEGATIVE_INFINITY;
      float minVel = Float.POSITIVE_INFINITY, maxVel = Float.NEGATIVE_INFINITY;

      for (int i = 0; i < steps; i++)
      {
         minPos = Math.min(minPos, positions[i]);
         maxPos = Math.max(maxPos, positions[i]);
         minVel = Math.min(minVel, velocities[i]);
         maxVel = Math.max(maxVel, velocities[i]);
      }

      int width = 60; // characters per line for each plot

      System.out.println("ASCII plot of position:");
      asciiPlot(times, positions, minPos, maxPos, width);

      System.out.println();
      System.out.println("ASCII plot of velocity:");
      asciiPlot(times, velocities, minVel, maxVel, width);
   }

   private void asciiPlot(float[] times, float[] values, float vMin, float vMax, int width)
   {
      float range = vMax - vMin;
      if (range < 1e-6f)
         range = 1e-6f; // avoid div by zero

      StringBuilder line0 = new StringBuilder();
      line0.append("Time | ");
      line0.append(" ".repeat(Math.max(0, width)));
      line0.append(" | ");
      System.out.println(line0);
      for (int i = 0; i < times.length; i++)
      {
         float t = times[i];
         float v = values[i];

         float normalized = (v - vMin) / range; // 0..1
         int idx = Math.round(normalized * (width - 1));

         StringBuilder line = new StringBuilder();
         line.append(String.format("%6.3f | ", t));

         for (int j = 0; j < width; j++)
         {
            if (j == idx)
               line.append('*');
            else
               line.append(' ');
         }

         line.append(String.format(" | %.5f", v));
         System.out.println(line);
      }
   }
}
