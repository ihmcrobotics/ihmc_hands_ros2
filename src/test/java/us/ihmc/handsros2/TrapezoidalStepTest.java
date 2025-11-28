package us.ihmc.handsros2;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TrapezoidalStepTest
{
   @Test
   public void testLongTrapezoidAsciiPlot()
   {
      // Distance much greater than (v^2)/(2a): profile should include cruise
      float x0 = 0;
      float v0 = 0;
      float xG = 2.0f;
      float vMax = 0.5f, aMax = 1.0f;
      float dt = 0.02f;
      int steps = 230;

      float[] times = new float[steps];
      float[] positions = new float[steps];
      float[] velocities = new float[steps];

      float x = x0, v = v0;

      for (int i = 0; i < steps; i++)
      {
         times[i] = i * dt;
         positions[i] = x;

         float prevX = x;
         x = TrapezoidalStep.step(x, v, xG, vMax, aMax, dt);
         v = (x - prevX) / dt;
         velocities[i] = v;
      }

      float minPos = x0;
      float maxPos = xG;

      float minVel = Float.POSITIVE_INFINITY;
      float maxVel = Float.NEGATIVE_INFINITY;
      for (float vv : velocities)
      {
         if (vv < minVel) minVel = vv;
         if (vv > maxVel) maxVel = vv;
      }

      System.out.println("=== Trapezoidal trajectory: position & velocity ===");
      asciiPlotPositionAndVelocity(times, positions, velocities, minPos, maxPos, minVel, maxVel, 40);

      assertEquals(xG, positions[steps - 1], 1e-4f);

      // 4) Peak velocity never exceeds vMax (within small tolerance)
      float observedMaxVel = 0.0f;
      for (float vv : velocities)
         observedMaxVel = Math.max(observedMaxVel, Math.abs(vv));
      assertTrue(observedMaxVel <= vMax + 1e-3f, "Velocity exceeded maxVelocity");
   }


   @Test
   public void testShortTriangularAsciiPlot()
   {
      // Distance short enough: only accel and decel, no cruise
      float x0 = 0;
      float v0 = 0;
      float xG = 0.15f;
      float vMax = 2f, aMax = 2f;
      float dt = 0.01f;
      int steps = 60;

      float[] times = new float[steps];
      float[] positions = new float[steps];
      float[] velocities = new float[steps];

      float x = x0, v = v0;
      for (int i = 0; i < steps; i++)
      {
         times[i] = i * dt;
         positions[i] = x;

         float prevX = x;
         x = TrapezoidalStep.step(x, v, xG, vMax, aMax, dt);
         v = (x - prevX) / dt;
         velocities[i] = v;
      }

      float minPos = x0;
      float maxPos = xG;

      float minVel = Float.POSITIVE_INFINITY;
      float maxVel = Float.NEGATIVE_INFINITY;
      for (float vv : velocities)
      {
         if (vv < minVel) minVel = vv;
         if (vv > maxVel) maxVel = vv;
      }

      System.out.println("=== Triangular trajectory: position & velocity ===");
      asciiPlotPositionAndVelocity(times, positions, velocities, minPos, maxPos, minVel, maxVel, 40);
   }

   @Test
   public void testReverseDirectionMidFlight()
   {
      // Start with positive velocity, goal behind
      float x0 = 0.8f;
      float v0 = 0.5f;
      float xG = 0.0f;
      float vMax = 0.5f, aMax = 1.0f;
      float dt = 0.02f;
      int steps = 150;

      float[] times = new float[steps];
      float[] positions = new float[steps];
      float[] velocities = new float[steps];

      float x = x0, v = v0;
      for (int i = 0; i < steps; i++)
      {
         times[i] = i * dt;
         positions[i] = x;

         float prevX = x;
         x = TrapezoidalStep.step(x, v, xG, vMax, aMax, dt);
         v = (x - prevX) / dt;
         velocities[i] = v;
      }

      float minPos = Float.POSITIVE_INFINITY;
      float maxPos = Float.NEGATIVE_INFINITY;
      for (float px : positions)
      {
         if (px < minPos) minPos = px;
         if (px > maxPos) maxPos = px;
      }

      float minVel = Float.POSITIVE_INFINITY;
      float maxVel = Float.NEGATIVE_INFINITY;
      for (float vv : velocities)
      {
         if (vv < minVel) minVel = vv;
         if (vv > maxVel) maxVel = vv;
      }

      System.out.println("=== Reverse, decel/turn trajectory: position & velocity ===");
      asciiPlotPositionAndVelocity(times, positions, velocities, minPos, maxPos, minVel, maxVel, 40);
   }

   @Test
   public void testMultipleGoalChanges()
   {
      // Start moving left, first goal far to the right
      float x0 = -1.0f;
      float v0 = -0.3f;
      float vMax = 1.0f;
      float aMax = 2.0f;
      float dt = 0.02f;
      int steps = 270;

      // Predefined sequence of goals, one every 50 ticks (5 changes)
      float[] goals = new float[] { 1.0f, -0.5f, 0.8f, -1.5f, 0.0f };
      int goalInterval = 50;

      float[] times = new float[steps];
      float[] positions = new float[steps];
      float[] velocities = new float[steps];

      float x = x0;
      float v = v0;
      float currentGoal = goals[0];

      for (int i = 0; i < steps; i++)
      {
         // Change goal every goalInterval steps, cycling through goals
         if (i > 0 && i % goalInterval == 0)
         {
            int goalIndex = (i / goalInterval);
            if (goalIndex < goals.length)
               currentGoal = goals[goalIndex];
         }

         times[i] = i * dt;
         positions[i] = x;

         float prevX = x;
         x = TrapezoidalStep.step(x, v, currentGoal, vMax, aMax, dt);
         v = (x - prevX) / dt;
         velocities[i] = v;
      }

      // Compute min/max position
      float minPos = Float.POSITIVE_INFINITY;
      float maxPos = Float.NEGATIVE_INFINITY;
      for (float px : positions)
      {
         if (px < minPos) minPos = px;
         if (px > maxPos) maxPos = px;
      }

      // Compute min/max velocity
      float minVel = Float.POSITIVE_INFINITY;
      float maxVel = Float.NEGATIVE_INFINITY;
      for (float vv : velocities)
      {
         if (vv < minVel) minVel = vv;
         if (vv > maxVel) maxVel = vv;
      }

      System.out.println("=== Multi-goal trajectory: position & velocity ===");
      asciiPlotPositionAndVelocity(times, positions, velocities, minPos, maxPos, minVel, maxVel, 40);

      // Basic sanity: positions should stay within a reasonable bound
      // (no numerical explosion)
      assertTrue(maxPos - minPos < 10.0f, "Trajectory range unexpectedly large");
   }


   @Test
   public void testAlreadyAtGoal()
   {
      float out = TrapezoidalStep.step(2.0f, 0.0f, 2.0f, 1.0f, 2.0f, 0.01f);
      assertEquals(2.0f, out, 1e-6f);
   }

   @Test
   public void testMaxVelocityClamped()
   {
      float x0 = 0.0f;
      float v0 = 0.0f;
      float xG = 10.0f;
      float vMax = 1.0f;
      float aMax = 100.0f;
      float dt = 0.05f;

      // Step should saturate velocity strongly
      float x1 = TrapezoidalStep.step(x0, v0, xG, vMax, aMax, dt);
      float v1 = (x1 - x0) / dt;
      assertTrue(Math.abs(v1 - vMax) < 1e-3f);
   }

   @Test
   public void testOvershootClamped()
   {
      // Fast motion, but shouldn't fly past the goal in a big dt
      float x0 = 0.9f, v0 = 2.0f, xG = 1.0f, vMax = 10.0f, aMax = 10.0f, dt = 0.1f;
      float x1 = TrapezoidalStep.step(x0, v0, xG, vMax, aMax, dt);
      assertEquals(xG, x1, 1e-6f);
   }

   private void asciiPlotPositionAndVelocity(float[] times,
                                             float[] positions,
                                             float[] velocities,
                                             float minPos,
                                             float maxPos,
                                             float minVel,
                                             float maxVel,
                                             int plotWidth)
   {
      float posRange = maxPos - minPos;
      if (posRange < 1e-6f)
         posRange = 1e-6f;

      float velRange = maxVel - minVel;
      if (velRange < 1e-6f)
         velRange = 1e-6f;

      String header = "Step |  Time  | "
                      + String.format("%-" + plotWidth + "s", "Position")
                      + " | "
                      + String.format("%-" + plotWidth + "s", "Velocity")
                      + " | pos, vel";
      System.out.println(header);

      for (int i = 0; i < times.length; i++)
      {
         float t = times[i];
         float p = positions[i];
         float v = velocities[i];

         float posNorm = (p - minPos) / posRange; // 0..1
         float velNorm = (v - minVel) / velRange; // 0..1

         int posIdx = Math.round(posNorm * (plotWidth - 1));
         int velIdx = Math.round(velNorm * (plotWidth - 1));

         StringBuilder line = new StringBuilder();
         line.append(String.format("%4d | %6.3f | ", i, t));

         // Position bar
         for (int c = 0; c < plotWidth; c++)
            line.append(c == posIdx ? '*' : ' ');

         line.append(" | ");

         // Velocity bar
         for (int c = 0; c < plotWidth; c++)
            line.append(c == velIdx ? '*' : ' ');

         line.append(String.format(" | %.5f, %.5f", p, v));
         System.out.println(line);
      }
   }
}
