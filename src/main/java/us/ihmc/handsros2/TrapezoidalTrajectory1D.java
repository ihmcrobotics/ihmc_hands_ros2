package us.ihmc.handsros2;

public class TrapezoidalTrajectory1D
{
   // Configuration (tune per joint)
   private final float maxAcc;   // maximum acceleration (units/s^2)
   private float maxVel;         // maximum velocity (units/s), can change per goal

   // State
   private float currentPos;     // current position
   private float currentVel;     // current velocity

   private float goalPos;        // target position

   // Planned profile (in transformed + direction)
   private int dir;              // +1 or -1, direction of motion
   private float ta;             // accel phase duration
   private float tc;             // cruise phase duration
   private float td;             // decel phase duration
   private float totalTime;      // ta + tc + td

   private float tElapsed;       // time since start of current plan

   public TrapezoidalTrajectory1D(float initialPos, float maxVel, float maxAcc)
   {
      this.currentPos = initialPos;
      this.currentVel = 0.0f;
      this.goalPos = initialPos;
      this.maxVel = Math.abs(maxVel);
      this.maxAcc = Math.abs(maxAcc);
      this.dir = 0;
      this.ta = this.tc = this.td = this.totalTime = 0.0f;
      this.tElapsed = 0.0f;
   }

   /**
    * Set a new goal position and max velocity.
    * Re-plans from current (pos, vel) to (goal, 0).
    */
   public void setGoal(float goalPos, float maxVel)
   {
      this.goalPos = goalPos;
      this.maxVel = Math.abs(maxVel);
      planProfile();
   }

   public float getCurrentPos()
   {
      return currentPos;
   }

   public float getCurrentVel()
   {
      return currentVel;
   }

   public float getGoalPos()
   {
      return goalPos;
   }

   /**
    * Advance the trajectory by dt seconds.
    * Returns the new commanded position.
    */
   public float update(float dt)
   {
      if (dt <= 0.0f || dir == 0)
         return currentPos; // nothing to do

      tElapsed += dt;
      if (tElapsed > totalTime)
         tElapsed = totalTime;

      float t = tElapsed;

      // Work in transformed coordinates (x0=0, v0 transformed, positive direction)
      float pos = 0.0f;
      float vel = 0.0f;

      if (t <= ta)
      {
         // Acceleration phase
         // x(t) = v0*t + 0.5*a*t^2
         // v(t) = v0 + a*t
         float a = maxAcc;
         float v0 = dir * currentVelAtPlanStart; // see note below
         pos = v0 * t + 0.5f * a * t * t;
         vel = v0 + a * t;
      }
      else if (t <= ta + tc)
      {
         // Cruise phase
         float tCruise = t - ta;
         float vPeak = vPeakMag; // precomputed during plan
         float xAccelEnd = xAccelEndMag;
         pos = xAccelEnd + vPeak * tCruise;
         vel = vPeak;
      }
      else
      {
         // Deceleration phase
         float tDecel = t - ta - tc;
         float vFinal = 0.0f;
         float vPeak = vPeakMag;
         float a = -maxAcc;
         float xAccelEnd = xAccelEndMag;
         float xCruise = xCruiseMag;
         // Start of decel: position and velocity
         float xDecelStart = xAccelEnd + xCruise;
         float vDecelStart = vPeak;
         pos = xDecelStart + vDecelStart * tDecel + 0.5f * a * tDecel * tDecel;
         vel = vDecelStart + a * tDecel;
      }

      // Transform back to world frame
      float x0World = planStartPos;
      currentPos = x0World + dir * pos;
      currentVel = dir * vel;

      return currentPos;
   }

   // Internal planning data
   private float planStartPos;
   private float planStartVel;
   private float currentVelAtPlanStart; // in transformed coordinates
   private float vPeakMag;
   private float xAccelEndMag;
   private float xCruiseMag;

   private void planProfile()
   {
      // Compute direction and distance
      float dx = goalPos - currentPos;
      if (Math.abs(dx) < 1e-6f && Math.abs(currentVel) < 1e-6f)
      {
         // Already there; no motion
         dir = 0;
         ta = tc = td = totalTime = 0.0f;
         tElapsed = 0.0f;
         return;
      }

      dir = dx >= 0.0f ? 1 : -1;

      // Transform initial state
      planStartPos = currentPos;
      planStartVel = currentVel;
      float x0 = 0.0f;
      float xf = Math.abs(dx);
      float v0 = dir * currentVel;
      float vf = 0.0f; // want to stop at goal

      currentVelAtPlanStart = v0;

      // Clamp initial velocity to [-maxVel, maxVel]
      if (v0 > maxVel) v0 = maxVel;
      if (v0 < -maxVel) v0 = -maxVel;

      // Compute peak velocity needed if we use full accel/decel
      // First, distance to reach maxVel from v0:
      // dv = v_max - v0, t_a = dv / a, x_a = v0*t_a + 0.5*a*t_a^2
      float vMax = maxVel;

      // Distance required for accel to vMax, then decel to 0.
      float taCandidate = Math.max((vMax - v0) / maxAcc, 0.0f);
      float tdCandidate = vMax / maxAcc;
      float xAccel = v0 * taCandidate + 0.5f * maxAcc * taCandidate * taCandidate;
      float xDecel = vMax * tdCandidate - 0.5f * maxAcc * tdCandidate * tdCandidate;
      float xMinForTrapezoid = xAccel + xDecel;

      if (xMinForTrapezoid < 0.0f) xMinForTrapezoid = 0.0f;

      if (xf >= xMinForTrapezoid)
      {
         // Trapezoidal profile: accel -> cruise -> decel
         ta = taCandidate;
         td = tdCandidate;
         float xRemaining = xf - xMinForTrapezoid;
         float vCruise = vMax;
         tc = xRemaining / vCruise;

         vPeakMag = vMax;
         xAccelEndMag = xAccel;
         xCruiseMag = xRemaining;

         totalTime = ta + tc + td;
      }
      else
      {
         // Triangle profile: accelerate then decelerate without cruise
         // Solve for peak velocity vPeak such that distance matches xf.
         // Assume symmetric accel/decel to vPeak and back to 0:
         // x = ( (vPeak^2 - v0^2) / (2*a) ) + ( vPeak^2 / (2*a) )
         //    = (2*vPeak^2 - v0^2) / (2*a) = xf
         // => 2*vPeak^2 - v0^2 = 2*a*xf
         float term = 2.0f * maxAcc * xf + v0 * v0;
         float vPeak = (float) Math.sqrt(Math.max(term / 2.0f, 0.0f));

         // Limit vPeak to vMax just in case
         if (vPeak > vMax) vPeak = vMax;

         ta = Math.max((vPeak - v0) / maxAcc, 0.0f);
         td = vPeak / maxAcc;
         tc = 0.0f;

         // Distances
         float xA = v0 * ta + 0.5f * maxAcc * ta * ta;
         float xD = vPeak * td - 0.5f * maxAcc * td * td;

         vPeakMag = vPeak;
         xAccelEndMag = xA;
         xCruiseMag = 0.0f; // no cruise

         totalTime = ta + td;
      }

      tElapsed = 0.0f;
   }
}
