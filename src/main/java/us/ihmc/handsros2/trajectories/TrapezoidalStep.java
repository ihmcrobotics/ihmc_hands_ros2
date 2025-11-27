package us.ihmc.handsros2.trajectories;

/**
 * Stateless trapezoidal (or triangular) trajectory step.
 *
 * Given the current position/velocity, a goal position (and optional goal velocity),
 * and motion limits, returns the next commanded position after deltaTime.
 *
 * The logic is:
 *  - If we are far from the goal, accelerate toward it (respecting maxVelocity, maxAcceleration).
 *  - If we are close enough that we must brake to stop at the goal, decelerate.
 *  - If we are at the goal within tolerance, hold position and zero velocity.
 */
public class TrapezoidalStep
{
   public static float step(float currentPosition,
                            float currentVelocity,
                            float goalPosition,
                            float goalVelocity,     // usually 0
                            float maxVelocity,
                            float maxAcceleration,
                            float deltaTime)
   {
      if (deltaTime <= 0.0f)
         return currentPosition;

      // Direction toward goal
      float positionError = goalPosition - currentPosition;
      float direction = positionError >= 0.0f ? 1.0f : -1.0f;

      float v = currentVelocity;
      float vMax = Math.abs(maxVelocity);
      float aMax = Math.abs(maxAcceleration);

      // If already very close and slow, snap to goal
      if (Math.abs(positionError) < 1e-4f && Math.abs(v - goalVelocity) < 1e-3f)
         return goalPosition;

      // Desired sign of velocity to move toward goal
      float vSign = direction;

      // Compute stopping distance with max decel from current speed
      float vAbs = Math.abs(v);
      float stoppingDistance = (vAbs * vAbs) / (2.0f * aMax);

      // If we are moving toward the goal and need to start braking to stop at goal
      boolean movingTowardGoal = Math.signum(v) == vSign;
      boolean needToBrake = movingTowardGoal && (Math.abs(positionError) <= stoppingDistance + 1e-6f);

      float a;

      if (needToBrake)
      {
         // Decelerate toward zero (and then goalVelocity if you extend this)
         a = -aMax * vSign; // opposite sign to velocity toward goal
      }
      else
      {
         // Accelerate toward goal until we hit max velocity
         if (Math.abs(v) < vMax || !movingTowardGoal)
            a = aMax * vSign;
         else
            a = 0.0f;
      }

      // Integrate
      float newVelocity = v + a * deltaTime;

      // Clamp velocity to max
      if (newVelocity >  vMax) newVelocity =  vMax;
      if (newVelocity < -vMax) newVelocity = -vMax;

      float newPosition = currentPosition + newVelocity * deltaTime;

      // If we would overshoot the goal, clamp to goal and zero velocity
      if (Math.signum(goalPosition - newPosition) != direction)
      {
         newPosition = goalPosition;
      }

      return newPosition;
   }
}
