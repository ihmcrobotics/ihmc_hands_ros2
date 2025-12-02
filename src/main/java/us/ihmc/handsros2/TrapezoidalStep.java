package us.ihmc.handsros2;

/**
 * Stateless trapezoidal (or triangular) trajectory step.
 *
 * Given the current position/velocity, a goal position, and motion limits,
 * returns the next commanded position after deltaTime.
 *
 * The logic is:
 *  - If we are far from the goal, accelerate toward it (respecting maxVelocity, maxAcceleration).
 *  - If we are close enough that we must brake to stop at the goal, decelerate.
 *  - If we are at the goal within tolerance, hold position.
 *
 * The goal velocity is implicitly zero (stop at the goal).
 */
public class TrapezoidalStep
{
   public static float step(float currentPosition,
                            float currentVelocity,
                            float goalPosition,
                            float maxVelocity,
                            float maxAcceleration,
                            float deltaTime)
   {
      if (deltaTime <= 0.0f)
         return currentPosition;

      // Direction toward goal
      float positionError = goalPosition - currentPosition;
      float direction = positionError >= 0.0f ? 1.0f : -1.0f;

      float v    = currentVelocity;
      float vMax = Math.abs(maxVelocity);
      float aMax = Math.abs(maxAcceleration);

      // If already very close and slow, snap to goal (goalVelocity == 0)
      if (Math.abs(positionError) < 1e-4f && Math.abs(v) < 1e-3f)
         return goalPosition;

      // Compute stopping distance with max decel from current speed (to zero)
      float vAbs = Math.abs(v);
      float stoppingDistance = (vAbs * vAbs) / (2.0f * aMax);

      // If we are moving toward the goal and need to start braking to stop at goal
      boolean movingTowardGoal = Math.signum(v) == direction;
      boolean needToBrake = movingTowardGoal && (Math.abs(positionError) <= stoppingDistance + 1e-6f);

      float a;

      if (needToBrake)
      {
         // Decelerate toward zero velocity
         a = -aMax * direction; // opposite sign to velocity toward goal
      }
      else
      {
         // Accelerate toward goal until we hit max velocity
         if (Math.abs(v) < vMax || !movingTowardGoal)
            a = aMax * direction;
         else
            a = 0.0f;
      }

      // Integrate velocity
      float newVelocity = v + a * deltaTime;

      // Clamp velocity to max
      if (newVelocity >  vMax) newVelocity =  vMax;
      if (newVelocity < -vMax) newVelocity = -vMax;

      float newPosition = currentPosition + newVelocity * deltaTime;

      // If we would overshoot the goal, clamp to goal
      if (Math.signum(goalPosition - newPosition) != direction)
      {
         newPosition = goalPosition;
      }

      // Ability hand have signed int internal representations so 150 deg / 32767 = 0.00458 minimum command delta
      // TODO: Move to ability side
      float outputDelta = newPosition - currentPosition;
      if (Math.abs(outputDelta) > 0.0f && Math.abs(outputDelta) < 0.005f)
      {
         newPosition = currentPosition + 0.005f * direction;
      }

      return newPosition;
   }
}
