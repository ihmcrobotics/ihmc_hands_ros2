package us.ihmc.handsros2;

/**
 * Utility functions for velocity-based motion control toward a position goal.
 * <p>
 * The main method in this class implements a simple, time-local trapezoidal
 * velocity strategy: on each control step it looks at the current position and
 * velocity relative to a desired goal, decides whether to accelerate,
 * cruise, or decelerate based on a stopping-distance check, and returns a
 * bounded velocity command in the direction of the goal.
 * <p>
 * Unlike an offline trajectory planner, this controller does not precompute
 * a full position profile. It instead recomputes the desired acceleration
 * and velocity every step from the latest measured state, which makes it
 * robust to disturbances and modeling error while still producing an
 * approximate accelerate–cruise–brake motion shape.
 */
public class VelocityControlTools
{
   /**
    * Computes a velocity command that drives a 1‑D system toward a goal
    * position using a trapezoidal-like velocity strategy.
    * <p>
    * Each call:
    * <ul>
    *   <li>Computes the signed distance to the goal and, if it is within the
    *       specified deadzone, commands zero velocity.</li>
    *   <li>Transforms the current velocity into the "toward-goal" frame
    *       (so motion toward the goal is positive).</li>
    *   <li>If the system is moving away from the goal, commands a braking
    *       acceleration up to zero velocity.</li>
    *   <li>Otherwise, computes the stopping distance for the current speed and
    *       compares it to the remaining distance:
    *       <ul>
    *         <li>If there is still plenty of distance left, accelerates toward
    *             the goal up to {@code maximumVelocity}.</li>
    *         <li>If the remaining distance is small, commands a deceleration
    *             so the system can come to rest near the goal.</li>
    *       </ul>
    *   </li>
    *   <li>Integrates the chosen acceleration over {@code deltaTime} to obtain
    *       a new "toward-goal" velocity, clamps it to
    *       {@code [0, maximumVelocity]}, and transforms it back to the
    *       original sign convention.</li>
    * </ul>
    * The method assumes a simple plant where the commanded velocity is
    * approximately realized over one control interval. It does not modify
    * position itself; callers are expected to integrate the returned velocity
    * into their own state estimate or plant model.
    *
    * @param currentPosition        current measured position of the joint or axis
    * @param currentVelocity        current measured velocity (same units as {@code maximumVelocity})
    * @param goalPosition           desired target position
    * @param maximumVelocity        maximum allowed speed magnitude when moving toward the goal
    * @param maximumAcceleration    maximum allowed acceleration magnitude used when
    *                               speeding up or braking; must be positive
    * @param deltaTime              control time step in seconds since the last call;
    *                               should be non-negative and reasonably small
    * @param deadzone               distance threshold around {@code goalPosition}
    *                               within which the controller commands zero velocity
    * @return a signed velocity command that moves the system toward
    *         {@code goalPosition}, bounded in magnitude by {@code maximumVelocity}
    * @throws IllegalArgumentException if {@code maximumAcceleration <= 0.0f}
    */
   public static float computeVelocityCommand(float currentPosition,
                                              float currentVelocity,
                                              float goalPosition,
                                              float maximumVelocity,
                                              float maximumAcceleration,
                                              float deltaTime,
                                              float deadzone)
   {
      if (maximumAcceleration <= 0.0f)
         throw new IllegalArgumentException("maximumAcceleration must be > 0.0");

      float distanceToGoal = goalPosition - currentPosition;
      float absoluteDistanceToGoal = Math.abs(distanceToGoal);

      // Stop if close enough
      if (absoluteDistanceToGoal < deadzone)
         return 0.0f;

      float motionDirection = distanceToGoal >= 0.0f ? 1.0f : -1.0f;

      // Transform velocity into "toward-goal" frame
      float velocityTowardGoal = motionDirection * currentVelocity;
      if (velocityTowardGoal < 0.0f)
      {
         // Moving away from the goal: brake first
         float accelerationTowardGoal = maximumAcceleration;
         float commandedVelocityTowardGoal = velocityTowardGoal + accelerationTowardGoal * deltaTime;
         if (commandedVelocityTowardGoal < 0.0f)
            commandedVelocityTowardGoal = 0.0f;
         return motionDirection * commandedVelocityTowardGoal;
      }

      float distanceTowardGoal = absoluteDistanceToGoal;

      // Stopping distance if we start braking now
      float stoppingDistance = (velocityTowardGoal * velocityTowardGoal) / (2.0f * maximumAcceleration);

      // Small margin so we do not cut it too close
      float safetyMargin = 1e-3f;

      float accelerationTowardGoal;
      if (distanceTowardGoal > stoppingDistance + safetyMargin)
      {
         // Still far: accelerate up to maximumVelocity
         if (velocityTowardGoal < maximumVelocity)
            accelerationTowardGoal = maximumAcceleration;
         else
            accelerationTowardGoal = 0.0f; // cruise
      }
      else
      {
         // Need to brake to hit the goal
         accelerationTowardGoal = -maximumAcceleration;
      }

      // Integrate one step and clamp
      float commandedVelocityTowardGoal = velocityTowardGoal + accelerationTowardGoal * deltaTime;
      if (commandedVelocityTowardGoal < 0.0f)
         commandedVelocityTowardGoal = 0.0f;
      if (commandedVelocityTowardGoal > maximumVelocity)
         commandedVelocityTowardGoal = maximumVelocity;

      return motionDirection * commandedVelocityTowardGoal;
   }
}
