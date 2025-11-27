package us.ihmc.handsros2.trajectories;

/**
 * Generates and follows a 1‑D time‑parametrized motion profile between the current
 * position and a goal position under maximum velocity and acceleration limits.
 * <p>
 * Given the current position/velocity and a new goal, this class plans a
 * trapezoidal (or, for short moves, triangular) velocity profile with three
 * phases: acceleration, optional constant‑velocity cruise, and deceleration,
 * such that the motion starts at the current state and ends at the goal with
 * zero velocity. The plan is stored as phase durations and distances.
 * <p>
 * On each call to {@link #update(float)}, the internal trajectory time is
 * advanced by the provided time step and the corresponding ideal position and
 * velocity are computed analytically for the current phase. These values are
 * transformed back into the original motion direction and exposed as the
 * current commanded position and velocity. Once the planned total time is
 * reached, the class snaps exactly to the goal position, zeros the velocity,
 * and disables further motion until a new goal is set.
 * <p>
 * The {@link #reset(float, float)} method allows resynchronizing the internal
 * state to a measured joint position and velocity and clears any active
 * trajectory, so subsequent calls to {@link #update(float)} do nothing until
 * {@link #setGoal(float, float)} is invoked again.
 */
public class TrapezoidalTrajectory1D
{
   // Configuration (tune per joint)
   private final float maximumAcceleration;   // maximum acceleration (units/s^2)
   private float maximumVelocity;             // maximum velocity (units/s), can change per goal

   // State
   private float currentPosition;             // current position
   private float currentVelocity;             // current velocity

   private float goalPosition;                // target position

   // Planned profile (in transformed + direction)
   private int motionDirection;               // +1 or -1, direction of motion
   private float accelerationPhaseDuration;   // accel phase duration
   private float cruisePhaseDuration;         // cruise phase duration
   private float decelerationPhaseDuration;   // decel phase duration
   private float totalTrajectoryTime;         // ta + tc + td

   private float elapsedTime;                 // time since start of current plan

   // Internal planning data
   private float planStartPosition;
   // currentVelocity at plan start is implicit in velocityAtPlanStart (transformed)
   private float velocityAtPlanStart;         // in transformed coordinates (already clamped)
   private float peakVelocityMagnitude;
   private float accelerationEndPositionMagnitude;
   private float cruiseDistanceMagnitude;

   public TrapezoidalTrajectory1D(float initialPosition, float maximumVelocity, float maximumAcceleration)
   {
      this.currentPosition = initialPosition;
      this.currentVelocity = 0.0f;
      this.goalPosition = initialPosition;
      this.maximumVelocity = Math.abs(maximumVelocity);
      this.maximumAcceleration = Math.abs(maximumAcceleration);
      resetInternalState();
   }

   /**
    * Reset the trajectory to a new measured state without any active motion.
    * After this, update() will do nothing until setGoal(...) is called.
    */
   public void reset(float currentPosition, float currentVelocity)
   {
      this.currentPosition = currentPosition;
      this.currentVelocity = currentVelocity;

      // By default, treat the current state as "at goal"
      this.goalPosition = currentPosition;

      resetInternalState();
   }

   private void resetInternalState()
   {
      motionDirection = 0;
      accelerationPhaseDuration = 0.0f;
      cruisePhaseDuration = 0.0f;
      decelerationPhaseDuration = 0.0f;
      totalTrajectoryTime = 0.0f;
      elapsedTime = 0.0f;

      planStartPosition = currentPosition;
      velocityAtPlanStart = 0.0f;
      peakVelocityMagnitude = 0.0f;
      accelerationEndPositionMagnitude = 0.0f;
      cruiseDistanceMagnitude = 0.0f;
   }

   /**
    * Set a new goal position and maximum velocity.
    * Re-plans from current (position, velocity) to (goal, 0).
    */
   public void setGoal(float goalPosition, float maximumVelocity)
   {
      this.goalPosition = goalPosition;
      this.maximumVelocity = Math.abs(maximumVelocity);
      planProfile();
   }

   public float getCurrentPosition()
   {
      return currentPosition;
   }

   public float getCurrentVelocity()
   {
      return currentVelocity;
   }

   public float getGoalPosition()
   {
      return goalPosition;
   }

   /**
    * Advance the trajectory by deltaTime seconds.
    * Returns the new commanded position.
    */
   public float update(float deltaTime)
   {
      if (deltaTime <= 0.0f || motionDirection == 0)
         return currentPosition; // nothing to do

      elapsedTime += deltaTime;
      if (elapsedTime > totalTrajectoryTime)
         elapsedTime = totalTrajectoryTime;

      float timeSinceStart = elapsedTime;

      // Work in transformed coordinates (x0=0, v0 transformed, positive direction)
      float positionAlongProfile;
      float velocityAlongProfile;

      if (timeSinceStart <= accelerationPhaseDuration)
      {
         // Acceleration phase
         // x(t) = v0*t + 0.5*a*t^2
         // v(t) = v0 + a*t
         float acceleration = maximumAcceleration;
         float initialVelocityTransformed = velocityAtPlanStart;
         positionAlongProfile = initialVelocityTransformed * timeSinceStart
                                + 0.5f * acceleration * timeSinceStart * timeSinceStart;
         velocityAlongProfile = initialVelocityTransformed + acceleration * timeSinceStart;
      }
      else if (timeSinceStart <= accelerationPhaseDuration + cruisePhaseDuration)
      {
         // Cruise phase
         float cruiseTime = timeSinceStart - accelerationPhaseDuration;
         float peakVelocity = peakVelocityMagnitude; // precomputed during plan
         float accelerationEndPosition = accelerationEndPositionMagnitude;
         positionAlongProfile = accelerationEndPosition + peakVelocity * cruiseTime;
         velocityAlongProfile = peakVelocity;
      }
      else
      {
         // Deceleration phase
         float decelerationTime = timeSinceStart - accelerationPhaseDuration - cruisePhaseDuration;
         float peakVelocity = peakVelocityMagnitude;
         float acceleration = -maximumAcceleration;
         float accelerationEndPosition = accelerationEndPositionMagnitude;
         float cruiseDistance = cruiseDistanceMagnitude;
         // Start of decel: position and velocity
         float decelerationStartPosition = accelerationEndPosition + cruiseDistance;
         float decelerationStartVelocity = peakVelocity;
         positionAlongProfile = decelerationStartPosition
                                + decelerationStartVelocity * decelerationTime
                                + 0.5f * acceleration * decelerationTime * decelerationTime;
         velocityAlongProfile = decelerationStartVelocity + acceleration * decelerationTime;
      }

      // Transform back to world frame
      float worldStartPosition = planStartPosition;
      currentPosition = worldStartPosition + motionDirection * positionAlongProfile;
      currentVelocity = motionDirection * velocityAlongProfile;

      // If done, snap to goal and stop
      if (elapsedTime >= totalTrajectoryTime - 1e-6f)
      {
         currentPosition = goalPosition;
         currentVelocity = 0.0f;
         motionDirection = 0;
      }

      return currentPosition;
   }

   private void planProfile()
   {
      // Compute direction and distance
      float positionError = goalPosition - currentPosition;
      if (Math.abs(positionError) < 1e-6f && Math.abs(currentVelocity) < 1e-6f)
      {
         // Already there; no motion
         resetInternalState();
         return;
      }

      motionDirection = positionError >= 0.0f ? 1 : -1;

      // Transform initial state
      planStartPosition = currentPosition;
      float finalPositionTransformed = Math.abs(positionError);
      float initialVelocityTransformed = motionDirection * currentVelocity;

      // Clamp initial velocity to [-maximumVelocity, maximumVelocity]
      if (initialVelocityTransformed > maximumVelocity) initialVelocityTransformed = maximumVelocity;
      if (initialVelocityTransformed < -maximumVelocity) initialVelocityTransformed = -maximumVelocity;

      // Store the (possibly clamped) value for use in update()
      velocityAtPlanStart = initialVelocityTransformed;

      // Compute peak velocity needed if we use full accel/decel
      float velocityLimit = maximumVelocity;

      // Distance required for accel to velocityLimit, then decel to 0.
      float accelerationDurationCandidate =
            Math.max((velocityLimit - velocityAtPlanStart) / maximumAcceleration, 0.0f);
      float decelerationDurationCandidate = velocityLimit / maximumAcceleration;
      float accelerationDistance =
            velocityAtPlanStart * accelerationDurationCandidate
            + 0.5f * maximumAcceleration * accelerationDurationCandidate * accelerationDurationCandidate;
      float decelerationDistance =
            velocityLimit * decelerationDurationCandidate
            - 0.5f * maximumAcceleration * decelerationDurationCandidate * decelerationDurationCandidate;
      float minimumDistanceForTrapezoid = accelerationDistance + decelerationDistance;

      if (minimumDistanceForTrapezoid < 0.0f)
         minimumDistanceForTrapezoid = 0.0f;

      if (finalPositionTransformed >= minimumDistanceForTrapezoid)
      {
         // Trapezoidal profile: accel -> cruise -> decel
         accelerationPhaseDuration = accelerationDurationCandidate;
         decelerationPhaseDuration = decelerationDurationCandidate;
         float remainingDistance = finalPositionTransformed - minimumDistanceForTrapezoid;
         float cruiseVelocity = velocityLimit;
         cruisePhaseDuration = remainingDistance / cruiseVelocity;

         peakVelocityMagnitude = velocityLimit;
         accelerationEndPositionMagnitude = accelerationDistance;
         cruiseDistanceMagnitude = remainingDistance;

         totalTrajectoryTime = accelerationPhaseDuration + cruisePhaseDuration + decelerationPhaseDuration;
      }
      else
      {
         // Triangle profile: accelerate then decelerate without cruise
         // Solve for peak velocity such that distance matches finalPositionTransformed.
         float term =
               2.0f * maximumAcceleration * finalPositionTransformed
               + velocityAtPlanStart * velocityAtPlanStart;
         float peakVelocity = (float) Math.sqrt(Math.max(term / 2.0f, 0.0f));

         // Limit peakVelocity to velocityLimit just in case
         if (peakVelocity > velocityLimit)
            peakVelocity = velocityLimit;

         accelerationPhaseDuration =
               Math.max((peakVelocity - velocityAtPlanStart) / maximumAcceleration, 0.0f);
         decelerationPhaseDuration = peakVelocity / maximumAcceleration;
         cruisePhaseDuration = 0.0f;

         // Distances
         float accelerationDistanceTriangle =
               velocityAtPlanStart * accelerationPhaseDuration
               + 0.5f * maximumAcceleration * accelerationPhaseDuration * accelerationPhaseDuration;

         peakVelocityMagnitude = peakVelocity;
         accelerationEndPositionMagnitude = accelerationDistanceTriangle;
         cruiseDistanceMagnitude = 0.0f; // no cruise

         totalTrajectoryTime = accelerationPhaseDuration + decelerationPhaseDuration;
      }

      elapsedTime = 0.0f;
   }
}
