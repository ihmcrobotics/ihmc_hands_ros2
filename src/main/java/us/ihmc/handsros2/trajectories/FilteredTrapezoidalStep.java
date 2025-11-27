package us.ihmc.handsros2.trajectories;

public class FilteredTrapezoidalStep
{
   // State: last filtered position
   private float filteredPosition;
   private boolean initialized = false;

   // Cutoff frequency and derived time constant (1 Hz default)
   private final float cutoffFrequencyHz;
   private final float timeConstant; // tau = 1 / (2*pi*fc)

   public FilteredTrapezoidalStep(float cutoffFrequencyHz)
   {
      this.cutoffFrequencyHz = cutoffFrequencyHz;
      this.timeConstant = 1.0f / (2.0f * (float) Math.PI * cutoffFrequencyHz);
   }

   /**
    * Call once per control cycle.
    *
    * @param currentPosition measured position
    * @param currentVelocity measured velocity
    * @param goalPosition    desired position
    * @param goalVelocity    desired velocity at goal (usually 0)
    * @param maxVelocity     max velocity
    * @param maxAcceleration max acceleration
    * @param deltaTime       dt in seconds
    * @return filtered commanded position
    */
   public float step(float currentPosition,
                     float currentVelocity,
                     float goalPosition,
                     float goalVelocity,
                     float maxVelocity,
                     float maxAcceleration,
                     float deltaTime)
   {
      if (deltaTime <= 0.0f)
      {
         // If never initialized, just pass through the current position
         if (!initialized)
         {
            filteredPosition = currentPosition;
            initialized = true;
         }
         return filteredPosition;
      }

      // Stateless trapezoidal step: raw command
      float rawCommand =
            TrapezoidalStep.step(currentPosition,
                                 currentVelocity,
                                 goalPosition,
                                 goalVelocity,
                                 maxVelocity,
                                 maxAcceleration,
                                 deltaTime);

      // Initialize filter state on first call
      if (!initialized)
      {
         filteredPosition = rawCommand;
         initialized = true;
         return filteredPosition;
      }

      // First-order low-pass with 1 Hz break (or configured fc)
      float alpha = deltaTime / (timeConstant + deltaTime);
      filteredPosition = alpha * rawCommand + (1.0f - alpha) * filteredPosition;

      return filteredPosition;
   }

   public float getFilteredPosition()
   {
      return filteredPosition;
   }

   public void reset(float position)
   {
      filteredPosition = position;
      initialized = true;
   }
}
