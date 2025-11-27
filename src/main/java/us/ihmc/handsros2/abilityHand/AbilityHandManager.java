package us.ihmc.handsros2.abilityHand;

import us.ihmc.handsros2.TrapezoidalTrajectory1D;

import static us.ihmc.handsros2.abilityHand.AbilityHand.ACTUATOR_COUNT;

/**
 * Manages higher-level control of an Ability Hand, including position, velocity,
 * velocity-to-position, and multi-stage grip operations.
 */
public class AbilityHandManager
{
   private static final float TOLERANCE = 1.0f;
   /** Trajectory configuration: tune acceleration per joint as needed (deg/s^2) */
   private static final float DEFAULT_MAXIMUM_ACCELERATION = 200.0f;

   private final AbilityHand hand;

   private AbilityHandControlMode controlMode = null;
   private AbilityHandControlMode previousControlMode = controlMode;

   private AbilityHandGrip grip = null;
   private AbilityHandGrip previousGrip = null;
   private int gripStage = Integer.MAX_VALUE;

   private final float[] goalPositions;
   private final float[] goalVelocities;

   /** Per-finger position trajectories used in POSITION, VEL_TO_POS, and GRIP modes */
   private final TrapezoidalTrajectory1D[] fingerTrajectories;

   /** Track previous time for computing dt */
   private long previousTimeNanos = -1L;
   private float dt;
   private boolean initialized = false;

   /**
    * Creates a new AbilityHandManager with default goal positions and velocities.
    *
    * @param hand the AbilityHand implementation for low-level control
    */
   public AbilityHandManager(AbilityHand hand)
   {
      this.hand = hand;

      goalPositions = new float[] {30.0f, 30.0f, 30.0f, 30.0f, 30.0f, -30.0f};
      goalVelocities = new float[] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

      fingerTrajectories = new TrapezoidalTrajectory1D[ACTUATOR_COUNT];
   }

   void initialize()
   {
      initialized = true;
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         goalPositions[i] = hand.getActuatorPosition(i);
         goalVelocities[i] = 30.0f;

         fingerTrajectories[i] = new TrapezoidalTrajectory1D(goalPositions[i], Math.abs(goalVelocities[i]), DEFAULT_MAXIMUM_ACCELERATION);
         fingerTrajectories[i].reset(hand.getActuatorPosition(i), hand.getActuatorVelocity(i));
      }
   }

   /**
    * Updates the hand commands based on the desired values set in this manager. Should be called periodically.
    */
   public void update()
   {
      long nowNanos = System.nanoTime();

      if (previousTimeNanos > 0L)
      {
         long deltaNanos = nowNanos - previousTimeNanos;
         float dt = Math.min(0.1f, deltaNanos * 1.0e-9f);

         update(dt);
      }

      previousTimeNanos = nowNanos;
   }

   /** Need to supply virtual dt in tests. */
   void update(float dt)
   {
      this.dt = dt;

      if (!initialized && hand.getActuatorPosition(0) != 0.0f)
         initialize();

      if (initialized)
      {
         // Only reset trajectories when entering POSITION from another mode
         if (controlMode == AbilityHandControlMode.POSITION && previousControlMode != AbilityHandControlMode.POSITION)
            for (int i = 0; i < ACTUATOR_COUNT; i++)
               fingerTrajectories[i].reset(hand.getActuatorPosition(i), hand.getActuatorVelocity(i));

         switch (controlMode)
         {
            case POSITION -> updatePositionControl();
            case VELOCITY -> updateVelocityControl();
            case GRIP -> updateGripControl();
         }

         previousControlMode = controlMode;
      }
   }

   /**
    * Updates hand in POSITION mode using per-finger trapezoidal trajectories.
    * goalPositions are the targets; goalVelocities set each finger's max velocity.
    */
   private void updatePositionControl()
   {
      hand.setCommandType(AbilityHandCommandType.POSITION);

      for (int actuatorIndex = 0; actuatorIndex < ACTUATOR_COUNT; actuatorIndex++)
      {
         TrapezoidalTrajectory1D trajectory = fingerTrajectories[actuatorIndex];
         trajectory.setGoal(goalPositions[actuatorIndex], Math.abs(goalVelocities[actuatorIndex]));
         float commandedPosition = trajectory.update(dt);
         hand.setCommandValue(actuatorIndex, commandedPosition);
      }
   }

   /** Updates hand to direct velocity control using goalVelocities. */
   private void updateVelocityControl()
   {
      hand.setCommandType(AbilityHandCommandType.VELOCITY);
      hand.setCommandValues(goalVelocities);
   }

   /** Performs multi-stage grip control, moving fingers sequentially through grip.stages. */
   private void updateGripControl()
   {
      // Handle entering GRIP mode vs switching grips while already in GRIP
      if (previousControlMode != AbilityHandControlMode.GRIP)
      {
         gripStage = 0;
         previousGrip = grip;

         // Re-sync trajectories to current hand state once when entering GRIP
         for (int actuatorIndex = 0; actuatorIndex < ACTUATOR_COUNT; actuatorIndex++)
         {
            fingerTrajectories[actuatorIndex].reset(hand.getActuatorPosition(actuatorIndex),
                                                    hand.getActuatorVelocity(actuatorIndex));
         }
      }
      else if (previousGrip != grip)
      {
         // New grip while already in GRIP: restart stage sequence but keep trajectories continuous
         gripStage = 0;
         previousGrip = grip;
      }

      // If we’re past the last stage, the grip is completed. No need to do anything
      if (gripStage >= grip.stages.length)
         return;

      hand.setCommandType(AbilityHandCommandType.POSITION);

      // Normal grip stages: move only the fingers in this stage toward their stage goals,
      // but update all trajectories every tick.
      int[] actuatorsToMove = grip.stages[gripStage];
      float[] stageGoalPositions = grip.positions[gripStage];

      boolean stageComplete = true;

      for (int actuatorIndex = 0; actuatorIndex < ACTUATOR_COUNT; actuatorIndex++)
      {
         boolean isActive = false;
         float desiredPosition = 0.0f;

         // Check if this finger is active in the current stage
         for (int i = 0; i < actuatorsToMove.length; i++)
         {
            if (actuatorIndex == actuatorsToMove[i])
            {
               isActive = true;
               desiredPosition = stageGoalPositions[i];
               break;
            }
         }

         TrapezoidalTrajectory1D trajectory = fingerTrajectories[actuatorIndex];
         float maximumVelocity = Math.abs(goalVelocities[actuatorIndex]);

         if (isActive)
         {
            trajectory.setGoal(desiredPosition, maximumVelocity);
         }
         else
         {
            // Not active: hold current trajectory position
            float holdPosition = trajectory.getCurrentPosition();
            trajectory.setGoal(holdPosition, maximumVelocity);
         }

         float commandedPosition = trajectory.update(dt);
         hand.setCommandValue(actuatorIndex, commandedPosition);

         if (isActive && Math.abs(commandedPosition - desiredPosition) >= TOLERANCE)
            stageComplete = false;
      }

      if (stageComplete)
      {
         // Snap active fingers to exact stage goal to avoid tiny residuals
         for (int i = 0; i < actuatorsToMove.length; i++)
         {
            int actuatorIndex = actuatorsToMove[i];
            float stageGoalPosition = stageGoalPositions[i];

            TrapezoidalTrajectory1D trajectory = fingerTrajectories[actuatorIndex];
            trajectory.reset(stageGoalPosition, 0.0f);
            hand.setCommandValue(actuatorIndex, stageGoalPosition);
         }

         gripStage++;
      }
   }

   /**
    * Get the hand object this manager manages.
    *
    * @return The hand this manager manages.
    */
   public AbilityHand getHand()
   {
      return hand;
   }

   /**
    * Sets the control mode for subsequent updates.
    *
    * @param controlMode desired control mode
    */
   public void setControlMode(AbilityHandControlMode controlMode)
   {
      this.controlMode = controlMode;
   }

   /**
    * Sets the grip pattern for subsequent grip control.
    *
    * @param grip desired Grip pattern
    */
   public void setGrip(AbilityHandGrip grip)
   {
      this.grip = grip;
   }

   /**
    * Sets a goal position for a specific actuator.
    *
    * @param index        actuator index (0 to ACTUATOR_COUNT-1)
    * @param goalPosition desired position in degrees
    */
   public void setGoalPosition(int index, float goalPosition)
   {
      goalPositions[index] = goalPosition;
   }

   /**
    * Sets goal positions for all actuators.
    *
    * @param goalPositions array of target positions, length ACTUATOR_COUNT
    */
   public void setGoalPositions(float[] goalPositions)
   {
      for (int i = 0; i < ACTUATOR_COUNT; ++i)
         setGoalPosition(i, goalPositions[i]);
   }

   /**
    * Sets a goal velocity for a specific actuator.
    *
    * @param index        actuator index (0 to ACTUATOR_COUNT-1)
    * @param goalVelocity desired velocity
    */
   public void setGoalVelocity(int index, float goalVelocity)
   {
      goalVelocities[index] = goalVelocity;
   }

   /**
    * Sets goal velocities for all actuators.
    *
    * @param goalVelocities array of target velocities, length ACTUATOR_COUNT
    */
   public void setGoalVelocities(float[] goalVelocities)
   {
      for (int i = 0; i < ACTUATOR_COUNT; ++i)
         setGoalVelocity(i, goalVelocities[i]);
   }

   /**
    * Retrieves the current grip stage.
    *
    * @return the current grip stage
    */
   public int getGripStage()
   {
      return gripStage;
   }

   /**
    * Retrieves the current goal position for a specific actuator.
    *
    * @param index
    * @return the current goal position for the specified actuator
    */
   public float getGoalPosition(int index)
   {
      return goalPositions[index];
   }

   /**
    * Retrieves the current goal velocity for a specific actuator.
    *
    * @param index
    * @return the current goal velocity for the specified actuator
    */
   public float getGoalVelocity(int index)
   {
      return goalVelocities[index];
   }
}
