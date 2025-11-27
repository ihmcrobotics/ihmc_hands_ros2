package us.ihmc.handsros2.abilityHand;

import us.ihmc.handsros2.TrapezoidalTrajectory1D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import static us.ihmc.handsros2.abilityHand.AbilityHand.ACTUATOR_COUNT;

/**
 * Manages higher-level control of an Ability Hand, including position, velocity,
 * velocity-to-position, and multi-stage grip operations, with YoVariables.
 */
public class AbilityHandManager
{
   private static final float TOLERANCE = 1.0f;
   /** Trajectory configuration: tune acceleration per joint as needed (deg/s^2) */
   private static final float DEFAULT_MAXIMUM_ACCELERATION = 200.0f;

   private final AbilityHand hand;

   private final YoEnum<AbilityHandControlMode> controlMode;
   private AbilityHandControlMode previousControlMode = null;

   private final YoEnum<AbilityHandGrip> grip;
   private AbilityHandGrip previousGrip = null;
   private int gripStage = Integer.MAX_VALUE;

   private final YoDouble[] goalPositions = new YoDouble[ACTUATOR_COUNT];
   private final YoDouble[] goalVelocities = new YoDouble[ACTUATOR_COUNT];

   /** Per-finger position trajectories used in POSITION, VEL_TO_POS, and GRIP modes */
   private final TrapezoidalTrajectory1D[] fingerTrajectories;

   /** Track previous time for computing dt */
   private long previousTimeNanos = -1L;
   private float dt;
   private boolean initialized = false;

   /**
    * Creates a new AbilityHandManager with YoVariables for goals and modes.
    *
    * @param hand the AbilityHand implementation for low-level control
    */
   public AbilityHandManager(AbilityHand hand)
   {
      this(new YoRegistry("AbilityHandManager_" + hand.getIdentifier() + "_" + hand.getSide().name()), hand);
   }

   /**
    * Creates a new AbilityHandManager with YoVariables for goals and modes.
    *
    * @param registry YoRegistry to register YoVariables into
    * @param hand     the AbilityHand implementation for low-level control
    */
   public AbilityHandManager(YoRegistry registry, AbilityHand hand)
   {
      this.hand = hand;

      String prefix = hand.getSide().name() + getClass().getSimpleName();

      controlMode = new YoEnum<>(prefix + "ControlMode", registry, AbilityHandControlMode.class);
      grip = new YoEnum<>(prefix + "Grip", registry, AbilityHandGrip.class);

      // Initialize goal positions and velocities with previous defaults
      float[] initialPositions = new float[] {30.0f, 30.0f, 30.0f, 30.0f, 30.0f, -30.0f};
      float[] initialVelocities = new float[] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

      for (int i = 0; i < ACTUATOR_COUNT; ++i)
      {
         goalPositions[i] = new YoDouble(prefix + "GoalPosition" + i, registry);
         goalPositions[i].set(initialPositions[i]);

         goalVelocities[i] = new YoDouble(prefix + "GoalVelocity" + i, registry);
         goalVelocities[i].set(initialVelocities[i]);
      }

      fingerTrajectories = new TrapezoidalTrajectory1D[ACTUATOR_COUNT];
   }

   void initialize()
   {
      initialized = true;
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         goalPositions[i].set(hand.getActuatorPosition(i));
         goalVelocities[i].set(30.0f);

         fingerTrajectories[i] = new TrapezoidalTrajectory1D((float) goalPositions[i].getValue(),
                                                             Math.abs((float) goalVelocities[i].getValue()),
                                                             DEFAULT_MAXIMUM_ACCELERATION);
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
         AbilityHandControlMode currentControlMode = controlMode.getValue();

         // Only reset trajectories when entering POSITION from another mode
         if (currentControlMode == AbilityHandControlMode.POSITION && previousControlMode != AbilityHandControlMode.POSITION)
         {
            for (int i = 0; i < ACTUATOR_COUNT; i++)
               fingerTrajectories[i].reset(hand.getActuatorPosition(i), hand.getActuatorVelocity(i));
         }

         switch (currentControlMode)
         {
            case POSITION -> updatePositionControl();
            case VELOCITY -> updateVelocityControl();
            case GRIP -> updateGripControl();
         }

         previousControlMode = currentControlMode;
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
         float targetPosition = (float) goalPositions[actuatorIndex].getValue();
         float maxVelocity = Math.abs((float) goalVelocities[actuatorIndex].getValue());

         trajectory.setGoal(targetPosition, maxVelocity);
         float commandedPosition = trajectory.update(dt);
         hand.setCommandValue(actuatorIndex, commandedPosition);
      }
   }

   /** Updates hand to direct velocity control using goalVelocities. */
   private void updateVelocityControl()
   {
      hand.setCommandType(AbilityHandCommandType.VELOCITY);

      float[] commandVelocities = new float[ACTUATOR_COUNT];
      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         commandVelocities[i] = (float) goalVelocities[i].getValue();
      }
      hand.setCommandValues(commandVelocities);
   }

   /** Performs multi-stage grip control, moving fingers sequentially through grip.stages. */
   private void updateGripControl()
   {
      AbilityHandControlMode currentControlMode = controlMode.getValue();
      AbilityHandGrip currentGrip = grip.getValue();

      // Handle entering GRIP mode vs switching grips while already in GRIP
      if (previousControlMode != AbilityHandControlMode.GRIP)
      {
         gripStage = 0;
         previousGrip = currentGrip;

         // Re-sync trajectories to current hand state once when entering GRIP
         for (int actuatorIndex = 0; actuatorIndex < ACTUATOR_COUNT; actuatorIndex++)
         {
            fingerTrajectories[actuatorIndex].reset(hand.getActuatorPosition(actuatorIndex),
                                                    hand.getActuatorVelocity(actuatorIndex));
         }
      }
      else if (previousGrip != currentGrip)
      {
         // New grip while already in GRIP: restart stage sequence but keep trajectories continuous
         gripStage = 0;
         previousGrip = currentGrip;
      }

      // If we’re past the last stage, the grip is completed. No need to do anything
      if (currentGrip == null || gripStage >= currentGrip.stages.length)
         return;

      hand.setCommandType(AbilityHandCommandType.POSITION);

      // Normal grip stages: move only the fingers in this stage toward their stage goals,
      // but update all trajectories every tick.
      int[] actuatorsToMove = currentGrip.stages[gripStage];
      float[] stageGoalPositions = currentGrip.positions[gripStage];

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
         float maximumVelocity = Math.abs((float) goalVelocities[actuatorIndex].getValue());

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
      this.controlMode.set(controlMode);
   }

   /**
    * Sets the grip pattern for subsequent grip control.
    *
    * @param grip desired Grip pattern
    */
   public void setGrip(AbilityHandGrip grip)
   {
      this.grip.set(grip);
   }

   /**
    * Sets a goal position for a specific actuator.
    *
    * @param index        actuator index (0 to ACTUATOR_COUNT-1)
    * @param goalPosition desired position in degrees
    */
   public void setGoalPosition(int index, float goalPosition)
   {
      goalPositions[index].set(goalPosition);
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
      goalVelocities[index].set(goalVelocity);
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
      return (float) goalPositions[index].getValue();
   }

   /**
    * Retrieves the current goal velocity for a specific actuator.
    *
    * @param index
    * @return the current goal velocity for the specified actuator
    */
   public float getGoalVelocity(int index)
   {
      return (float) goalVelocities[index].getValue();
   }
}
