package us.ihmc.handsros2.abilityHand;

import us.ihmc.handsros2.HandManager;
import us.ihmc.handsros2.TrapezoidalTrajectory1D;
import us.ihmc.handsros2.VelocityControlTools;

import static us.ihmc.handsros2.abilityHand.AbilityHandInterface.ACTUATOR_COUNT;

/**
 * Manages higher-level control of an Ability Hand, including position, velocity,
 * velocity-to-position, and multi-stage grip operations.
 */
public class AbilityHandManager implements HandManager<AbilityHandInterface>
{
   /**
    * Control modes for the Ability Hand Manager.
    */
   public enum ControlMode
   {
      POSITION, VELOCITY, VEL_TO_POS, GRIP;

      public static final ControlMode[] values = values();

      /**
       * Retrieves a ControlMode by its byte ordinal.
       *
       * @param ordinal the byte ordinal
       * @return the corresponding ControlMode
       */
      public static ControlMode fromByte(byte ordinal)
      {
         return values[ordinal];
      }

      /**
       * Converts this ControlMode to its byte representation.
       *
       * @return the ordinal as a byte
       */
      public byte toByte()
      {
         return (byte) this.ordinal();
      }
   }

   /**
    * Predefined multi-stage grip patterns with associated finger indices and target positions.
    * Use Grip Editor in the PSYONIC app to get angles for other grips.
    */
   public enum Grip
   {
      OPEN(new int[][] {{0, 1, 2, 3, 4, 5}}, new float[][] {{1, 1, 1, 1, 1, -93}}),
      CLOSE(new int[][] {{0, 1, 2, 3, 4, 5}}, new float[][] {{65, 65, 65, 65, 42, -93}}),
      PINCH(new int[][] {{0, 1, 2, 3, 4, 5}}, new float[][] {{61, 64, 1, 1, 53, -76}}),
      FLAT(new int[][] {{0, 1, 2, 3, 4}, {5}}, new float[][] {{1, 1, 1, 1, 1}, {-3}}),
      HOOK(new int[][] {{0, 1, 2, 3, 4, 5}}, new float[][] {{70, 70, 70, 70, 10, -10}}),
      RELAX(new int[][] {{4}, {0, 1, 2, 3, 5}}, new float[][] {{30}, {30, 30, 30, 30, -30}}),
      DOOR_LEVER_OPEN(new int[][] {{0, 1, 2, 3, 4, 5}}, new float[][] {{38, 47, 53, 60, 60, -4}}),
      DOOR_LEVER_CLOSE(new int[][] {{0, 1, 2, 3, 4, 5}}, new float[][] {{65, 71, 78, 81, 60, -4}}),
      DOOR_LEVER_CRUSH(new int[][] {{0, 1, 2, 3, 4, 5}}, new float[][] {{105, 105, 105, 105, 33, -4}}),
      KEY_OPEN(new int[][] {{0, 1, 2, 3, 4, 5}}, new float[][] {{90, 90, 90, 90, 0, -20}}),
      KEY_CLOSE(new int[][] {{0, 1, 2, 3, 4, 5}}, new float[][] {{90, 90, 90, 90, 80, -20}}),
      ;

      public static final Grip[] values = values();

      final int[][] stages;
      final float[][] positions;

      /**
       * Constructs a Grip pattern.
       *
       * @param stages    finger index stages for sequential movement
       * @param positions target positions for each stage
       */
      Grip(int[][] stages, float[][] positions)
      {
         this.stages = stages;
         this.positions = positions;
      }

      /**
       * Retrieves a Grip by its byte ordinal.
       *
       * @param ordinal the byte ordinal
       * @return the corresponding Grip
       */
      public static Grip fromByte(byte ordinal)
      {
         return values[ordinal];
      }

      /**
       * Converts this Grip to its byte representation.
       *
       * @return the ordinal as a byte
       */
      public byte toByte()
      {
         return (byte) this.ordinal();
      }

      /** Number of stages in this grip sequence. */
      public int getNumberOfStages()
      {
         return stages.length;
      }

      /** Number of fingers active in this grip stage. */
      public int getFingersInStage(int stage)
      {
         return stages[stage].length;
      }

      /** Which finger is specified at this index in the grip stage. */
      public int getStageFingerIndex(int stage, int finger)
      {
         return stages[stage][finger];
      }

      /** The position of the finger which is specified at this index in the grip stage. */
      public float getStageFingerPosition(int stage, int finger)
      {
         return positions[stage][finger];
      }
   }

   private static final float TOLERANCE = 1.0f;
   /** Trajectory configuration: tune acceleration per joint as needed (deg/s^2) */
   private static final float DEFAULT_MAXIMUM_ACCELERATION = 200.0f;

   private final AbilityHandInterface hand;

   private ControlMode controlMode = null;
   private ControlMode previousControlMode = controlMode;

   private Grip grip = null;
   private Grip previousGrip = null;
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
    * @param hand the AbilityHandInterface implementation for low-level control
    */
   public AbilityHandManager(AbilityHandInterface hand)
   {
      this.hand = hand;

      goalPositions = new float[] {30.0f, 30.0f, 30.0f, 30.0f, 30.0f, -30.0f};
      goalVelocities = new float[] {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

      fingerTrajectories = new TrapezoidalTrajectory1D[ACTUATOR_COUNT];
   }

   public void initialize()
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

   /** {@inheritDoc} */
   @Override
   public void update()
   {
      long nowNanos = System.nanoTime();

      if (previousTimeNanos > 0L)
      {
         long deltaNanos = nowNanos - previousTimeNanos;
         float dt = Math.min(0.1f, Math.max(deltaNanos * 1.0e-9f, 0.001f));

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
         if (controlMode == ControlMode.POSITION && previousControlMode != ControlMode.POSITION)
            for (int i = 0; i < ACTUATOR_COUNT; i++)
               fingerTrajectories[i].reset(hand.getActuatorPosition(i), hand.getActuatorVelocity(i));

         switch (controlMode)
         {
            case POSITION -> updatePositionControl();
            case VELOCITY -> updateVelocityControl();
            case VEL_TO_POS -> updateVelToPosControl();
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

   /** Velocity control toward goalPositions using trapezoidal-like velocity strategy. */
   private void updateVelToPosControl()
   {
      hand.setCommandType(AbilityHandCommandType.VELOCITY);

      for (int actuatorIndex = 0; actuatorIndex < ACTUATOR_COUNT; actuatorIndex++)
      {
         float currentPosition = hand.getActuatorPosition(actuatorIndex);
         float currentVelocity = hand.getActuatorVelocity(actuatorIndex);
         float goalPosition = goalPositions[actuatorIndex];
         float maximumVelocity = Math.abs(goalVelocities[actuatorIndex]);
         float maximumAcceleration = DEFAULT_MAXIMUM_ACCELERATION;
         float deadzone = TOLERANCE;

         float commandedVelocity = VelocityControlTools.computeVelocityCommand(currentPosition,
                                                                               currentVelocity,
                                                                               goalPosition,
                                                                               maximumVelocity,
                                                                               maximumAcceleration,
                                                                               dt,
                                                                               deadzone);
         hand.setCommandValue(actuatorIndex, commandedVelocity);
      }
   }

   /** Performs multi-stage grip control, moving fingers sequentially through grip.stages. */
   private void updateGripControl()
   {
      // Handle entering GRIP mode vs switching grips while already in GRIP
      if (previousControlMode != ControlMode.GRIP)
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
         // Optionally snap active fingers to exact stage goal to avoid tiny residuals
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

   /** {@inheritDoc} */
   @Override
   public AbilityHandInterface getHand()
   {
      return hand;
   }

   /**
    * Sets the control mode for subsequent updates.
    *
    * @param controlMode desired control mode
    */
   public void setControlMode(ControlMode controlMode)
   {
      this.controlMode = controlMode;
   }

   /**
    * Sets the grip pattern for subsequent grip control.
    *
    * @param grip desired Grip pattern
    */
   public void setGrip(Grip grip)
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
