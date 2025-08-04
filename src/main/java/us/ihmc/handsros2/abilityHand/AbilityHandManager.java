package us.ihmc.handsros2.abilityHand;

import static us.ihmc.handsros2.abilityHand.AbilityHandInterface.ACTUATOR_COUNT;

/**
 * Manages higher-level control of an Ability Hand, including position, velocity,
 * velocity-to-position, and multi-stage grip operations.
 */
public class AbilityHandManager
{
   /**
    * Control modes for the Ability Hand Manager.
    */
   public enum ControlMode
   {
      POSITION, VELOCITY, VEL_TO_POS, GRIP;

      /** Array of all control modes. */
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
    * Use Grip Editor from the Psyonic app to get angles for other grips
    */
   public enum Grip
   {
      POWER    (new int[][] {{0, 1, 2, 3}, {5}, {4}}, new float[][] {{97.5f, 97.5f, 97.5f, 97.5f}, {-75}, {75}}),
      KEY      (new int[][] {{0, 1, 2, 3}, {5}, {4}}, new float[][] {{90, 90, 90, 90}, {-20}, {75}}),
      TRIPOD_O (new int[][] {{0, 1, 2, 3}, {5}, {4}}, new float[][] {{60, 63, 20, 20}, {-76}, {54}}),
      TRIPOD_C (new int[][] {{0, 1, 2, 3}, {5}, {4}}, new float[][] {{60, 63, 97.5f, 97.5f}, {-76}, {54}}),
      RELAX    (new int[][] {{4}, {0, 1, 2, 3, 5}},   new float[][] {{30}, {30, 30, 30, 30, -30}}),
      RUDE     (new int[][] {{0, 1, 2, 3, 4}, {5}},   new float[][] {{100, 10, 100, 100, 20}, {-30}}),
      HOOK     (new int[][] {{0, 1, 2, 3, 4}, {5}},   new float[][] {{70, 70, 70, 70, 10}, {-10}}),
      PINCH_O  (new int[][] {{0, 1, 2, 3}, {5}, {4}}, new  float[][] {{61, 20, 20, 20}, {-67}, {53}}),
      PINCH_C  (new int[][] {{0, 1, 2, 3}, {5}, {4}}, new  float[][] {{61, 97.5f, 97.5f, 97.5f}, {-67}, {53}});

      /** Array of all grip patterns. */
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
   }

   private static final float TOLERANCE = 2.0f;
   private static final float THUMB_CLEAR_POSITION = 30.0f;

   private final AbilityHandInterface hand;

   private ControlMode controlMode = ControlMode.POSITION;
   private ControlMode previousControlMode = controlMode;

   private Grip grip = null;
   private Grip previousGrip = null;
   private int gripStage = Integer.MAX_VALUE;

   private final float[] goalPositions;
   private final float[] goalVelocities;

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
   }

   /**
    * Updates the hand commands based on the current control mode. Should be called periodically.
    */
   public void update()
   {
      switch (controlMode)
      {
         case POSITION -> updatePositionControl();
         case VELOCITY -> updateVelocityControl();
         case VEL_TO_POS -> updateVelToPosControl();
         case GRIP -> updateGripControl();
      }

      previousControlMode = controlMode;
   }

   /** Updates hand to direct position control using goalPositions. */
   private void updatePositionControl()
   {
      hand.setCommandType(AbilityHandCommandType.POSITION);
      hand.setCommandValues(goalPositions);
   }

   /** Updates hand to direct velocity control using goalVelocities. *** */
   private void updateVelocityControl()
   {
      hand.setCommandType(AbilityHandCommandType.VELOCITY);
      hand.setCommandValues(goalVelocities);
   }

   /**
    * Calculates the velocity needed to move an actuator toward a goal position.
    *
    * @param actuatorIndex index of the actuator
    * @param goalPosition  target position
    * @param goalVelocity  desired speed magnitude
    * @return computed velocity (positive or negative) or zero if within tolerance
    */
   private float calculateVelocityToPosition(int actuatorIndex, float goalPosition, float goalVelocity)
   {
      // Get the current position
      float currentPosition = hand.getActuatorPosition(actuatorIndex);

      // If we've reached the goal position, velocity should be 0 (stop moving)
      if (Math.abs(currentPosition - goalPosition) < TOLERANCE)
         return 0.0f;

      // Otherwise velocity should be in the correct direction
      float speed = Math.abs(goalVelocity);
      return currentPosition < goalPosition ? speed : -speed;
   }

   /** Updates hand to velocity control that moves toward goalPositions. */
   private void updateVelToPosControl()
   {
      hand.setCommandType(AbilityHandCommandType.VELOCITY);

      for (int i = 0; i < ACTUATOR_COUNT; i++)
      {
         float velocity = calculateVelocityToPosition(i, goalPositions[i], goalVelocities[i]);
         hand.setCommandValue(i, velocity);
      }
   }

   /** Performs multi-stage grip control, moving fingers sequentially through grip.stages. */
   private void updateGripControl()
   {
      // If goal grip changed, reset grip stage
      if (previousGrip != grip || previousControlMode != ControlMode.GRIP)
      {
         gripStage = -1;
         previousGrip = grip;
      }

      // If we’re past the last stage, the grip is completed. No need to do anything
      if (gripStage >= grip.stages.length)
         return;

      // Using velocity to position control
      hand.setCommandType(AbilityHandCommandType.VELOCITY);

      // Move the thumb out of the way before any grip (stage = -1)
      if (gripStage == -1)
      {
         // Calculate the velocity required to read the clear position
         float thumbVelocity = calculateVelocityToPosition(4, THUMB_CLEAR_POSITION, goalVelocities[4]);

         // If velocity is 0 or positive, thumb is already clear. Only move thumb if velocity is negative.
         if (thumbVelocity < 0.0f)
         {
            // Tell thumb to move out of the way
            hand.setCommandValue(4, thumbVelocity);

            // Rest of the fingers shouldn't move
            for (int i = 0; i < ACTUATOR_COUNT; ++i)
               if (i != 4)
                  hand.setCommandValue(i, 0.0f);

            return;
         }
         for (int i = 0; i < ACTUATOR_COUNT; ++i)
            hand.setCommandValue(i, 0.0f);

         // Thumb is clear. Start normal grip stages.
         gripStage = 0;
      }

      // Get the actuators that need to move during this stage and their goal positions
      int[] actuatorsToMove = grip.stages[gripStage];
      float[] goalPositions = grip.positions[gripStage];

      boolean stageComplete = true;
      for (int i = 0; i < actuatorsToMove.length; i++)
      {
         int actuatorIndex = actuatorsToMove[i];
         float goalPosition = goalPositions[i];
         float goalVelocity = goalVelocities[actuatorIndex];

         float velocity = calculateVelocityToPosition(actuatorIndex, goalPosition, goalVelocity);

         if (velocity != 0.0f)
            stageComplete = false;

         hand.setCommandValue(actuatorIndex, velocity);
      }

      if (stageComplete)
      {
         gripStage++;
      }
   }

   /**
    * Returns the underlying hand interface.
    *
    * @return the AbilityHandInterface
    */
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
}
