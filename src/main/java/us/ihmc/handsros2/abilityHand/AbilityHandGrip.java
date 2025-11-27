package us.ihmc.handsros2.abilityHand;

/**
 * Predefined multi-stage grip patterns with associated finger indices and target positions.
 * Use Grip Editor in the PSYONIC app to get angles for other grips.
 */
public enum AbilityHandGrip
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

   public static final AbilityHandGrip[] values = values();

   final int[][] stages;
   final float[][] positions;

   /**
    * Constructs a Grip pattern.
    *
    * @param stages    finger index stages for sequential movement
    * @param positions target positions for each stage
    */
   AbilityHandGrip(int[][] stages, float[][] positions)
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
   public static AbilityHandGrip fromByte(byte ordinal)
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
