package us.ihmc.handsros2.abilityHand;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.Grip;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

public class AbilityHandManagerTest
{
   private static Stream<Arguments> getControllers()
   {
      TestAbilityHand abilityHand = new TestAbilityHand("24ABH000", RobotSide.LEFT);
      AbilityHandManager manager = new AbilityHandManager(abilityHand);

      YoAbilityHand yoAbilityHand = new YoAbilityHand(null, "24ABH001", RobotSide.RIGHT);
      YoAbilityHandManager yoManager = new YoAbilityHandManager(null, yoAbilityHand);

      return Stream.of(Arguments.of(manager, abilityHand), Arguments.of(yoManager, yoAbilityHand));
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testPositionControl(AbilityHandManager manager, AbilityHandInterface hand)
   {
      float[] positions = {10f, 20f, 30f, 40f, 50f, -10f};
      manager.setControlMode(ControlMode.POSITION);
      manager.setGoalPositions(positions);

      manager.update();

      assertEquals(AbilityHandCommandType.POSITION, hand.getCommandType());
      for (int i = 0; i < positions.length; i++)
      {
         assertEquals(positions[i], hand.getCommandValue(i), 1e-6f);
      }
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testVelocityControl(AbilityHandManager manager, AbilityHandInterface hand)
   {
      float[] velocities = {1f, 2f, 3f, 4f, 5f, -5f};
      manager.setControlMode(ControlMode.VELOCITY);
      manager.setGoalVelocities(velocities);

      manager.update();

      assertEquals(AbilityHandCommandType.VELOCITY, hand.getCommandType());
      for (int i = 0; i < velocities.length; i++)
      {
         assertEquals(velocities[i], hand.getCommandValue(i), 1e-6f);
      }
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testVelToPosControl(AbilityHandManager manager, AbilityHandInterface hand)
   {
      // current < goal => positive, current > goal => negative
      float[] current = {5f, 50f, 0f, 0f, 0f, 0f};
      float[] goals = {10f, 20f, 0f, 0f, 0f, 0f};
      float[] speeds = {2f, 3f, 0f, 0f, 0f, 0f};

      hand.setActuatorPositions(current);
      manager.setControlMode(ControlMode.VEL_TO_POS);
      manager.setGoalPositions(goals);
      manager.setGoalVelocities(speeds);

      manager.update();

      assertEquals(AbilityHandCommandType.VELOCITY, hand.getCommandType());
      assertTrue(hand.getCommandValue(0) > 0, "Index 0 should move positively");
      assertTrue(hand.getCommandValue(1) < 0, "Index 1 should move negatively");
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testGripInitialThumbStage(AbilityHandManager manager, AbilityHandInterface hand)
   {
      // Thumb (index 4) must clear first: simulate it at the clear position already
      float[] current = {30f, 30f, 30f, 30f, 30f, 30f};
      hand.setActuatorPositions(current);

      manager.setGoalVelocities(new float[] {5f, 5f, 5f, 5f, 5f, 5f});
      manager.setControlMode(ControlMode.GRIP);
      manager.setGrip(Grip.GRIP);

      manager.update();

      assertEquals(AbilityHandCommandType.VELOCITY, hand.getCommandType());

      // Thumb index 4 should be zero (already clear) and stage 0 should run:
      for (int i = 0; i < 4; i++)
         assertTrue(hand.getCommandValue(i) > 0, "Finger " + i + " should start closing");
      assertEquals(0f, hand.getCommandValue(4), 1e-6f, "Thumb should not move on clear");
      assertEquals(0f, hand.getCommandValue(5), 1e-6f, "Pinky shouldn't move in stage 0");
   }
}
