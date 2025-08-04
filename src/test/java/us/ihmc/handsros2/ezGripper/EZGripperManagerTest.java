package us.ihmc.handsros2.ezGripper;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

public class EZGripperManagerTest
{
   private static Stream<Arguments> getControllers()
   {
      TestEZGripper abilityHand = new TestEZGripper(RobotSide.LEFT);
      EZGripperManager manager = new EZGripperManager(abilityHand);

      YoEZGripper yoEZGripper = new TestYoEZGripper(null, RobotSide.RIGHT);
      YoEZGripperManager yoManager = new YoEZGripperManager(null, yoEZGripper);

      return Stream.of(Arguments.of(manager, abilityHand), Arguments.of(yoManager, yoEZGripper));
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testPositionControlUpdate(EZGripperManager gripperManager, EZGripperInterface testGripper)
   {
      gripperManager.setGoalPosition(0.5f);
      gripperManager.setMaxEffort(1.0f);
      gripperManager.setTorqueOn(true);
      gripperManager.setOperationMode(EZGripperManager.OperationMode.POSITION_CONTROL);

      gripperManager.update();

      assertEquals(0.5f, testGripper.getGoalPosition(), 1e-6);
      assertEquals(1.0f, testGripper.getMaxEffort(), 1e-6);
      assertTrue(testGripper.getTorqueOnCommand());
      assertEquals(EZGripperManager.OperationMode.POSITION_CONTROL, gripperManager.getOperationMode());
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testCalibrationMode(EZGripperManager gripperManager)
   {
      gripperManager.setOperationMode(EZGripperManager.OperationMode.CALIBRATION);

      // First update: calibrates
      gripperManager.update();

      // Second update: finishes calibration
      gripperManager.update();

      assertTrue(gripperManager.isCalibrated());
      assertEquals(EZGripperManager.OperationMode.POSITION_CONTROL, gripperManager.getOperationMode());
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testErrorResetWithNoError(EZGripperManager gripperManager, EZGripperInterface testGripper)
   {
      testGripper.setErrorCode((byte) 0);
      gripperManager.setOperationMode(EZGripperManager.OperationMode.ERROR_RESET);

      // First update: starts reset
      gripperManager.update();
      assertEquals(EZGripperManager.OperationMode.ERROR_RESET, gripperManager.getOperationMode());

      // Second update: finishes reset
      gripperManager.update();
      assertEquals(EZGripperManager.OperationMode.POSITION_CONTROL, gripperManager.getOperationMode());
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testErrorResetWithError(EZGripperManager gripperManager, EZGripperInterface testGripper)
   {
      testGripper.setErrorCode((byte) 1);
      gripperManager.setOperationMode(EZGripperManager.OperationMode.ERROR_RESET);

      gripperManager.update();

      assertEquals(EZGripperManager.OperationMode.ERROR_RESET, gripperManager.getOperationMode());
      assertEquals(0.0f, testGripper.getMaxEffort(), 1e-6);
      assertFalse(testGripper.getTorqueOnCommand());
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testCooldownStart(EZGripperManager gripperManager, EZGripperInterface testGripper)
   {
      gripperManager.setTemperatureLimit((byte) 50);
      testGripper.setCurrentTemperature((byte) 60); // Above limit

      gripperManager.update();

      assertEquals(EZGripperManager.OperationMode.COOLDOWN, gripperManager.getOperationMode());
      assertEquals(0.0f, testGripper.getMaxEffort(), 1e-6);
      assertFalse(testGripper.getTorqueOnCommand());
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testCooldownComplete(EZGripperManager gripperManager, EZGripperInterface testGripper)
   {
      gripperManager.setTemperatureLimit((byte) 50);
      testGripper.setCurrentTemperature((byte) 60);
      gripperManager.update(); // Start cooldown

      testGripper.setCurrentTemperature((byte) 44); // Below 90% of 50
      gripperManager.update(); // Should end cooldown

      assertEquals(EZGripperManager.OperationMode.POSITION_CONTROL, gripperManager.getOperationMode());
   }
}