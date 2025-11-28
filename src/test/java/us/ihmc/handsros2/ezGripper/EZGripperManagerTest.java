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
      EZGripper leftEZGripper = new EZGripper("LeftEZGripper", RobotSide.LEFT)
      {
         @Override
         public boolean updateCalibration()
         {
            return true;
         }
      };
      EZGripperManager manager = new EZGripperManager(leftEZGripper);

      EZGripper rightEZGripper = new EZGripper(null, "RightEZGripper", RobotSide.RIGHT)
      {
         @Override
         public boolean updateCalibration()
         {
            return true;
         }
      };
      YoEZGripperManager yoManager = new YoEZGripperManager(null, rightEZGripper);

      return Stream.of(Arguments.of(manager, leftEZGripper), Arguments.of(yoManager, rightEZGripper));
   }

   @ParameterizedTest
   @MethodSource("getControllers")
   public void testPositionControlUpdate(EZGripperManager gripperManager, EZGripper testGripper)
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
   public void testErrorResetWithNoError(EZGripperManager gripperManager, EZGripper testGripper)
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
   public void testErrorResetWithError(EZGripperManager gripperManager, EZGripper testGripper)
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
   public void testCooldownStart(EZGripperManager gripperManager, EZGripper testGripper)
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
   public void testCooldownComplete(EZGripperManager gripperManager, EZGripper testGripper)
   {
      gripperManager.setTemperatureLimit((byte) 50);
      testGripper.setCurrentTemperature((byte) 60);
      gripperManager.update(); // Start cooldown

      testGripper.setCurrentTemperature((byte) 44); // Below 90% of 50
      gripperManager.update(); // Should end cooldown

      assertEquals(EZGripperManager.OperationMode.POSITION_CONTROL, gripperManager.getOperationMode());
   }
}