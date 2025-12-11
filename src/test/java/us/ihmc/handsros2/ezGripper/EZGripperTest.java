package us.ihmc.handsros2.ezGripper;

import org.junit.jupiter.api.Test;
import us.ihmc.handsros2.HandType;
import us.ihmc.robotics.robotSide.RobotSide;

import static org.junit.jupiter.api.Assertions.*;

public class EZGripperTest
{
   private static EZGripper createTestGripper()
   {
      return new EZGripper("LeftEZGripper", RobotSide.LEFT)
      {
         @Override
         public boolean updateCalibration()
         {
            return true;
         }
      };
   }

   @Test
   public void testPositionControlUpdate()
   {
      EZGripper testGripper = createTestGripper();
      testGripper.setGoalPosition(0.5f);
      testGripper.setMaxEffort(1.0f);
      testGripper.setTorqueOn(true);
      testGripper.setOperationMode(EZGripper.OperationMode.POSITION_CONTROL);

      testGripper.update();

      assertEquals(0.5f, testGripper.getGoalPosition(), 1e-6);
      assertEquals(1.0f, testGripper.getMaxEffort(), 1e-6);
      assertTrue(testGripper.getTorqueOnCommand());
      assertEquals(EZGripper.OperationMode.POSITION_CONTROL, testGripper.getOperationMode());
   }

   @Test
   public void testCalibrationMode()
   {
      EZGripper testGripper = createTestGripper();
      testGripper.setOperationMode(EZGripper.OperationMode.CALIBRATION);

      // First update: calibrates
      testGripper.update();

      // Second update: finishes calibration
      testGripper.update();

      assertTrue(testGripper.isCalibrated());
      assertEquals(EZGripper.OperationMode.POSITION_CONTROL, testGripper.getOperationMode());
   }

   @Test
   public void testErrorResetWithNoError()
   {
      EZGripper testGripper = createTestGripper();
      testGripper.setErrorCode((byte) 0);
      testGripper.setOperationMode(EZGripper.OperationMode.ERROR_RESET);

      // First update: starts reset
      testGripper.update();
      assertEquals(EZGripper.OperationMode.ERROR_RESET, testGripper.getOperationMode());

      // Second update: finishes reset
      testGripper.update();
      assertEquals(EZGripper.OperationMode.POSITION_CONTROL, testGripper.getOperationMode());
   }

   @Test
   public void testErrorResetWithError()
   {
      EZGripper testGripper = createTestGripper();
      testGripper.setErrorCode((byte) 1);
      testGripper.setOperationMode(EZGripper.OperationMode.ERROR_RESET);

      testGripper.update();

      assertEquals(EZGripper.OperationMode.ERROR_RESET, testGripper.getOperationMode());
      assertEquals(0.0f, testGripper.getMaxEffort(), 1e-6);
      assertFalse(testGripper.getTorqueOnCommand());
   }

   @Test
   public void testCooldownStart()
   {
      EZGripper testGripper = createTestGripper();
      testGripper.setTemperatureLimit((byte) 50);
      testGripper.setCurrentTemperature((byte) 60); // Above limit

      testGripper.update();

      assertEquals(EZGripper.OperationMode.COOLDOWN, testGripper.getOperationMode());
      assertEquals(0.0f, testGripper.getMaxEffort(), 1e-6);
      assertFalse(testGripper.getTorqueOnCommand());
   }

   @Test
   public void testCooldownComplete()
   {
      EZGripper testGripper = createTestGripper();
      testGripper.setTemperatureLimit((byte) 50);
      testGripper.setCurrentTemperature((byte) 60);
      testGripper.update(); // Start cooldown

      testGripper.setCurrentTemperature((byte) 44); // Below 90% of 50
      testGripper.update(); // Should end cooldown

      assertEquals(EZGripper.OperationMode.POSITION_CONTROL, testGripper.getOperationMode());
   }

   @Test
   public void testEZGripperType()
   {
      EZGripper testEZGripper = new EZGripper("EZGripper", RobotSide.LEFT);
      assertEquals(HandType.EZ_GRIPPER, testEZGripper.getType());
   }
}