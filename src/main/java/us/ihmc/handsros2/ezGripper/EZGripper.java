package us.ihmc.handsros2.ezGripper;

import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;

import static us.ihmc.handsros2.ezGripper.EZGripperModel.EZGripperJointName.*;
import us.ihmc.handsros2.ezGripper.EZGripperModel.EZGripperJointName;

import java.util.List;

/**
 * Manages and controls a SAKE Robotics EZ Gripper.
 * Supports calibration, error reset, and cooldown.
 */
public class EZGripper implements HandInterface
{
   public static final int RAW_RANGE_OF_MOTION = 2500;

   public enum OperationMode
   {
      POSITION_CONTROL, CALIBRATION, ERROR_RESET, COOLDOWN;

      public static final OperationMode[] values = values();

      public static OperationMode fromByte(byte ordinal)
      {
         return values[ordinal];
      }

      public byte toByte()
      {
         return (byte) this.ordinal();
      }
   }

   public static final int DISABLE_AUTO_COOLDOWN = 255;

   private final String identifier;
   private final RobotSide robotSide;

   private final YoDouble goalPosition;
   private final YoDouble maxEffort;
   private final YoBoolean torqueOn;

   private final YoDouble currentPosition;
   private final YoDouble currentEffort;
   private final YoInteger temperature;
   private final YoInteger realtimeTick;
   private final YoInteger errorCode;

   private final YoBoolean isCalibrated;
   private final YoDouble temperatureLimit;
   private final YoRegistry registry;

   private final YoEnum<OperationMode> desiredOperationMode;
   private final YoEnum<OperationMode> operationMode;
   private final StateMachine<OperationMode, State> stateMachine;

   public EZGripper(String identifier, RobotSide robotSide)
   {
      this(new YoRegistry("EZGripper_" + identifier + "_" + robotSide.name()), identifier, robotSide);
   }

   public EZGripper(YoRegistry registry, String identifier, RobotSide robotSide)
   {
      this.identifier = identifier;
      this.robotSide = robotSide;
      this.registry = registry;

      String prefix = robotSide.name() + "EZGripper";

      // Low level control
      goalPosition = new YoDouble(prefix + "GoalPosition", registry);
      maxEffort = new YoDouble(prefix + "MaxEffort", registry);
      torqueOn = new YoBoolean(prefix + "TorqueOn", registry);

      // State
      currentPosition = new YoDouble(prefix + "CurrentPosition", registry);
      currentEffort = new YoDouble(prefix + "CurrentEffort", registry);
      temperature = new YoInteger(prefix + "Temperature", registry);
      realtimeTick = new YoInteger(prefix + "RealtimeTick", registry);
      errorCode = new YoInteger(prefix + "ErrorCode", registry);

      // High level state
      isCalibrated = new YoBoolean(prefix + "IsCalibrated", registry);
      isCalibrated.set(false);
      temperatureLimit = new YoDouble(prefix + "TemperatureLimit", registry);
      temperatureLimit.set(DISABLE_AUTO_COOLDOWN);

      desiredOperationMode = new YoEnum<>(prefix + "DesiredOperationMode", registry, OperationMode.class);
      desiredOperationMode.set(OperationMode.POSITION_CONTROL);
      operationMode = new YoEnum<>(prefix + "OperationMode", registry, OperationMode.class);
      operationMode.set(OperationMode.POSITION_CONTROL);

      StateMachineFactory<OperationMode, State> factory = new StateMachineFactory<>(OperationMode.class);
      factory.buildClock(System::nanoTime);

      // Add position control state
      factory.addState(OperationMode.POSITION_CONTROL, new PositionControlState());

      // Position control goes to calibration or error reset if demanded
      factory.addTransition(OperationMode.POSITION_CONTROL, OperationMode.CALIBRATION,
                            nanoTime -> desiredOperationMode.getValue() == OperationMode.CALIBRATION);
      factory.addTransition(OperationMode.POSITION_CONTROL, OperationMode.ERROR_RESET,
                            nanoTime -> desiredOperationMode.getValue() == OperationMode.ERROR_RESET);
      factory.addTransition(OperationMode.POSITION_CONTROL, OperationMode.COOLDOWN, nanoTime -> desiredOperationMode.getValue() == OperationMode.COOLDOWN);

      // Add calibration state. Goes to position control once done.
      factory.addStateAndDoneTransition(OperationMode.CALIBRATION, new CalibrationState(), OperationMode.POSITION_CONTROL);

      // Add error reset state. Goes to position control once done.
      factory.addStateAndDoneTransition(OperationMode.ERROR_RESET, new ErrorResetState(), OperationMode.POSITION_CONTROL);

      // Add cooldown state. Goes to position control once done.
      factory.addStateAndDoneTransition(OperationMode.COOLDOWN, new CooldownState(), OperationMode.POSITION_CONTROL);

      // All other states go to cooldown if gripper overheats.
      factory.addTransition(List.of(OperationMode.POSITION_CONTROL, OperationMode.CALIBRATION, OperationMode.ERROR_RESET),
                            OperationMode.COOLDOWN,
                            nanoTime -> temperatureLimit.getValue() != DISABLE_AUTO_COOLDOWN && temperature.getValue() > temperatureLimit.getValue());

      // Build the state machine
      stateMachine = factory.build(OperationMode.POSITION_CONTROL);
   }

   private class PositionControlState implements State
   {
      @Override
      public void onEntry()
      {
      }

      @Override
      public void doAction(double timeInState)
      {
         // Don't do anything in case of error
         if ((byte) errorCode.getValue() != EZGripperError.NONE.errorCode)
         {
            maxEffort.set(0.0);
            torqueOn.set(false);
         }
      }

      @Override
      public void onExit(double timeInState)
      {
      }
   }

   private class CalibrationState implements State
   {
      private boolean done = false;

      @Override
      public void onEntry()
      {
      }

      @Override
      public void doAction(double timeInState)
      {
         done = updateCalibration();
      }

      @Override
      public void onExit(double timeInState)
      {
         isCalibrated.set(done);
         torqueOn.set(false);
         desiredOperationMode.set(OperationMode.POSITION_CONTROL);
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return done;
      }
   }

   private class ErrorResetState implements State
   {
      private byte step;

      @Override
      public void onEntry()
      {
         step = 0;
      }

      @Override
      public void doAction(double timeInState)
      {
         switch (step)
         {
            case 0 -> // First step: power off the Dynamixel
            {
               torqueOn.set(false);
               maxEffort.set(0.0);
            }
            case 1 -> // Second step: power the Dynamixel back on. This clears the error
            {
               torqueOn.set(true);
               maxEffort.set(0.1);
            }
         }

         ++step;
      }

      @Override
      public void onExit(double timeInState)
      {
         torqueOn.set(false);
         desiredOperationMode.set(OperationMode.POSITION_CONTROL);
      }

      @Override
      public boolean isDone(double timeInState)
      {
         // Done if there's no error, or we tried the two steps and failed to clear the error
         return (byte) errorCode.getValue() == EZGripperError.NONE.errorCode || step >= 2;
      }
   }

   private class CooldownState implements State
   {
      private byte goalTemp = (byte) 255;

      @Override
      public void onEntry()
      {
         byte limit = (byte) temperatureLimit.getValue();
         goalTemp = (byte) (limit - limit / 10);
      }

      @Override
      public void doAction(double timeInState)
      {
         // Power off the Dynamixel so it cools down
         torqueOn.set(false);
         maxEffort.set(0.0);
      }

      @Override
      public void onExit(double timeInState)
      {
         torqueOn.set(false);
         desiredOperationMode.set(OperationMode.POSITION_CONTROL);
      }

      @Override
      public boolean isDone(double timeInState)
      {
         // Done if the cooldown is disabled or Dynamixel reaches the goal temperature
         return temperatureLimit.getValue() == DISABLE_AUTO_COOLDOWN || temperature.getValue() <= goalTemp;
      }
   }

   /**
    * Call periodically to update internal state machine and enforce temperature-based cooldown.
    */
   @Override
   public void update()
   {
      stateMachine.doActionAndTransition();
      operationMode.set(stateMachine.getCurrentStateKey());
   }

   public String getIdentifier()
   {
      return identifier;
   }

   public RobotSide getSide()
   {
      return robotSide;
   }

   public HandType getType()
   {
      return HandType.EZ_GRIPPER;
   }

   // COMMAND METHODS //

   /**
    * Start the calibration process.
    * <p>
    * Generally the grippers can be calibrated by closing until the fingers collide,
    * then recording the position as the fully closed position. The open position is
    * 2500 raw position units away from the closed position.
    *
    * @return {@code true} if calibration is complete.
    *       If {@code false} is returned, this method will be called again before any other commands are given to the gripper.
    */
   public boolean updateCalibration()
   {
      return false;
   }

   /**
    * Set the goal position. 0.0 = closed, 1.0 = open.
    *
    * @param goalPosition The goal position.
    */
   public void setGoalPosition(float goalPosition)
   {
      this.goalPosition.set(goalPosition);
   }

   public float getGoalPosition()
   {
      return (float) goalPosition.getValue();
   }

   /**
    * <p>Set the maximum effort to be used to achieve the goal position. 0.0 = no effort, 1.0 = max effort.</p>
    * <p>0.0 = no effort (fingers will not move), 1.0 = maximum effort (can quickly overheat actuator).
    * 0.3 is a reasonable normal value, and it is recommended not to exceed 0.8.</p>
    *
    * @param maxEffort The maximum effort to use to read the goal position.
    */
   public void setMaxEffort(float maxEffort)
   {
      this.maxEffort.set(maxEffort);
   }

   public float getMaxEffort()
   {
      return (float) maxEffort.getValue();
   }

   /**
    * Set whether to turn torque on.
    * Keeping the torque off when not needed can help keep the gripper's temperature down.
    *
    * @param on Whether to turn the torque on.
    */
   public void setTorqueOn(boolean on)
   {
      torqueOn.set(on);
   }

   public boolean getTorqueOnCommand()
   {
      return torqueOn.getValue();
   }

   /**
    * Set the desired operation mode.
    *
    * @param operationMode The desired operation mode.
    */
   public void setOperationMode(OperationMode operationMode)
   {
      this.desiredOperationMode.set(operationMode);
   }

   /**
    * Get the current operation mode. Depending on the state of the high level state,
    * the current operation mode may not be the desired operation mode.
    *
    * @return The current operation mode.
    */
   public OperationMode getOperationMode()
   {
      OperationMode currentMode = stateMachine.getCurrentStateKey();
      return currentMode == null ? stateMachine.getInitialStateKey() : currentMode;
   }

   /**
    * Whether the gripper has been calibrated.
    *
    * @return {@code true} if the gripper has been calibrated. {@code false} otherwise.
    */
   public boolean isCalibrated()
   {
      return isCalibrated.getBooleanValue();
   }

   /**
    * <p>Set the temperature limit.</p>
    * <p>The hand will automatically go into cooldown mode if the gripper exceed this temperature.
    * Cooldown will continue until the gripper's temperature drops by 10% of the temperature limit.</p>
    * <p>To override or disable automatic cooldown, set the temperature limit to {@link #DISABLE_AUTO_COOLDOWN}</p>
    *
    * @param temperatureLimit The temperature limit, in Celsius.
    */
   public void setTemperatureLimit(byte temperatureLimit)
   {
      this.temperatureLimit.set(temperatureLimit);
   }

   public float getCurrentPosition()
   {
      return (float) currentPosition.getValue();
   }

   public void setCurrentPosition(float currentPosition)
   {
      this.currentPosition.set(currentPosition);
   }

   public float getCurrentEffort()
   {
      return (float) currentEffort.getValue();
   }

   public void setCurrentEffort(float currentEffort)
   {
      this.currentEffort.set(currentEffort);
   }

   public byte getTemperature()
   {
      return (byte) temperature.getValue();
   }

   public void setCurrentTemperature(byte currentTemperature)
   {
      temperature.set(Byte.toUnsignedInt(currentTemperature));
   }

   public int getRealtimeTick()
   {
      return realtimeTick.getValue();
   }

   public void setRealtimeTick(int realtimeTick)
   {
      this.realtimeTick.set(realtimeTick);
   }

   public byte getErrorCode()
   {
      return (byte) errorCode.getValue();
   }

   public void setErrorCode(byte errorCode)
   {
      this.errorCode.set(Byte.toUnsignedInt(errorCode));
   }

   public void readJointAngles(double[] jointAngles)
   {
      double angle = JOINT_RANGE * (1.0f - getCurrentPosition());
      jointAngles[KNUCKLE_PALM_L1_1.getIndex(getSide())] = angle;
      jointAngles[KNUCKLE_PALM_L1_2.getIndex(getSide())] = angle;
      // We can't know the angle of the fingertips
      jointAngles[KNUCKLE_L1_L2_1.getIndex(getSide())] = 0.0;
      jointAngles[KNUCKLE_L1_L2_2.getIndex(getSide())] = 0.0;
   }

   public int getJointCount()
   {
      return EZGripperJointName.values.length;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }
}
