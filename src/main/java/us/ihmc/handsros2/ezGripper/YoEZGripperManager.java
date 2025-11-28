package us.ihmc.handsros2.ezGripper;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * A YoVariable-ized version of the {@link EZGripperManager}.
 */
public class YoEZGripperManager extends EZGripperManager
{
   private final YoEnum<OperationMode> desiredOperationMode;
   private final YoEnum<OperationMode> operationMode;

   private final YoDouble goalPosition;
   private final YoDouble maxEffort;
   private final YoBoolean torqueOn;
   private final YoInteger temperatureLimit;

   private final YoBoolean isCalibrated;

   public YoEZGripperManager(YoRegistry registry, EZGripper gripper)
   {
      super(gripper);

      String prefix = gripper.getSide().name() + super.getClass().getSimpleName();
      desiredOperationMode = new YoEnum<>(prefix + "DesiredOperationMode", registry, OperationMode.class);
      desiredOperationMode.set(OperationMode.POSITION_CONTROL);
      operationMode = new YoEnum<>(prefix + "operationMode", registry, OperationMode.class);
      operationMode.set(OperationMode.POSITION_CONTROL);

      goalPosition = new YoDouble(prefix + "GoalPosition", registry);
      goalPosition.set(0.1);
      maxEffort = new YoDouble(prefix + "MaxEffort", registry);
      maxEffort.set(0.0);
      torqueOn = new YoBoolean(prefix + "TorqueOn", registry);
      torqueOn.set(false);
      temperatureLimit = new YoInteger(prefix + "TemperatureLimit", registry);
      temperatureLimit.set(75);

      isCalibrated = new YoBoolean(prefix + "IsCalibrated", registry);
      isCalibrated.set(false);
   }

   public void update()
   {
      super.setOperationMode(desiredOperationMode.getValue());
      super.setGoalPosition((float) goalPosition.getValue());
      super.setMaxEffort((float) maxEffort.getValue());
      super.setTorqueOn(torqueOn.getValue());
      super.setTemperatureLimit((byte) temperatureLimit.getValue());

      super.update();

      operationMode.set(super.getOperationMode());
      isCalibrated.set(super.isCalibrated());
   }

   @Override
   public void setGoalPosition(float goalPosition)
   {
      this.goalPosition.set(goalPosition);
   }

   @Override
   public void setMaxEffort(float maxEffort)
   {
      this.maxEffort.set(maxEffort);
   }

   @Override
   public void setTorqueOn(boolean torqueOn)
   {
      this.torqueOn.set(torqueOn);
   }

   @Override
   public void setOperationMode(OperationMode operationMode)
   {
      this.desiredOperationMode.set(operationMode.ordinal());
   }

   @Override
   public void setTemperatureLimit(byte temperatureLimit)
   {
      this.temperatureLimit.set(Byte.toUnsignedInt(temperatureLimit));
   }
}
