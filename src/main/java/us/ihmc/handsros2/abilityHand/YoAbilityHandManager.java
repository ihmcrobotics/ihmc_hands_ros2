package us.ihmc.handsros2.abilityHand;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import static us.ihmc.handsros2.abilityHand.AbilityHandInterface.ACTUATOR_COUNT;

/**
 * A YoVariable-ized version of the {@link AbilityHandManager}.
 */
public class YoAbilityHandManager extends AbilityHandManager
{
   private final YoEnum<ControlMode> controlMode;
   private final YoEnum<Grip> grip;
   private final YoDouble[] goalPositions = new YoDouble[ACTUATOR_COUNT];
   private final YoDouble[] goalVelocities = new YoDouble[ACTUATOR_COUNT];

   public YoAbilityHandManager(YoRegistry registry, AbilityHandInterface hand)
   {
      super(hand);

      String prefix = hand.getHandSide().name() + super.getClass().getSimpleName();
      controlMode = new YoEnum<>(prefix + "ControlMode", registry, ControlMode.class);
      grip = new YoEnum<>(prefix + "Grip", registry, Grip.class);
      for (int i = 0; i < ACTUATOR_COUNT; ++i)
      {
         goalPositions[i] = new YoDouble(prefix + "GoalPosition" + i, registry);
         goalPositions[i].set(30.0);
         goalVelocities[i] = new YoDouble(prefix + "GoalVelocity" + i, registry);
      }
      goalPositions[ACTUATOR_COUNT-1].set(-30.0);
   }

   @Override
   public void update()
   {
      super.setControlMode(controlMode.getValue());
      super.setGrip(grip.getValue());
      for (int i = 0; i < ACTUATOR_COUNT; ++i)
      {
         super.setGoalPosition(i, (float) goalPositions[i].getValue());
         super.setGoalVelocity(i, (float) goalVelocities[i].getValue());
      }

      super.update();
   }

   @Override
   public void setControlMode(ControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }

   @Override
   public void setGrip(Grip grip)
   {
      this.grip.set(grip);
   }

   @Override
   public void setGoalPosition(int index, float goalPosition)
   {
      goalPositions[index].set(goalPosition);
   }

   @Override
   public void setGoalVelocity(int index, float goalVelocity)
   {
      goalVelocities[index].set(goalVelocity);
   }
}
