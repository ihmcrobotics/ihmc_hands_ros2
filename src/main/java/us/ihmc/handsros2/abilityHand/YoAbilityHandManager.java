package us.ihmc.handsros2.abilityHand;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import static us.ihmc.handsros2.abilityHand.AbilityHand.ACTUATOR_COUNT;

/**
 * A YoVariable-ized version of the {@link AbilityHandManager}.
 */
public class YoAbilityHandManager extends AbilityHandManager
{
   private final YoEnum<AbilityHandControlMode> controlMode;
   private final YoEnum<AbilityHandGrip> grip;
   private final YoDouble[] goalPositions = new YoDouble[ACTUATOR_COUNT];
   private final YoDouble[] goalVelocities = new YoDouble[ACTUATOR_COUNT];

   public YoAbilityHandManager(YoRegistry registry, AbilityHand hand)
   {
      super(hand);

      String prefix = hand.getSide().name() + super.getClass().getSimpleName();
      controlMode = new YoEnum<>(prefix + "ControlMode", registry, AbilityHandControlMode.class);
      grip = new YoEnum<>(prefix + "Grip", registry, AbilityHandGrip.class);
      for (int i = 0; i < ACTUATOR_COUNT; ++i)
      {
         goalPositions[i] = new YoDouble(prefix + "GoalPosition" + i, registry);
         goalPositions[i].set(getGoalPosition(i));
         goalVelocities[i] = new YoDouble(prefix + "GoalVelocity" + i, registry);
         goalVelocities[i].set(getGoalVelocity(i));
      }
   }

   @Override
   void update(float dt)
   {
      super.setControlMode(controlMode.getValue());
      super.setGrip(grip.getValue());
      for (int i = 0; i < ACTUATOR_COUNT; ++i)
      {
         super.setGoalPosition(i, (float) goalPositions[i].getValue());
         super.setGoalVelocity(i, (float) goalVelocities[i].getValue());
      }

      super.update(dt);
   }

   @Override
   public void setControlMode(AbilityHandControlMode controlMode)
   {
      this.controlMode.set(controlMode);
   }

   @Override
   public void setGrip(AbilityHandGrip grip)
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
