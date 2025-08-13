package ihmc_hands_ros2.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class EZGripperCommand extends Packet<EZGripperCommand> implements Settable<EZGripperCommand>, EpsilonComparable<EZGripperCommand>
{
   public static final byte POSITION_CONTROL = (byte) 0;
   public static final byte CALIBRATION = (byte) 1;
   public static final byte ERROR_RESET = (byte) 2;
   /**
            * An identifier for the hand. Up to 8 characters long
            */
   public java.lang.StringBuilder identifier_;
   /**
            * Specifies the desired operation mode.
            */
   public byte operation_mode_ = (byte) 255;
   /**
            * The temperature limit. Once reached, the gripper will go into cooldown mode
            * until the actuator's temperature falls by 10% of the temperature limit.
            * By default set to 75. It is not recommended going above 80.
            * Automatic cooldown can be disabled by setting this value to 255.
            */
   public byte temperature_limit_ = (byte) 75;
   /**
            * The goal position
            * 0.0 = fully closed, 1.0 = fully open
            */
   public float goal_position_;
   /**
            * Maximum effort to put into achieving the goal position
            * 0.0 = no effort (fingers will not move), 1.0 = maximum effort (can quickly overheat actuator)
            * 0.3 is a reasonable normal value, and it is recommended not to exceed 0.8.
            */
   public float max_effort_;
   /**
            * Keeping the torque off when not needed can help keep the gripper's temperature down
            */
   public boolean torque_on_;

   public EZGripperCommand()
   {
      identifier_ = new java.lang.StringBuilder(8);
   }

   public EZGripperCommand(EZGripperCommand other)
   {
      this();
      set(other);
   }

   public void set(EZGripperCommand other)
   {
      identifier_.setLength(0);
      identifier_.append(other.identifier_);

      operation_mode_ = other.operation_mode_;

      temperature_limit_ = other.temperature_limit_;

      goal_position_ = other.goal_position_;

      max_effort_ = other.max_effort_;

      torque_on_ = other.torque_on_;

   }

   /**
            * An identifier for the hand. Up to 8 characters long
            */
   public void setIdentifier(java.lang.String identifier)
   {
      identifier_.setLength(0);
      identifier_.append(identifier);
   }

   /**
            * An identifier for the hand. Up to 8 characters long
            */
   public java.lang.String getIdentifierAsString()
   {
      return getIdentifier().toString();
   }
   /**
            * An identifier for the hand. Up to 8 characters long
            */
   public java.lang.StringBuilder getIdentifier()
   {
      return identifier_;
   }

   /**
            * Specifies the desired operation mode.
            */
   public void setOperationMode(byte operation_mode)
   {
      operation_mode_ = operation_mode;
   }
   /**
            * Specifies the desired operation mode.
            */
   public byte getOperationMode()
   {
      return operation_mode_;
   }

   /**
            * The temperature limit. Once reached, the gripper will go into cooldown mode
            * until the actuator's temperature falls by 10% of the temperature limit.
            * By default set to 75. It is not recommended going above 80.
            * Automatic cooldown can be disabled by setting this value to 255.
            */
   public void setTemperatureLimit(byte temperature_limit)
   {
      temperature_limit_ = temperature_limit;
   }
   /**
            * The temperature limit. Once reached, the gripper will go into cooldown mode
            * until the actuator's temperature falls by 10% of the temperature limit.
            * By default set to 75. It is not recommended going above 80.
            * Automatic cooldown can be disabled by setting this value to 255.
            */
   public byte getTemperatureLimit()
   {
      return temperature_limit_;
   }

   /**
            * The goal position
            * 0.0 = fully closed, 1.0 = fully open
            */
   public void setGoalPosition(float goal_position)
   {
      goal_position_ = goal_position;
   }
   /**
            * The goal position
            * 0.0 = fully closed, 1.0 = fully open
            */
   public float getGoalPosition()
   {
      return goal_position_;
   }

   /**
            * Maximum effort to put into achieving the goal position
            * 0.0 = no effort (fingers will not move), 1.0 = maximum effort (can quickly overheat actuator)
            * 0.3 is a reasonable normal value, and it is recommended not to exceed 0.8.
            */
   public void setMaxEffort(float max_effort)
   {
      max_effort_ = max_effort;
   }
   /**
            * Maximum effort to put into achieving the goal position
            * 0.0 = no effort (fingers will not move), 1.0 = maximum effort (can quickly overheat actuator)
            * 0.3 is a reasonable normal value, and it is recommended not to exceed 0.8.
            */
   public float getMaxEffort()
   {
      return max_effort_;
   }

   /**
            * Keeping the torque off when not needed can help keep the gripper's temperature down
            */
   public void setTorqueOn(boolean torque_on)
   {
      torque_on_ = torque_on;
   }
   /**
            * Keeping the torque off when not needed can help keep the gripper's temperature down
            */
   public boolean getTorqueOn()
   {
      return torque_on_;
   }


   public static Supplier<EZGripperCommandPubSubType> getPubSubType()
   {
      return EZGripperCommandPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EZGripperCommandPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EZGripperCommand other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.identifier_, other.identifier_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.operation_mode_, other.operation_mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_limit_, other.temperature_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_position_, other.goal_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_effort_, other.max_effort_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.torque_on_, other.torque_on_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EZGripperCommand)) return false;

      EZGripperCommand otherMyClass = (EZGripperCommand) other;

      if (!us.ihmc.idl.IDLTools.equals(this.identifier_, otherMyClass.identifier_)) return false;

      if(this.operation_mode_ != otherMyClass.operation_mode_) return false;

      if(this.temperature_limit_ != otherMyClass.temperature_limit_) return false;

      if(this.goal_position_ != otherMyClass.goal_position_) return false;

      if(this.max_effort_ != otherMyClass.max_effort_) return false;

      if(this.torque_on_ != otherMyClass.torque_on_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EZGripperCommand {");
      builder.append("identifier=");
      builder.append(this.identifier_);      builder.append(", ");
      builder.append("operation_mode=");
      builder.append(this.operation_mode_);      builder.append(", ");
      builder.append("temperature_limit=");
      builder.append(this.temperature_limit_);      builder.append(", ");
      builder.append("goal_position=");
      builder.append(this.goal_position_);      builder.append(", ");
      builder.append("max_effort=");
      builder.append(this.max_effort_);      builder.append(", ");
      builder.append("torque_on=");
      builder.append(this.torque_on_);
      builder.append("}");
      return builder.toString();
   }
}
