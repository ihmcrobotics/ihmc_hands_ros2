package ihmc_hands_ros2.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AbilityHandCommand extends Packet<AbilityHandCommand> implements Settable<AbilityHandCommand>, EpsilonComparable<AbilityHandCommand>
{
   public static final byte POSITION_CONTROL = (byte) 0;
   public static final byte VELOCITY_CONTROL = (byte) 1;
   public static final byte GRIP_CONTROL = (byte) 2;
   public static final byte OPEN_GRIP = (byte) 0;
   public static final byte CLOSE_GRIP = (byte) 1;
   public static final byte PINCH_GRIP = (byte) 2;
   public static final byte FLAT_GRIP = (byte) 3;
   public static final byte HOOK_GRIP = (byte) 4;
   public static final byte RELAX_GRIP = (byte) 5;
   public static final byte DOOR_LEVER_OPEN_GRIP = (byte) 6;
   public static final byte DOOR_LEVER_CLOSE_GRIP = (byte) 7;
   public static final byte DOOR_LEVER_CRUSH_GRIP = (byte) 8;
   public static final byte KEY_OPEN_GRIP = (byte) 9;
   public static final byte KEY_CLOSE_GRIP = (byte) 10;
   /**
            * The hand's serial number. E.g. 24ABH265
            */
   public java.lang.StringBuilder identifier_;
   /**
            * Specifies the control mode (ordinal of AbilityHandControlMode)
            * Default = position control
            */
   public byte control_mode_;
   /**
            * Goal velocities when executing commands in velocity control mode
            * Max velocities when in position or grip control modes
            */
   public float[] goal_velocities_;
   /**
            * Goal positions when using POSITION_CONTROL mode
            */
   public float[] goal_positions_;
   /**
            * Grip to execute in GRIP_CONTROL mode
            */
   public byte grip_;

   public AbilityHandCommand()
   {
      identifier_ = new java.lang.StringBuilder(32);
      goal_velocities_ = new float[6];

      goal_positions_ = new float[6];

   }

   public AbilityHandCommand(AbilityHandCommand other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandCommand other)
   {
      identifier_.setLength(0);
      identifier_.append(other.identifier_);

      control_mode_ = other.control_mode_;

      for(int i1 = 0; i1 < goal_velocities_.length; ++i1)
      {
            goal_velocities_[i1] = other.goal_velocities_[i1];

      }

      for(int i3 = 0; i3 < goal_positions_.length; ++i3)
      {
            goal_positions_[i3] = other.goal_positions_[i3];

      }

      grip_ = other.grip_;

   }

   /**
            * The hand's serial number. E.g. 24ABH265
            */
   public void setIdentifier(java.lang.String identifier)
   {
      identifier_.setLength(0);
      identifier_.append(identifier);
   }

   /**
            * The hand's serial number. E.g. 24ABH265
            */
   public java.lang.String getIdentifierAsString()
   {
      return getIdentifier().toString();
   }
   /**
            * The hand's serial number. E.g. 24ABH265
            */
   public java.lang.StringBuilder getIdentifier()
   {
      return identifier_;
   }

   /**
            * Specifies the control mode (ordinal of AbilityHandControlMode)
            * Default = position control
            */
   public void setControlMode(byte control_mode)
   {
      control_mode_ = control_mode;
   }
   /**
            * Specifies the control mode (ordinal of AbilityHandControlMode)
            * Default = position control
            */
   public byte getControlMode()
   {
      return control_mode_;
   }


   /**
            * Goal velocities when executing commands in velocity control mode
            * Max velocities when in position or grip control modes
            */
   public float[] getGoalVelocities()
   {
      return goal_velocities_;
   }


   /**
            * Goal positions when using POSITION_CONTROL mode
            */
   public float[] getGoalPositions()
   {
      return goal_positions_;
   }

   /**
            * Grip to execute in GRIP_CONTROL mode
            */
   public void setGrip(byte grip)
   {
      grip_ = grip;
   }
   /**
            * Grip to execute in GRIP_CONTROL mode
            */
   public byte getGrip()
   {
      return grip_;
   }


   public static Supplier<AbilityHandCommandPubSubType> getPubSubType()
   {
      return AbilityHandCommandPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AbilityHandCommandPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AbilityHandCommand other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.identifier_, other.identifier_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.control_mode_, other.control_mode_, epsilon)) return false;

      for(int i5 = 0; i5 < goal_velocities_.length; ++i5)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_velocities_[i5], other.goal_velocities_[i5], epsilon)) return false;
      }

      for(int i7 = 0; i7 < goal_positions_.length; ++i7)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_positions_[i7], other.goal_positions_[i7], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grip_, other.grip_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AbilityHandCommand)) return false;

      AbilityHandCommand otherMyClass = (AbilityHandCommand) other;

      if (!us.ihmc.idl.IDLTools.equals(this.identifier_, otherMyClass.identifier_)) return false;

      if(this.control_mode_ != otherMyClass.control_mode_) return false;

      for(int i9 = 0; i9 < goal_velocities_.length; ++i9)
      {
                if(this.goal_velocities_[i9] != otherMyClass.goal_velocities_[i9]) return false;

      }
      for(int i11 = 0; i11 < goal_positions_.length; ++i11)
      {
                if(this.goal_positions_[i11] != otherMyClass.goal_positions_[i11]) return false;

      }
      if(this.grip_ != otherMyClass.grip_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandCommand {");
      builder.append("identifier=");
      builder.append(this.identifier_);      builder.append(", ");
      builder.append("control_mode=");
      builder.append(this.control_mode_);      builder.append(", ");
      builder.append("goal_velocities=");
      builder.append(java.util.Arrays.toString(this.goal_velocities_));      builder.append(", ");
      builder.append("goal_positions=");
      builder.append(java.util.Arrays.toString(this.goal_positions_));      builder.append(", ");
      builder.append("grip=");
      builder.append(this.grip_);
      builder.append("}");
      return builder.toString();
   }
}
