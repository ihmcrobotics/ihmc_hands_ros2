package ihmc_hands_ros2.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AbilityHandState extends Packet<AbilityHandState> implements Settable<AbilityHandState>, EpsilonComparable<AbilityHandState>
{
   public static final byte LEFT = (byte) 0;
   public static final byte RIGHT = (byte) 1;
   /**
            * The hand's serial number. E.g. 24ABH265
            */
   public java.lang.StringBuilder identifier_;
   /**
            * Specifies whether the hand is a left or right hand
            */
   public byte hand_side_ = (byte) 255;
   /**
            * The actuator positions in degrees
            */
   public float[] actuator_positions_;
   /**
            * The touch sensor pressure readings in Newtons
            */
   public float[] touch_sensor_readings_;
   /**
            * If grip control mode, the current grip stage
            */
   public int grip_stage_;
   /**
            * The goal positions in degrees
            */
   public float[] goal_positions_;
   /**
            * The goal velocities (units unknown, 30 is slow, 300 is really fast)
            */
   public float[] goal_velocities_;

   public AbilityHandState()
   {
      identifier_ = new java.lang.StringBuilder(32);
      actuator_positions_ = new float[6];

      touch_sensor_readings_ = new float[30];

      goal_positions_ = new float[6];

      goal_velocities_ = new float[6];

   }

   public AbilityHandState(AbilityHandState other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandState other)
   {
      identifier_.setLength(0);
      identifier_.append(other.identifier_);

      hand_side_ = other.hand_side_;

      for(int i1 = 0; i1 < actuator_positions_.length; ++i1)
      {
            actuator_positions_[i1] = other.actuator_positions_[i1];

      }

      for(int i3 = 0; i3 < touch_sensor_readings_.length; ++i3)
      {
            touch_sensor_readings_[i3] = other.touch_sensor_readings_[i3];

      }

      grip_stage_ = other.grip_stage_;

      for(int i5 = 0; i5 < goal_positions_.length; ++i5)
      {
            goal_positions_[i5] = other.goal_positions_[i5];

      }

      for(int i7 = 0; i7 < goal_velocities_.length; ++i7)
      {
            goal_velocities_[i7] = other.goal_velocities_[i7];

      }

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
            * Specifies whether the hand is a left or right hand
            */
   public void setHandSide(byte hand_side)
   {
      hand_side_ = hand_side;
   }
   /**
            * Specifies whether the hand is a left or right hand
            */
   public byte getHandSide()
   {
      return hand_side_;
   }


   /**
            * The actuator positions in degrees
            */
   public float[] getActuatorPositions()
   {
      return actuator_positions_;
   }


   /**
            * The touch sensor pressure readings in Newtons
            */
   public float[] getTouchSensorReadings()
   {
      return touch_sensor_readings_;
   }

   /**
            * If grip control mode, the current grip stage
            */
   public void setGripStage(int grip_stage)
   {
      grip_stage_ = grip_stage;
   }
   /**
            * If grip control mode, the current grip stage
            */
   public int getGripStage()
   {
      return grip_stage_;
   }


   /**
            * The goal positions in degrees
            */
   public float[] getGoalPositions()
   {
      return goal_positions_;
   }


   /**
            * The goal velocities (units unknown, 30 is slow, 300 is really fast)
            */
   public float[] getGoalVelocities()
   {
      return goal_velocities_;
   }


   public static Supplier<AbilityHandStatePubSubType> getPubSubType()
   {
      return AbilityHandStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AbilityHandStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AbilityHandState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.identifier_, other.identifier_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.hand_side_, other.hand_side_, epsilon)) return false;

      for(int i9 = 0; i9 < actuator_positions_.length; ++i9)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.actuator_positions_[i9], other.actuator_positions_[i9], epsilon)) return false;
      }

      for(int i11 = 0; i11 < touch_sensor_readings_.length; ++i11)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.touch_sensor_readings_[i11], other.touch_sensor_readings_[i11], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grip_stage_, other.grip_stage_, epsilon)) return false;

      for(int i13 = 0; i13 < goal_positions_.length; ++i13)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_positions_[i13], other.goal_positions_[i13], epsilon)) return false;
      }

      for(int i15 = 0; i15 < goal_velocities_.length; ++i15)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_velocities_[i15], other.goal_velocities_[i15], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AbilityHandState)) return false;

      AbilityHandState otherMyClass = (AbilityHandState) other;

      if (!us.ihmc.idl.IDLTools.equals(this.identifier_, otherMyClass.identifier_)) return false;

      if(this.hand_side_ != otherMyClass.hand_side_) return false;

      for(int i17 = 0; i17 < actuator_positions_.length; ++i17)
      {
                if(this.actuator_positions_[i17] != otherMyClass.actuator_positions_[i17]) return false;

      }
      for(int i19 = 0; i19 < touch_sensor_readings_.length; ++i19)
      {
                if(this.touch_sensor_readings_[i19] != otherMyClass.touch_sensor_readings_[i19]) return false;

      }
      if(this.grip_stage_ != otherMyClass.grip_stage_) return false;

      for(int i21 = 0; i21 < goal_positions_.length; ++i21)
      {
                if(this.goal_positions_[i21] != otherMyClass.goal_positions_[i21]) return false;

      }
      for(int i23 = 0; i23 < goal_velocities_.length; ++i23)
      {
                if(this.goal_velocities_[i23] != otherMyClass.goal_velocities_[i23]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandState {");
      builder.append("identifier=");
      builder.append(this.identifier_);      builder.append(", ");
      builder.append("hand_side=");
      builder.append(this.hand_side_);      builder.append(", ");
      builder.append("actuator_positions=");
      builder.append(java.util.Arrays.toString(this.actuator_positions_));      builder.append(", ");
      builder.append("touch_sensor_readings=");
      builder.append(java.util.Arrays.toString(this.touch_sensor_readings_));      builder.append(", ");
      builder.append("grip_stage=");
      builder.append(this.grip_stage_);      builder.append(", ");
      builder.append("goal_positions=");
      builder.append(java.util.Arrays.toString(this.goal_positions_));      builder.append(", ");
      builder.append("goal_velocities=");
      builder.append(java.util.Arrays.toString(this.goal_velocities_));
      builder.append("}");
      return builder.toString();
   }
}
