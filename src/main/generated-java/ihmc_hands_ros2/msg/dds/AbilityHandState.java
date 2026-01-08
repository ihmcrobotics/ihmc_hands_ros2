package ihmc_hands_ros2.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AbilityHandState extends Packet<AbilityHandState> implements Settable<AbilityHandState>, EpsilonComparable<AbilityHandState>
{
   /**
            * The actuator positions in degrees
            */
   public float[] actuator_positions_;
   /**
            * The current finger velocities (deg/sec)
            */
   public float[] actuator_velocities_;
   /**
            * The actuator currents in amps
            */
   public float[] actuator_currents_;
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
            * The goal or max velocities (deg/sec)
            */
   public float[] goal_velocities_;

   public AbilityHandState()
   {
      actuator_positions_ = new float[6];

      actuator_velocities_ = new float[6];

      actuator_currents_ = new float[6];

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
      for(int i1 = 0; i1 < actuator_positions_.length; ++i1)
      {
            actuator_positions_[i1] = other.actuator_positions_[i1];

      }

      for(int i3 = 0; i3 < actuator_velocities_.length; ++i3)
      {
            actuator_velocities_[i3] = other.actuator_velocities_[i3];

      }

      for(int i5 = 0; i5 < actuator_currents_.length; ++i5)
      {
            actuator_currents_[i5] = other.actuator_currents_[i5];

      }

      for(int i7 = 0; i7 < touch_sensor_readings_.length; ++i7)
      {
            touch_sensor_readings_[i7] = other.touch_sensor_readings_[i7];

      }

      grip_stage_ = other.grip_stage_;

      for(int i9 = 0; i9 < goal_positions_.length; ++i9)
      {
            goal_positions_[i9] = other.goal_positions_[i9];

      }

      for(int i11 = 0; i11 < goal_velocities_.length; ++i11)
      {
            goal_velocities_[i11] = other.goal_velocities_[i11];

      }

   }


   /**
            * The actuator positions in degrees
            */
   public float[] getActuatorPositions()
   {
      return actuator_positions_;
   }


   /**
            * The current finger velocities (deg/sec)
            */
   public float[] getActuatorVelocities()
   {
      return actuator_velocities_;
   }


   /**
            * The actuator currents in amps
            */
   public float[] getActuatorCurrents()
   {
      return actuator_currents_;
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
            * The goal or max velocities (deg/sec)
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

      for(int i13 = 0; i13 < actuator_positions_.length; ++i13)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.actuator_positions_[i13], other.actuator_positions_[i13], epsilon)) return false;
      }

      for(int i15 = 0; i15 < actuator_velocities_.length; ++i15)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.actuator_velocities_[i15], other.actuator_velocities_[i15], epsilon)) return false;
      }

      for(int i17 = 0; i17 < actuator_currents_.length; ++i17)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.actuator_currents_[i17], other.actuator_currents_[i17], epsilon)) return false;
      }

      for(int i19 = 0; i19 < touch_sensor_readings_.length; ++i19)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.touch_sensor_readings_[i19], other.touch_sensor_readings_[i19], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grip_stage_, other.grip_stage_, epsilon)) return false;

      for(int i21 = 0; i21 < goal_positions_.length; ++i21)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_positions_[i21], other.goal_positions_[i21], epsilon)) return false;
      }

      for(int i23 = 0; i23 < goal_velocities_.length; ++i23)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_velocities_[i23], other.goal_velocities_[i23], epsilon)) return false;
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

      for(int i25 = 0; i25 < actuator_positions_.length; ++i25)
      {
                if(this.actuator_positions_[i25] != otherMyClass.actuator_positions_[i25]) return false;

      }
      for(int i27 = 0; i27 < actuator_velocities_.length; ++i27)
      {
                if(this.actuator_velocities_[i27] != otherMyClass.actuator_velocities_[i27]) return false;

      }
      for(int i29 = 0; i29 < actuator_currents_.length; ++i29)
      {
                if(this.actuator_currents_[i29] != otherMyClass.actuator_currents_[i29]) return false;

      }
      for(int i31 = 0; i31 < touch_sensor_readings_.length; ++i31)
      {
                if(this.touch_sensor_readings_[i31] != otherMyClass.touch_sensor_readings_[i31]) return false;

      }
      if(this.grip_stage_ != otherMyClass.grip_stage_) return false;

      for(int i33 = 0; i33 < goal_positions_.length; ++i33)
      {
                if(this.goal_positions_[i33] != otherMyClass.goal_positions_[i33]) return false;

      }
      for(int i35 = 0; i35 < goal_velocities_.length; ++i35)
      {
                if(this.goal_velocities_[i35] != otherMyClass.goal_velocities_[i35]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandState {");
      builder.append("actuator_positions=");
      builder.append(java.util.Arrays.toString(this.actuator_positions_));      builder.append(", ");
      builder.append("actuator_velocities=");
      builder.append(java.util.Arrays.toString(this.actuator_velocities_));      builder.append(", ");
      builder.append("actuator_currents=");
      builder.append(java.util.Arrays.toString(this.actuator_currents_));      builder.append(", ");
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
