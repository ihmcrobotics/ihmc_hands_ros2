package ihmc_hands_ros2.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class EZGripperState extends Packet<EZGripperState> implements Settable<EZGripperState>, EpsilonComparable<EZGripperState>
{
   public static final byte LEFT = (byte) 0;
   public static final byte RIGHT = (byte) 1;
   public static final byte POSITION_CONTROL = (byte) 0;
   public static final byte CALIBRATION = (byte) 1;
   public static final byte ERROR_RESET = (byte) 2;
   public static final byte COOLDOWN = (byte) 3;
   /**
            * An identifier for the hand. Up to 8 characters long
            */
   public java.lang.StringBuilder identifier_;
   /**
            * Specifies the side of the robot of the gripper being referred to
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Specifies the current operation mode.
            */
   public byte operation_mode_ = (byte) 255;
   /**
            * Temperature of the Dynamixel in Celsius
            */
   public byte temperature_;
   /**
            * The current Dynamixel position
            * 0.0 = fully closed, 1.0 = fully open
            */
   public float current_position_;
   /**
            * The current amount of effort being used
            * 0.0 = no effort, 1.0 = maximum effort
            */
   public float current_effort_;
   /**
            * Dynamixel's error code
            * See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public byte error_code_;
   /**
            * Realtime tick of the Dynamixel
            * If this value isn't changing, communication with the gripper is broken
            */
   public int realtime_tick_;
   /**
            * Whether the hand has been calibrated
            */
   public boolean is_calibrated_;

   public EZGripperState()
   {
      identifier_ = new java.lang.StringBuilder(32);
   }

   public EZGripperState(EZGripperState other)
   {
      this();
      set(other);
   }

   public void set(EZGripperState other)
   {
      identifier_.setLength(0);
      identifier_.append(other.identifier_);

      robot_side_ = other.robot_side_;

      operation_mode_ = other.operation_mode_;

      temperature_ = other.temperature_;

      current_position_ = other.current_position_;

      current_effort_ = other.current_effort_;

      error_code_ = other.error_code_;

      realtime_tick_ = other.realtime_tick_;

      is_calibrated_ = other.is_calibrated_;

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
            * Specifies the side of the robot of the gripper being referred to
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot of the gripper being referred to
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * Specifies the current operation mode.
            */
   public void setOperationMode(byte operation_mode)
   {
      operation_mode_ = operation_mode;
   }
   /**
            * Specifies the current operation mode.
            */
   public byte getOperationMode()
   {
      return operation_mode_;
   }

   /**
            * Temperature of the Dynamixel in Celsius
            */
   public void setTemperature(byte temperature)
   {
      temperature_ = temperature;
   }
   /**
            * Temperature of the Dynamixel in Celsius
            */
   public byte getTemperature()
   {
      return temperature_;
   }

   /**
            * The current Dynamixel position
            * 0.0 = fully closed, 1.0 = fully open
            */
   public void setCurrentPosition(float current_position)
   {
      current_position_ = current_position;
   }
   /**
            * The current Dynamixel position
            * 0.0 = fully closed, 1.0 = fully open
            */
   public float getCurrentPosition()
   {
      return current_position_;
   }

   /**
            * The current amount of effort being used
            * 0.0 = no effort, 1.0 = maximum effort
            */
   public void setCurrentEffort(float current_effort)
   {
      current_effort_ = current_effort;
   }
   /**
            * The current amount of effort being used
            * 0.0 = no effort, 1.0 = maximum effort
            */
   public float getCurrentEffort()
   {
      return current_effort_;
   }

   /**
            * Dynamixel's error code
            * See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public void setErrorCode(byte error_code)
   {
      error_code_ = error_code;
   }
   /**
            * Dynamixel's error code
            * See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
            */
   public byte getErrorCode()
   {
      return error_code_;
   }

   /**
            * Realtime tick of the Dynamixel
            * If this value isn't changing, communication with the gripper is broken
            */
   public void setRealtimeTick(int realtime_tick)
   {
      realtime_tick_ = realtime_tick;
   }
   /**
            * Realtime tick of the Dynamixel
            * If this value isn't changing, communication with the gripper is broken
            */
   public int getRealtimeTick()
   {
      return realtime_tick_;
   }

   /**
            * Whether the hand has been calibrated
            */
   public void setIsCalibrated(boolean is_calibrated)
   {
      is_calibrated_ = is_calibrated;
   }
   /**
            * Whether the hand has been calibrated
            */
   public boolean getIsCalibrated()
   {
      return is_calibrated_;
   }


   public static Supplier<EZGripperStatePubSubType> getPubSubType()
   {
      return EZGripperStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EZGripperStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EZGripperState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.identifier_, other.identifier_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.operation_mode_, other.operation_mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_, other.temperature_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_, other.current_position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_effort_, other.current_effort_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.error_code_, other.error_code_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.realtime_tick_, other.realtime_tick_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_calibrated_, other.is_calibrated_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EZGripperState)) return false;

      EZGripperState otherMyClass = (EZGripperState) other;

      if (!us.ihmc.idl.IDLTools.equals(this.identifier_, otherMyClass.identifier_)) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.operation_mode_ != otherMyClass.operation_mode_) return false;

      if(this.temperature_ != otherMyClass.temperature_) return false;

      if(this.current_position_ != otherMyClass.current_position_) return false;

      if(this.current_effort_ != otherMyClass.current_effort_) return false;

      if(this.error_code_ != otherMyClass.error_code_) return false;

      if(this.realtime_tick_ != otherMyClass.realtime_tick_) return false;

      if(this.is_calibrated_ != otherMyClass.is_calibrated_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EZGripperState {");
      builder.append("identifier=");
      builder.append(this.identifier_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("operation_mode=");
      builder.append(this.operation_mode_);      builder.append(", ");
      builder.append("temperature=");
      builder.append(this.temperature_);      builder.append(", ");
      builder.append("current_position=");
      builder.append(this.current_position_);      builder.append(", ");
      builder.append("current_effort=");
      builder.append(this.current_effort_);      builder.append(", ");
      builder.append("error_code=");
      builder.append(this.error_code_);      builder.append(", ");
      builder.append("realtime_tick=");
      builder.append(this.realtime_tick_);      builder.append(", ");
      builder.append("is_calibrated=");
      builder.append(this.is_calibrated_);
      builder.append("}");
      return builder.toString();
   }
}
