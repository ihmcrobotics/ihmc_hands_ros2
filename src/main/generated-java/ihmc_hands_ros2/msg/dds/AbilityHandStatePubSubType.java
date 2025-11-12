package ihmc_hands_ros2.msg.dds;

/**
* 
* Topic data type of the struct "AbilityHandState" defined in "AbilityHandState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AbilityHandState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AbilityHandState_.idl instead.
*
*/
public class AbilityHandStatePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_hands_ros2.msg.dds.AbilityHandState>
{
   public static final java.lang.String name = "ihmc_hands_ros2::msg::dds_::AbilityHandState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "98dd371258934794f8589bdf19cb16186bf6237ec99d566f8e800a21e3d852d9";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_hands_ros2.msg.dds.AbilityHandState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_hands_ros2.msg.dds.AbilityHandState data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 32 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((30) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_hands_ros2.msg.dds.AbilityHandState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_hands_ros2.msg.dds.AbilityHandState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getIdentifier().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((30) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += ((6) * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_hands_ros2.msg.dds.AbilityHandState data, us.ihmc.idl.CDR cdr)
   {
      if(data.getIdentifier().length() <= 32)
      cdr.write_type_d(data.getIdentifier());else
          throw new RuntimeException("identifier field exceeds the maximum length: %d > %d".formatted(data.getIdentifier().length(), 32));

      cdr.write_type_9(data.getHandSide());

      for(int i0 = 0; i0 < data.getActuatorPositions().length; ++i0)
      {
        	cdr.write_type_5(data.getActuatorPositions()[i0]);	
      }

      for(int i0 = 0; i0 < data.getTouchSensorReadings().length; ++i0)
      {
        	cdr.write_type_5(data.getTouchSensorReadings()[i0]);	
      }

      cdr.write_type_2(data.getGripStage());

      for(int i0 = 0; i0 < data.getGoalPositions().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalPositions()[i0]);	
      }

      for(int i0 = 0; i0 < data.getGoalVelocities().length; ++i0)
      {
        	cdr.write_type_5(data.getGoalVelocities()[i0]);	
      }

   }

   public static void read(ihmc_hands_ros2.msg.dds.AbilityHandState data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getIdentifier());	
      data.setHandSide(cdr.read_type_9());
      	
      for(int i0 = 0; i0 < data.getActuatorPositions().length; ++i0)
      {
        	data.getActuatorPositions()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getTouchSensorReadings().length; ++i0)
      {
        	data.getTouchSensorReadings()[i0] = cdr.read_type_5();
        	
      }
      	
      data.setGripStage(cdr.read_type_2());
      	
      for(int i0 = 0; i0 < data.getGoalPositions().length; ++i0)
      {
        	data.getGoalPositions()[i0] = cdr.read_type_5();
        	
      }
      	
      for(int i0 = 0; i0 < data.getGoalVelocities().length; ++i0)
      {
        	data.getGoalVelocities()[i0] = cdr.read_type_5();
        	
      }
      	

   }

   @Override
   public final void serialize(ihmc_hands_ros2.msg.dds.AbilityHandState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("identifier", data.getIdentifier());
      ser.write_type_9("hand_side", data.getHandSide());
      ser.write_type_f("actuator_positions", data.getActuatorPositions());
      ser.write_type_f("touch_sensor_readings", data.getTouchSensorReadings());
      ser.write_type_2("grip_stage", data.getGripStage());
      ser.write_type_f("goal_positions", data.getGoalPositions());
      ser.write_type_f("goal_velocities", data.getGoalVelocities());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_hands_ros2.msg.dds.AbilityHandState data)
   {
      ser.read_type_d("identifier", data.getIdentifier());
      data.setHandSide(ser.read_type_9("hand_side"));
      ser.read_type_f("actuator_positions", data.getActuatorPositions());
      ser.read_type_f("touch_sensor_readings", data.getTouchSensorReadings());
      data.setGripStage(ser.read_type_2("grip_stage"));
      ser.read_type_f("goal_positions", data.getGoalPositions());
      ser.read_type_f("goal_velocities", data.getGoalVelocities());
   }

   public static void staticCopy(ihmc_hands_ros2.msg.dds.AbilityHandState src, ihmc_hands_ros2.msg.dds.AbilityHandState dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_hands_ros2.msg.dds.AbilityHandState createData()
   {
      return new ihmc_hands_ros2.msg.dds.AbilityHandState();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(ihmc_hands_ros2.msg.dds.AbilityHandState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_hands_ros2.msg.dds.AbilityHandState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_hands_ros2.msg.dds.AbilityHandState src, ihmc_hands_ros2.msg.dds.AbilityHandState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AbilityHandStatePubSubType newInstance()
   {
      return new AbilityHandStatePubSubType();
   }
}
