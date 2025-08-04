package ihmc_hands_ros2.msg.dds;

/**
* 
* Topic data type of the struct "EZGripperState" defined in "EZGripperState_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EZGripperState_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EZGripperState_.idl instead.
*
*/
public class EZGripperStatePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_hands_ros2.msg.dds.EZGripperState>
{
   public static final java.lang.String name = "ihmc_hands_ros2::msg::dds_::EZGripperState_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "09f8687b13dcd926b261451144fcc193ecae6e9d679734922f0f5657ac5e4829";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_hands_ros2.msg.dds.EZGripperState data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_hands_ros2.msg.dds.EZGripperState data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_hands_ros2.msg.dds.EZGripperState data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_hands_ros2.msg.dds.EZGripperState data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_hands_ros2.msg.dds.EZGripperState data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_9(data.getOperationMode());

      cdr.write_type_9(data.getTemperature());

      cdr.write_type_5(data.getCurrentPosition());

      cdr.write_type_5(data.getCurrentEffort());

      cdr.write_type_9(data.getErrorCode());

      cdr.write_type_2(data.getRealtimeTick());

      cdr.write_type_7(data.getIsCalibrated());

   }

   public static void read(ihmc_hands_ros2.msg.dds.EZGripperState data, us.ihmc.idl.CDR cdr)
   {
      data.setRobotSide(cdr.read_type_9());
      	
      data.setOperationMode(cdr.read_type_9());
      	
      data.setTemperature(cdr.read_type_9());
      	
      data.setCurrentPosition(cdr.read_type_5());
      	
      data.setCurrentEffort(cdr.read_type_5());
      	
      data.setErrorCode(cdr.read_type_9());
      	
      data.setRealtimeTick(cdr.read_type_2());
      	
      data.setIsCalibrated(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(ihmc_hands_ros2.msg.dds.EZGripperState data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_9("operation_mode", data.getOperationMode());
      ser.write_type_9("temperature", data.getTemperature());
      ser.write_type_5("current_position", data.getCurrentPosition());
      ser.write_type_5("current_effort", data.getCurrentEffort());
      ser.write_type_9("error_code", data.getErrorCode());
      ser.write_type_2("realtime_tick", data.getRealtimeTick());
      ser.write_type_7("is_calibrated", data.getIsCalibrated());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_hands_ros2.msg.dds.EZGripperState data)
   {
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setOperationMode(ser.read_type_9("operation_mode"));
      data.setTemperature(ser.read_type_9("temperature"));
      data.setCurrentPosition(ser.read_type_5("current_position"));
      data.setCurrentEffort(ser.read_type_5("current_effort"));
      data.setErrorCode(ser.read_type_9("error_code"));
      data.setRealtimeTick(ser.read_type_2("realtime_tick"));
      data.setIsCalibrated(ser.read_type_7("is_calibrated"));
   }

   public static void staticCopy(ihmc_hands_ros2.msg.dds.EZGripperState src, ihmc_hands_ros2.msg.dds.EZGripperState dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_hands_ros2.msg.dds.EZGripperState createData()
   {
      return new ihmc_hands_ros2.msg.dds.EZGripperState();
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
   
   public void serialize(ihmc_hands_ros2.msg.dds.EZGripperState data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_hands_ros2.msg.dds.EZGripperState data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_hands_ros2.msg.dds.EZGripperState src, ihmc_hands_ros2.msg.dds.EZGripperState dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EZGripperStatePubSubType newInstance()
   {
      return new EZGripperStatePubSubType();
   }
}
