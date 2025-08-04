package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.communication.packets.Packet;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.NewMessageListener;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class EZGripperMessageListener<T extends Packet<T>> implements NewMessageListener<T>
{
   private final Supplier<T> newMessageSupplier;
   private final T message;
   private final SideDependentList<T> gripperMessageList = new SideDependentList<>();
   private final List<Consumer<RobotSide>> onNewGripperRegisteredConsumers = new ArrayList<>();

   public EZGripperMessageListener(Supplier<T> newMessageInstantiator)
   {
      this.newMessageSupplier = newMessageInstantiator;
      message = newMessageInstantiator.get();
   }

   @Override
   public void onNewDataMessage(@SuppressWarnings("deprecation") Subscriber<T> subscriber)
   {
      subscriber.takeNextData(message, null);

      RobotSide gripperSide;
      if (message instanceof EZGripperCommand commandMessage)
      {
         gripperSide = RobotSide.fromByte(commandMessage.getRobotSide());
      }
      else if (message instanceof EZGripperState stateMessage)
      {
         gripperSide = RobotSide.fromByte(stateMessage.getRobotSide());
      }
      else
      {
         throw new IllegalArgumentException("Unrecognized message type: " + message);
      }

      if (gripperMessageList.containsKey(gripperSide))
      {
         synchronized (gripperMessageList.get(gripperSide))
         {
            gripperMessageList.get(gripperSide).set(message);
         }
      }
      else
      {
         T messageCopy = newMessageSupplier.get();
         messageCopy.set(message);
         gripperMessageList.put(gripperSide, messageCopy);
         for (int i = 0; i < onNewGripperRegisteredConsumers.size(); ++i)
         {
            onNewGripperRegisteredConsumers.get(i).accept(gripperSide);
         }
      }
   }

   public boolean readLatestMessage(RobotSide gripperSide, T messageToPack)
   {
      if (!gripperMessageList.containsKey(gripperSide))
         return false;

      synchronized (gripperMessageList.get(gripperSide))
      {
         messageToPack.set(gripperMessageList.get(gripperSide));
      }

      return true;
   }

   public void onNewHandRegistered(Consumer<RobotSide> gripperSideConsumer)
   {
      onNewGripperRegisteredConsumers.add(gripperSideConsumer);
   }
}
