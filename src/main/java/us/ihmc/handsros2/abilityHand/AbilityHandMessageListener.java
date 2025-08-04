package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.commons.lists.PairList;
import us.ihmc.communication.packets.Packet;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class AbilityHandMessageListener<T extends Packet<T>> implements NewMessageListener<T>
{
   private final Supplier<T> newMessageSupplier;
   private final T message;
   private final PairList<StringBuilder, T> handMessageList = new PairList<>();
   private final List<Consumer<StringBuilder>> onNewHandRegisteredConsumers = new ArrayList<>();

   public AbilityHandMessageListener(Supplier<T> newMessageSupplier)
   {
      this.newMessageSupplier = newMessageSupplier;
      message = newMessageSupplier.get();
   }

   @Override
   public void onNewDataMessage(@SuppressWarnings("deprecation") Subscriber<T> subscriber)
   {
      subscriber.takeNextData(message, null);

      StringBuilder serialNumber;
      if (message instanceof AbilityHandCommand commandMessage)
      {
         serialNumber = commandMessage.getSerialNumber();
      }
      else if (message instanceof AbilityHandState stateMessage)
      {
         serialNumber = stateMessage.getSerialNumber();
      }
      else
      {
         throw new IllegalArgumentException("Unrecognized message type: " + message);
      }

      for (int i = 0; i < handMessageList.size(); i++)
      {
         if (serialNumber.compareTo(handMessageList.first(i)) == 0)
         {
            synchronized (handMessageList.second(i))
            {
               handMessageList.second(i).set(message);
            }

            return;
         }
      }

      StringBuilder serialNumberCopy = new StringBuilder(serialNumber);
      T messageCopy = newMessageSupplier.get();
      messageCopy.set(message);
      handMessageList.add(serialNumberCopy, messageCopy);
      for (int i = 0; i < onNewHandRegisteredConsumers.size(); ++i)
      {
         onNewHandRegisteredConsumers.get(i).accept(serialNumberCopy);
      }
   }

   public boolean readLatestMessage(String serialNumber, T messageToPack)
   {
      for (int i = 0; i < handMessageList.size(); i++)
      {
         if (serialNumber.contentEquals(handMessageList.first(i)))
         {
            synchronized (handMessageList.second(i))
            {
               messageToPack.set(handMessageList.second(i));
            }

            return true;
         }
      }

      return false;
   }

   public void onNewHandRegistered(Consumer<StringBuilder> serialNumberConsumer)
   {
      onNewHandRegisteredConsumers.add(serialNumberConsumer);
   }
}
