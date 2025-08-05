package us.ihmc.handsros2;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.commons.lists.PairList;
import us.ihmc.communication.packets.Packet;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A {@link NewMessageListener} that listens to hand command and state messages.
 *
 * @param <T> The type of hand message to listen to.
 */
public class HandMessageListener<T extends Packet<T>> implements NewMessageListener<T>
{
   private final Supplier<T> newMessageSupplier;
   private final T message;
   private final PairList<StringBuilder, T> handMessageList = new PairList<>();
   private final List<Consumer<StringBuilder>> onNewHandRegisteredConsumers = new ArrayList<>();

   public HandMessageListener(Supplier<T> newMessageSupplier)
   {
      this.newMessageSupplier = newMessageSupplier;
      message = newMessageSupplier.get();
   }

   @Override
   public void onNewDataMessage(@SuppressWarnings("deprecation") Subscriber<T> subscriber)
   {
      subscriber.takeNextData(message, null);
      StringBuilder identifier = getIdentifier(message);

      if (identifier == null)
      {
         LogTools.error("Received an unknown message type: {}", message.getClass().getSimpleName());
         return;
      }

      StringBuilder identifierCopy = new StringBuilder(identifier);
      T messageCopy = newMessageSupplier.get();
      messageCopy.set(message);

      handMessageList.add(identifierCopy, messageCopy);
      for (int i = 0; i < onNewHandRegisteredConsumers.size(); ++i)
      {
         onNewHandRegisteredConsumers.get(i).accept(identifierCopy);
      }
   }

   private StringBuilder getIdentifier(T message)
   {
      // Ability Hand message
      if (message instanceof AbilityHandCommand command)
         return command.getSerialNumber();
      else if (message instanceof AbilityHandState state)
         return state.getSerialNumber();
      // EZGripper messages
      else if (message instanceof EZGripperCommand command)
         return command.getIdentifier();
      else if (message instanceof EZGripperState state)
         return state.getIdentifier();

      return null;
   }

   /**
    * Read the latest message received for the hand specified by its identifier.
    *
    * @param identifier    The hand's identifier.
    * @param messageToPack The message to read into.
    * @return {@code true} if a message was received for the specified hand, and thus packed into the passed in message object.
    *       {@code false} if no messages have been received for the specified hand.
    */
   public boolean readLatestMessage(String identifier, T messageToPack)
   {
      for (int i = 0; i < handMessageList.size(); i++)
      {
         if (identifier.contentEquals(handMessageList.first(i)))
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
