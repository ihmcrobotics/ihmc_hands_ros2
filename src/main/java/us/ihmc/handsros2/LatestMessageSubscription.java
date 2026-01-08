package us.ihmc.handsros2;

import us.ihmc.communication.packets.Packet;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

public class LatestMessageSubscription<T extends Packet<T>>
{
   private final T latestMessage;
   private final T incomingMessage;
   private final ReadWriteLock lock = new ReentrantReadWriteLock();

   private final ROS2Subscription<T> subscription;

   private final LongSupplier timeSupplier;

   private boolean hasReceivedMessage = false;
   private volatile long latestMessageTimestamp = 0;

   public LatestMessageSubscription(ROS2Node node, ROS2Topic<T> topic, Supplier<T> messageBuilder)
   {
      this(node, topic, messageBuilder, System::currentTimeMillis);
   }

   public LatestMessageSubscription(ROS2Node node, ROS2Topic<T> topic, Supplier<T> messageBuilder, LongSupplier epochMillisSupplier)
   {
      latestMessage = messageBuilder.get();
      incomingMessage = messageBuilder.get();

      timeSupplier = epochMillisSupplier;

      subscription = node.createSubscription(topic, this::receiveMessage);
   }

   public void readLatestMessage(T messageToPack)
   {
      lock.readLock().lock();
      try
      {
         messageToPack.set(latestMessage);
      }
      finally
      {
         lock.readLock().unlock();
      }
   }

   public long getLatestMessageTimestamp()
   {
      return latestMessageTimestamp;
   }

   public boolean hasReceivedAMessage()
   {
      return hasReceivedMessage;
   }

   public void remove()
   {
      subscription.remove();
   }

   private void receiveMessage(@SuppressWarnings("deprecation") Subscriber<T> subscriber)
   {
      if (subscriber.takeNextData(incomingMessage, null))
      {
         lock.writeLock().lock();
         try
         {
            latestMessage.set(incomingMessage);
         }
         finally
         {
            lock.writeLock().unlock();
         }

         latestMessageTimestamp = timeSupplier.getAsLong();

         if (!hasReceivedMessage)
            hasReceivedMessage = true;
      }
   }
}
