package us.ihmc.handsros2;

import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

public class LatestMessageSubscription<T extends ROS2Message<T>>
{
   private final T latestMessage;
   private final ReadWriteLock lock = new ReentrantReadWriteLock();

   private final ROS2Node node;
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
      this.node = node;
      latestMessage = messageBuilder.get();
      timeSupplier = epochMillisSupplier;

      subscription = node.createSubscriptionSampler(topic, this::onMessage);
   }

   private void onMessage(T incoming)
   {
      lock.writeLock().lock();
      try
      {
         latestMessage.set(incoming);
      }
      finally
      {
         lock.writeLock().unlock();
      }

      latestMessageTimestamp = timeSupplier.getAsLong();

      if (!hasReceivedMessage)
         hasReceivedMessage = true;
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
      node.destroySubscription(subscription);
   }
}
