package us.ihmc.handsros2;

import us.ihmc.jros2.ROS2Message;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.util.function.LongSupplier;
import java.util.function.Supplier;

public class LatestMessageSubscription<T extends ROS2Message<T>>
{
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
      this(node, topic, topic.getQoS(), epochMillisSupplier);
   }

   public LatestMessageSubscription(ROS2Node node, ROS2Topic<T> topic, ROS2QoSProfile qosProfile, LongSupplier epochMillisSupplier)
   {
      this.node = node;
      timeSupplier = epochMillisSupplier;
      subscription = node.createSubscription(topic, qosProfile);
   }

   public boolean readLatestMessage(T messageToPack)
   {
      if (subscription.readLatest(messageToPack) > 0)
      {
         latestMessageTimestamp = timeSupplier.getAsLong();
         hasReceivedMessage = true;
         return true;
      }

      return false;
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
