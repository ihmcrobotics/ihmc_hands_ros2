package us.ihmc.handsros2.abilityHand;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.RealtimeROS2Node;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * <p>High level ROS 2 communication for the {@link AbilityHandInterface}. Communicates with low-level hardware control process.</p>
 * <p>Subscribes to {@link AbilityHandState} messages and publishes {@link AbilityHandCommand} messages.</p>
 */
public class AbilityHandROS2HardwareCommunication
{
   private final List<String> registeredHandSerialNumbers;

   private final RealtimeROS2Node node;

   private final AbilityHandMessageListener<AbilityHandState> stateListener;
   private final ROS2Subscription<AbilityHandState> stateSubscription;

   private final Map<String, AbilityHandCommand> commandMessages;
   private final ROS2Publisher<AbilityHandCommand> commandPublisher;

   public AbilityHandROS2HardwareCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public AbilityHandROS2HardwareCommunication(String nodeName, int domainId)
   {
      registeredHandSerialNumbers = Collections.synchronizedList(new ArrayList<>(2));
      commandMessages = new ConcurrentHashMap<>(2);

      ROS2NodeBuilder nodeBuilder = new ROS2NodeBuilder();
      if (domainId >= 0)
         nodeBuilder.domainId(domainId);
      node = nodeBuilder.buildRealtime(nodeName);

      stateListener = new AbilityHandMessageListener<>(AbilityHandState::new);
      stateListener.onNewHandRegistered(this::registerNewHand);
      stateSubscription = node.createSubscription(AbilityHandROS2API.STATE_TOPIC, stateListener);

      commandPublisher = node.createPublisher(AbilityHandROS2API.COMMAND_TOPIC);
   }

   private void registerNewHand(StringBuilder newHandSerialNumber)
   {
      String serialNumber = newHandSerialNumber.toString();
      AbilityHandCommand commandMessage = new AbilityHandCommand();
      commandMessage.setSerialNumber(serialNumber);
      commandMessages.put(serialNumber, commandMessage);
      registeredHandSerialNumbers.add(serialNumber);
   }

   /**
    * <p>Get the serial numbers of the available hands.</p>
    * <p>Treat the set as read-only.</p>
    *
    * @return Set of serial numbers of the available hands.
    */
   public Set<String> getAvailableHandSerialNumbers()
   {
      return commandMessages.keySet();
   }

   /**
    * <p>Get a synchronized list of the serial numbers of the available hands.</p>
    * <p>The list is created using {@link Collections#synchronizedList(List)},
    * thus a synchronized block must be used when iterating over the list.</p>
    * <p>Treat the list as read-only.</p>
    *
    * @return List of serial numbers of the available hands.
    */
   public List<String> getAvailableHandSerialNumbersList()
   {
      return registeredHandSerialNumbers;
   }

   /**
    * Read the latest state message of the specified hand.
    *
    * @param serialNumber  Serial number specifying the hand.
    * @param messageToPack Message to pack with the latest state.
    * @return {@code true} if a state message was available. {@code false} if no state had been received.
    */
   public boolean readState(String serialNumber, AbilityHandState messageToPack)
   {
      return stateListener.readLatestMessage(serialNumber, messageToPack);
   }

   /**
    * Read the latest state message of the specified hand.
    *
    * @param serialNumber Serial number specifying the hand.
    * @return A copy of the latest state message.
    */
   public AbilityHandState readState(String serialNumber)
   {
      AbilityHandState stateMessage = new AbilityHandState();
      if (readState(serialNumber, stateMessage))
         return stateMessage;

      return null;
   }

   /**
    * <p>Get the command message for the specified hand.</p>
    * <p>Use this method to set the desired command values.
    * Then publish the command using {@link #publishCommand(String)}.</p>
    *
    * @param handSerialNumber Serial number specifying the hand.
    * @return A reference to the command message for the specified hand.
    */
   public AbilityHandCommand getCommand(String handSerialNumber)
   {
      return commandMessages.get(handSerialNumber);
   }

   /**
    * Publish the command for the specified hand.
    *
    * @param handSerialNumber Serial number specifying the hand.
    * @return {@code true} if the message was published. {@code false} if the hand specified wasn't found.
    */
   public boolean publishCommand(String handSerialNumber)
   {
      AbilityHandCommand commandMessage = commandMessages.get(handSerialNumber);
      if (commandMessage != null)
      {
         commandPublisher.publish(commandMessage);
         return true;
      }

      return false;
   }

   /**
    * Start the communication.
    */
   public void start()
   {
      node.spin();
   }

   /**
    * Stop the communication. {@link #start()} can be called again after this method to re-start communication.
    */
   public void stop()
   {
      node.stopSpinning();
   }

   /**
    * Shut the communication down. {@link #start()} cannot be called again after this method.
    */
   public void shutdown()
   {
      stop();

      commandPublisher.remove();
      stateSubscription.remove();

      node.destroy();
   }
}
