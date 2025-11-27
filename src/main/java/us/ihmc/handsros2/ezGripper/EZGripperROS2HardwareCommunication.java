package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.handsros2.HandMessageListener;
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
 * <p>Controller side ROS 2 communication for the {@link EZGripperInterface}. Communicates with low-level hardware control process.</p>
 * <p>Subscribes to {@link EZGripperState} messages and publishes {@link EZGripperCommand} messages.</p>
 */
public class EZGripperROS2HardwareCommunication
{
   private final List<String> registeredHandIdentifiers;

   private final RealtimeROS2Node node;

   private final HandMessageListener<EZGripperState> stateListener;
   private final ROS2Subscription<EZGripperState> stateSubscription;

   private final Map<String, EZGripperCommand> commandMessages;
   private final ROS2Publisher<EZGripperCommand> commandPublisher;

   public EZGripperROS2HardwareCommunication(String nodeName)
   {
      this(nodeName, -1);
   }

   public EZGripperROS2HardwareCommunication(String nodeName, int domainId)
   {
      registeredHandIdentifiers = Collections.synchronizedList(new ArrayList<>(2));
      commandMessages = new ConcurrentHashMap<>(2);

      ROS2NodeBuilder nodeBuilder = new ROS2NodeBuilder();
      if (domainId >= 0)
         nodeBuilder.domainId(domainId);
      node = nodeBuilder.buildRealtime(nodeName);

      stateListener = new HandMessageListener<>(EZGripperState::new);
      stateListener.onNewHandRegistered(this::registerNewHand);
      stateSubscription = node.createSubscription(EZGripperROS2API.STATE_TOPIC, stateListener);

      commandPublisher = node.createPublisher(EZGripperROS2API.COMMAND_TOPIC);
   }

   private void registerNewHand(StringBuilder newGripperIdentifier)
   {
      String identifier = newGripperIdentifier.toString();
      EZGripperCommand commandMessage = new EZGripperCommand();
      commandMessage.setIdentifier(identifier);
      commandMessages.put(identifier, commandMessage);
      registeredHandIdentifiers.add(identifier);
   }

   /**
    * <p>Get the identifiers of the available hands.</p>
    * <p>Treat the set as read-only.</p>
    *
    * @return Set of identifiers of the available hands.
    */
   public Set<String> getAvailableHands()
   {
      return commandMessages.keySet();
   }

   /**
    * <p>Get a synchronized list of the identifiers of the available hands.</p>
    * <p>The list should be created using {@link java.util.Collections#synchronizedList(List)},
    * thus a synchronized block must be used when iterating over the list/</p>
    * <p>Tread the list as read-only.</p>
    *
    * @return List of identifiers of the available hands.
    */
   public List<String> getAvailableHandList()
   {
      return registeredHandIdentifiers;
   }

   /**
    * Read the latest state message of the specified hand.
    *
    * @param identifier  Identifier specifying the hand.
    * @param stateToPack State message to pack with the latest state.
    * @return {@code true} if a state message was available. {@code false} if no state had been received.
    */
   public boolean readState(String identifier, EZGripperState messageToPack)
   {
      return stateListener.readLatestMessage(identifier, messageToPack);
   }

   /**
    * Read the latest state message of the specified hand.
    *
    * @param identifier Identifier specifying the hand.
    * @return A copy of the latest state message.
    */
   public EZGripperState readState(String identifier)
   {
      EZGripperState stateMessage = new EZGripperState();
      if (readState(identifier, stateMessage))
         return stateMessage;

      return null;
   }

   /**
    * <p>Get the command message for the specified hand.</p>
    * <p>Use this method to set the desired command values.
    * Then publish the command using {@link #publishCommand(String)}.</p>
    *
    * @param identifier Identifier specifying the hand.
    * @return A reference to the command message for the specified hand.
    */
   public EZGripperCommand getCommand(String identifier)
   {
      return commandMessages.get(identifier);
   }

   /**
    * Publish the command for the specified hand.
    *
    * @param identifier Serial number specifying the hand.
    * @return {@code true} if the message was published. {@code false} if the hand specified wasn't found.
    */
   public boolean publishCommand(String identifier)
   {
      EZGripperCommand commandMessage = commandMessages.get(identifier);
      if (commandMessage == null)
         return false;

      commandPublisher.publish(commandMessage);
      return true;
   }

   /**
    * Start the communication.
    */
   public void start()
   {
      node.spin();
   }

   /**
    * Shut the communication down. {@link #start()} cannot be called again after this method.
    */
   public void shutdown()
   {
      node.stopSpinning();

      commandPublisher.remove();
      stateSubscription.remove();

      node.destroy();
   }
}
