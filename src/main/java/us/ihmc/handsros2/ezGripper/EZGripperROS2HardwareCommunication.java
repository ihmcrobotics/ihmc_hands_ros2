package us.ihmc.handsros2.ezGripper;

import ihmc_hands_ros2.msg.dds.EZGripperCommand;
import ihmc_hands_ros2.msg.dds.EZGripperState;
import us.ihmc.handsros2.HandMessageListener;
import us.ihmc.handsros2.HandROS2HardwareCommunication;
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
public class EZGripperROS2HardwareCommunication implements HandROS2HardwareCommunication<EZGripperCommand, EZGripperState>
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

   /** {@inheritDoc} */
   @Override
   public Set<String> getAvailableHands()
   {
      return commandMessages.keySet();
   }

   /** {@inheritDoc} */
   @Override
   public List<String> getAvailableHandList()
   {
      return registeredHandIdentifiers;
   }

   /** {@inheritDoc} */
   @Override
   public boolean readState(String identifier, EZGripperState messageToPack)
   {
      return stateListener.readLatestMessage(identifier, messageToPack);
   }

   /** {@inheritDoc} */
   @Override
   public EZGripperState readState(String identifier)
   {
      EZGripperState stateMessage = new EZGripperState();
      if (readState(identifier, stateMessage))
         return stateMessage;

      return null;
   }

   /** {@inheritDoc} */
   @Override
   public EZGripperCommand getCommand(String identifier)
   {
      return commandMessages.get(identifier);
   }

   /** {@inheritDoc} */
   @Override
   public boolean publishCommand(String identifier)
   {
      EZGripperCommand commandMessage = commandMessages.get(identifier);
      if (commandMessage == null)
         return false;

      commandPublisher.publish(commandMessage);
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public void start()
   {
      node.spin();
   }

   /** {@inheritDoc} */
   public void shutdown()
   {
      node.stopSpinning();

      commandPublisher.remove();
      stateSubscription.remove();

      node.destroy();
   }
}
