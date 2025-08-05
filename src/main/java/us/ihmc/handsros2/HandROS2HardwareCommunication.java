package us.ihmc.handsros2;

import us.ihmc.communication.packets.Packet;

import java.util.List;
import java.util.Set;

/**
 * <p>Interface for high level ROS 2 communication for hands. Communicates with low-level hardware control process.</p>
 * <p>Subscribes to state messages and publishes command messages.</p>
 *
 * @param <Cmd> The command message type.
 * @param <Stt> The state message type.
 */
public interface HandROS2HardwareCommunication<Cmd extends Packet<Cmd>, Stt extends Packet<Stt>>
{
   /**
    * <p>Get the identifiers of the available hands.</p>
    * <p>Treat the set as read-only.</p>
    *
    * @return Set of identifiers of the available hands.
    */
   Set<String> getAvailableHands();

   /**
    * <p>Get a synchronized list of the identifiers of the available hands.</p>
    * <p>The list should be created using {@link java.util.Collections#synchronizedList(List)},
    * thus a synchronized block must be used when iterating over the list/</p>
    * <p>Tread the list as read-only.</p>
    *
    * @return List of identifiers of the available hands.
    */
   List<String> getAvailableHandList();

   /**
    * Read the latest state message of the specified hand.
    *
    * @param identifier  Identifier specifying the hand.
    * @param stateToPack State message to pack with the latest state.
    * @return {@code true} if a state message was available. {@code false} if no state had been received.
    */
   boolean readState(String identifier, Stt stateToPack);

   /**
    * Read the latest state message of the specified hand.
    *
    * @param identifier Identifier specifying the hand.
    * @return A copy of the latest state message.
    */
   Stt readState(String identifier);

   /**
    * <p>Get the command message for the specified hand.</p>
    * <p>Use this method to set the desired command values.
    * Then publish the command using {@link #publishCommand(String)}.</p>
    *
    * @param identifier Identifier specifying the hand.
    * @return A reference to the command message for the specified hand.
    */
   Cmd getCommand(String identifier);

   /**
    * Publish the command for the specified hand.
    *
    * @param identifier Serial number specifying the hand.
    * @return {@code true} if the message was published. {@code false} if the hand specified wasn't found.
    */
   boolean publishCommand(String identifier);

   /**
    * Start the communication.
    */
   void start();

   /**
    * Shut the communication down. {@link #start()} cannot be called again after this method.
    */
   void shutdown();
}
