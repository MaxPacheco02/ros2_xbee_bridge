#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import pathlib
import queue
import re
import signal
import zlib

import rclpy
import yaml
from digi.xbee.devices import DigiMeshDevice
from digi.xbee.exception import (TimeoutException, TransmitException,
                                 XBeeException)
from digi.xbee.models.message import XBeeMessage
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from serial import SerialException
from std_srvs.srv import Empty

from ros2_xbee_bridge.serialization import (deserialize_message,
                                            serialize_message)
from ros2_xbee_bridge.utilities import dict_to_msg, import_msg, msg_to_dict
from ros2_xbee_bridge.xbee_addresses import XbeeNeigbors


class RelayNode(Node):

    def __init__(self):
        super().__init__('bridge_node')

        # Get the namespace
        self.ns = self.get_namespace().strip('/')
        
        # Does the rest of the system run on ROS1 or ROS2?
        self.declare_parameter('ros_version', descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value))
        self.ros_version = self.get_parameter('ros_version').get_parameter_value().string_value
        if self.ros_version == 'ros1':
            import roslibpy
            self.bridge_client = roslibpy.Ros(host='localhost', port=9090)
            self.bridge_client.run()
         
        # Open XBee serial port.
        self.declare_parameter('dev', descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value))
        port = self.get_parameter('dev').get_parameter_value().string_value
        self.xbee = DigiMeshDevice(port, 115200)
        self.xbee.open()
        self.xbee.add_data_received_callback(self.xbee_rx_cb)
        self.xbee.add_modem_status_received_callback(self.modem_status_cb)
        
        # Define maximum message size.
        try:
            self.max_msg_size = int.from_bytes(self.xbee.get_parameter('NP'), byteorder='big')
        except (TimeoutException, XBeeException, AttributeError):
            self.max_msg_size = 256
            
        # Load a list of devices in the network and create an object to manage them.
        self.declare_parameter('config_dir', descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value))
        self.config_dir = self.get_parameter('config_dir').get_parameter_value().string_value
        self.config_dir = pathlib.Path(self.config_dir)
        self.declare_parameter('xbee_device_list_file', 
                               descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value))
        device_list_file = self.get_parameter('xbee_device_list_file').get_parameter_value().string_value
        self.neighbors = XbeeNeigbors(self.ns, self.xbee, self.config_dir / device_list_file)
        
        # Load remaining parameters.
        self.declare_parameter('incoming_message_prefix',
                               descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value))
        self.recv_message_prefix = self.get_parameter('incoming_message_prefix').get_parameter_value().string_value
        self.declare_parameter('monitoring_station_name',
                               descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value))
        self.monitoring_station_name = self.get_parameter('monitoring_station_name').get_parameter_value().string_value
        self.declare_parameter('compression_level',
                               descriptor=ParameterDescriptor(type=Parameter.Type.INTEGER.value))
        self.compression_level = self.get_parameter('compression_level').get_parameter_value().integer_value
        self.declare_parameter('max_transmission_retries',
                               descriptor=ParameterDescriptor(type=Parameter.Type.INTEGER.value))
        self.max_retries = self.get_parameter('max_transmission_retries').get_parameter_value().integer_value
        
        # Make a list of possible message destinations, i.e., all devices in the network and the broadcast address.
        self.possible_destinations = self.neighbors.namespaces.union({'broadcast'})
        
        # Message queue for outgoing messages.
        self.msg_queue = queue.PriorityQueue()
        self.tx_counter = 0

        # Define callback groups. Using callback groups ensures the proper use of resources thanks to multi-threading.
        # All callbacks assigned to a MutuallyExclusiveCallbackGroup will be executed in the same thread, but in
        # parallel to callbacks in other callback groups.
        default_cb_group = MutuallyExclusiveCallbackGroup()  # Non-essential functions - periodic checks.
        sending_cb_group = MutuallyExclusiveCallbackGroup()  # Exclussively for sending messages. 
        self.subs_cb_group = MutuallyExclusiveCallbackGroup()  # Exclusively for subscribing to topics.
        
        # Initialize structures.
        self.pubs = {}
        self.subs = {}
        self.msg_types = {}
        self.known_topics = set()
        
        # Preload topics from a file if it exists.
        self.preload_topics()
        
        # Timer for publishing messages.
        self.tx_timer = self.create_timer(0.1, self.send_timer_cb, callback_group=sending_cb_group)
        
        # Timer for checking new topics.
        self.check_topics_timer = self.create_timer(1.0, self.check_new_topics, callback_group=default_cb_group)
        
        # Timer for checking Xbee connection
        self.check_xbee_timer = self.create_timer(5.0, self.check_xbee, callback_group=default_cb_group)

        # Dummy service to report when node is ready. You can call service client's `wait_for_service` in another node. 
        self.ready_srv = self.create_service(Empty, f'{self.get_name()}/ready', self.empty_callback,
                                             callback_group=default_cb_group)

    def check_xbee(self):
        """
        Check the status of the XBee connection.

        This method attempts to get the power level of the XBee device. If the device is disconnected,
        it logs a fatal error message and exits the program. ROS launch will attempt to restart the node
        in the hope that device is reconnected.
        """
        try:
            self.xbee.get_power_level()
        except SerialException:
            self.get_logger().fatal('Xbee disconnected, exiting!')
            os.kill(os.getpid(), signal.SIGINT)
            
    def modem_status_cb(self, status):
        """
        Callback function for XBee modem status.

        Args:
            status: The modem status object.
        """
        self.get_logger().info(f"XBee modem status: {status.description}")
        
    def empty_callback(self, req, resp):
        """
        This is an empty callback method used to query the node readiness.
        """
        self.get_logger().warn(f"Service {self.get_name()}/ready should not be called.")
        return resp
    
    def preload_topics(self):
        """
        Preload topics from a configuration file and create bridges for each topic.

        The configuration file path is obtained from the 'ros1_bridge_topics_file' parameter.
        If the parameter is not set or the file cannot be opened, the method returns without creating any bridges.
        """
        self.declare_parameter('ros1_bridge_topics_file',
                               descriptor=ParameterDescriptor(type=Parameter.Type.STRING.value))
        config_file = self.get_parameter('ros1_bridge_topics_file').get_parameter_value().string_value
        if not config_file:
            return
        
        try:
            with open(self.config_dir / config_file, 'r') as f:
                bridge_topics = yaml.safe_load(f)
                bridge_topics = bridge_topics['topics']
                self.create_bridges(bridge_topics)
        except (FileNotFoundError, PermissionError):
            self.get_logger().warn("Could not open bridge config file. If not using ros1_bridge, you can ignore this.")
          
    def check_new_topics(self):
        """
        Check for new topics and create bridges for them.

        This method compares the current topics in the ROS system with the known topics,
        and creates bridges for any new topics that are detected. It updates the set of
        known topics with the newly detected topics.
        """
        current_topics_and_types = dict(self.get_topic_names_and_types())
        new_topics = set(current_topics_and_types.keys()) - self.known_topics
        if new_topics:
            self.get_logger().info(f"New topics detected: {new_topics}")
            new_topics_and_types = [{'topic': topic, 'type': current_topics_and_types[topic][0]} for topic in new_topics]
            self.create_bridges(new_topics_and_types)
            self.known_topics.update(new_topics)
            
    def create_bridges(self, topics_and_types):
        """
        Creates bridges between ROS 2 topics and XBee destinations based on the given topics and types.

        Args:
            topics_and_types (list): A list of dictionaries containing topic and type information.
        """
        for entry in topics_and_types:
            topic = entry['topic']
            dest = topic.split('/')[1]
            if dest in self.possible_destinations or dest == self.recv_message_prefix:
                # ROS 2 topic types are in the form '<package>/msg/<type>'
                # import_msg() expects only '<package>/<type>'.
                msg_description = re.sub(r'/msg', '', entry['type'])
                msg_type = import_msg(msg_description)
                self.msg_types[topic] = msg_type
                if dest in self.possible_destinations:
                    self.subs[topic] = self.create_bridge_ros2xbee(topic, msg_description, msg_type)
                    self.get_logger().info(f"Subscribed to topic {topic}")
                else:
                    self.pubs[topic] = self.create_bridge_xbee2ros(topic, msg_description, msg_type)
                    self.get_logger().info(f"Publishing to topic {topic}")
                    
                self.known_topics.add(topic)
        
    def create_bridge_ros2xbee(self, topic, msg_description, msg_type):
        """
        Create a bridge from ROS to XBee.
        
        This node subscribes to ROS messages and sends them over XBee.

        Args:
            topic (str): The topic name to subscribe to.
            msg_description (str): The description of the message. (needed for rosbridge_suite)
            msg_type (Any): The type of the message as a message object. (needed for ros2)

        Returns:
            bridge: The bridge object created.

        Raises:
            ValueError: If the ros_version parameter is not 'ros1' or 'ros2'.
        """
        if self.ros_version == 'ros1':
            bridge = roslibpy.Topic(self.bridge_client, topic, msg_description)
            bridge.subscribe(lambda x, t=topic: self.xbee_tx_cb(x, t))
        elif self.ros_version == 'ros2':
            # TODO: partial, partialmethod, closures
            # All topics have the same subscription callback, so we need to pass the topic name to the callback.
            bridge = self.create_subscription(msg_type, topic, (lambda x, t=topic: self.xbee_tx_cb(x, t)), 1,
                                              callback_group=self.subs_cb_group)
        else:
            raise ValueError("ros_version parameter must be either 'ros1' or 'ros2'.")

        return bridge
    
    def create_bridge_xbee2ros(self, topic, msg_description, msg_type):
        """
        Creates a bridge from XBee to ROS.

        Args:
            topic (str): The name of the ROS topic.
            msg_description (str): The description of the message. (needed for rosbridge_suite)
            msg_type (Any): The type of the message as a message object. (needed for ros2)

        Returns:
            bridge: The bridge object created.
            
        Raises:
            ValueError: If the ros_version parameter is not 'ros1' or 'ros2'.
        """
        
        if self.ros_version == 'ros1':
            bridge = roslibpy.Topic(self.bridge_client, topic, msg_description)
        elif self.ros_version == 'ros2':
            bridge = self.create_publisher(msg_type, topic, 1)
        else:
            raise ValueError("ros_version parameter must be either 'ros1' or 'ros2'.")
        
        return bridge
    
    def xbee_tx_cb(self, msg, topic):
        """
        Store received ROS messages in a sending queue.

        Args:
            msg: The message to be transmitted.
            topic: The topic associated with the message.
        """
        if self.ros_version == 'ros1':
            msg = dict_to_msg(self.msg_types[topic], msg)
        
        dest = topic.split('/')[1]    
        topic = '/'.join(topic.split('/')[2:])
        data = self.serialize(topic, msg)
        
        self.tx_counter += 1
        self.get_logger().info(f"Adding new message to queue>\n"
                               f"\tmessage id: {self.tx_counter}\n"
                               f"\tdestination: {dest}\n"
                               f"\ttopic: {topic}\n"
                               f"\tlength: {len(data)} bytes\n"
                               f"\thex: {data.hex()}")
        if len(data) <= self.max_msg_size:
            # If two messages have same priority, they will sent FIFO because of the tx_counter.
            self.msg_queue.put_nowait((0, self.tx_counter, data, dest))
        else:
            self.get_logger().error(f"[MSG {self.tx_counter}] Message too long to be sent over Xbee.\n")
        
    def xbee_rx_cb(self, msg: XBeeMessage):
        """
        Callback function for handling received XBee messages.

        Args:
            msg (XBeeMessage): The received XBee message.
        """
        sender_addr = msg.remote_device.get_64bit_addr().address.hex()
        sender_name = self.neighbors.get_name(sender_addr)
        topics, in_msg = self.deserialize(msg, sender_name)
        self.get_logger().info(f"Received message: {'(incomplete)' if in_msg is None else ''}\n"
                               f"\ttimestamp: {msg.timestamp}\n"
                               f"\ttopic: {topics}\n"
                               f"\tsize: {len(msg.data)} bytes\n"
                               f"\tsender: {sender_addr} ({sender_name})\n"
                               f"\tbroadcast: {msg.is_broadcast}")
        if in_msg is None:
            return
        
        # Publish received messages to ROS.
        for topic in topics:
            if topic in self.pubs:
                if self.ros_version == 'ros1':
                        in_msg = msg_to_dict(in_msg)
                self.pubs[topic].publish(in_msg)
            else:
                self.get_logger().warn(f"Publisher for this topic is not initialized.\n")
        
    def send_timer_cb(self):
        """
        Callback function for sending messages from the message queue.

        This function retrieves a message from the message queue and sends it to the appropriate destination.
        If the message fails to transmit or times out, it may be retransmitted with a decreased priority.
        Messages sent to the monitoring station are not queued for retransmission.
        """
        try:
            priority, tx_counter, data, dest = self.msg_queue.get_nowait()
            self.get_logger().info(f"Publishing message from queue:\n"
                                   f"\tmessage id: {tx_counter}\n"
                                   f"\tlength: {len(data)} bytes")
            retransmit = False
            
            try:
                # Messages sent to every device in the network. Success is not checked.
                if dest == 'broadcast':
                    self.xbee.send_data_broadcast(data)
                # Messages sent to the monitoring (ground) station. Such messages are ususally status messages that are 
                # sent periodically and it's not a big deal if they don't arrive. We are not waiting for a confirmation.
                elif dest == self.monitoring_station_name:
                    self.xbee.send_data_async(self.neighbors[dest], data)
                # Messages sent directly to other devices in the network are important. We are waiting for a 
                # confirmation that have been received. If the transmission fails, re-add the message to the queue.
                else:
                    self.xbee.send_data(self.neighbors[dest], data)
            except TransmitException as e:
                self.get_logger().warn(f"[MSG {tx_counter}] Transmit error: {e}")
                retransmit = True
            except TimeoutException as e:
                self.get_logger().warn(f"[MSG {tx_counter}] No confirmation from remote device: {e}")
                retransmit = True
                
            if dest == self.monitoring_station_name:
                retransmit = False
            
            if retransmit:
                # Add failed messages back to the queue, but with lower priority.
                new_priority = priority + 1
                if new_priority > self.max_retries:
                    self.get_logger().error(f"[MSG {tx_counter}] Maximum retries reached.")
                else:
                    self.get_logger().info(f"Re-adding new message to queue>\n"
                                        f"\tmessage id: {tx_counter}\n"
                                        f"\tretry: {new_priority}\n"
                                        f"\tdestination: {dest}\n"
                                        f"\tlength: {len(data)} bytes\n"
                                        f"\thex: {data.hex()}")
                    self.msg_queue.put_nowait((new_priority, tx_counter, data, dest))
        except queue.Empty:
            pass
    
    def serialize(self, topic, msg):
        """
        Serializes the given message and topic into a compressed data format.

        Args:
            topic (str): The topic associated with the message.
            msg (Any): The message to be serialized.

        Returns:
            bytes: The serialized and compressed data.
        """
        serialized = serialize_message(msg)
        data = bytes([len(topic)]) + topic.encode('ascii') + bytes([len(serialized)]) + serialized
        if self.compression_level != 0:
            try:
                compressed = zlib.compress(data, level=self.compression_level)
            except zlib.error:
                compressed = data
                self.get_logger().warn("Zlib compression failed. Using uncompressed data.")
        else:
            compressed = data

        return compressed
    
    def deserialize(self, msg, sender):
        """
        Deserialize a message received from the sender.

        Args:
            msg (Any): The message data to be deserialized.
            sender (str): The sender of the message.

        Returns:
            Tuple[List[str], Any]: A tuple containing a list of topics and the deserialized message.
        """
        data = msg.data
        if self.compression_level != 0:
            try:
                decompressed = zlib.decompress(data)
            except zlib.error:
                self.get_logger().warn(f"Zlib decompression failed. Trying raw message.")
                decompressed = data
        else:
            decompressed = data
            
        topic_len = decompressed[0]
        try:
            topic = decompressed[1:topic_len+1].decode('ascii')
        except UnicodeDecodeError:
            self.get_logger().error("Message topic cannot be decoded.")
            return [None], None
        
        topics = []
        if f'/{self.recv_message_prefix}/{sender}/{topic}' in self.msg_types:
            topics.append(f'/{self.recv_message_prefix}/{sender}/{topic}')
        if f'/{self.recv_message_prefix}/{topic}' in self.msg_types:
            topics.append(f'/{self.recv_message_prefix}/{topic}')
        
        if not topics:
            self.get_logger().warn(f"Received message with unknown topic: {topic}")
            return [None], None
        
        if len(topics) == 2 and self.msg_types[topics[0]] != self.msg_types[topics[1]]:
            self.get_logger().warn(f"There are subscribers for *both* unicast and broadcast of topic {topic}, but they"
                                   f"have different types {self.msg_types[topics[0]]} != {self.msg_types[topics[1]]}. "
                                   f"Publishing only to unicast subscribers.")  # The logic behind publishing only to
                                                                                # unicast is that they are probably
                                                                                # subscribing with a more specific use 
                                                                                # case.
        
        expected_data_len = decompressed[topic_len+1]
        real_data_len = len(decompressed[topic_len+2:])
        if real_data_len != expected_data_len:
            self.get_logger().warn(f"Message length mismatch." 
                                   f"Expected {expected_data_len} bytes, got {real_data_len} bytes.")
        
        try:
            msg = deserialize_message(decompressed[topic_len+2:], self.msg_types[topics[0]])
        except Exception as e:
            self.get_logger().error(f"Message deserialization failed: {e}")
            self.get_logger().error(f'{expected_data_len=}, {real_data_len=}\n{decompressed[topic_len+2:].hex()}')
            return topics, None
        return topics, msg


def main(args=None):
    rclpy.init(args=args)

    relay = RelayNode()    
    
    executor = MultiThreadedExecutor()
    executor.add_node(relay)

    try:
        relay.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        relay.get_logger().info('Keyboard interrupt, shutting down.\n')
    relay.xbee.close()
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
