# ROS 2 package for communicating over XBee devices

This package contains the code to bridge the communication between the XBee and the ROS network.

## Features
- **Automatic topic discovery**: The bridge node scans the ROS network for all topics and their types every second. That way it is not necessary to specify them upfront in a configuration file.
- **Unicast and broadcast support**: You can publish messages to a specific robot or to all robots at once.
- **Automatic message serialization and deserialization**: The bridge node will automatically serialize and compress the message before sending it over the Xbee network and deserialize it on the receiving end.
- **Message queue**: The bridge node uses a message queue to prevent messages from being lost if the Xbee device is busy sending another message.
- **ROS 1 and ROS 2 support**: The bridge node can run on both ROS 1 and ROS 2 systems.

## Environment
The package primarily targets ROS 2, but can also work on ROS 1 systems with the help of Docker and [rosbridge_suite](http://wiki.ros.org/rosbridge_suite) or [ros1_bridge](https://github.com/ros2/ros1_bridge).

### Supported devices
Any Digi XBee device that supports _DigiMesh_ wireless mesh networking topology should be supported. The list of devices includes:
- Digi XBee 3 2.4 RF Modules
- Digi XBee SX 868 RF Modules
- Digi XBee SX 900 RF Modules
- Digi XBee-PRO 900HP RF Modules
- Digi XBee XR 868 Modules  
The library was tested with SX 868 RF module for long-range intra-robot communication.


## How it works?
1. The XBee bridge node scans the ROS network for all topics and their types every second. That way it is not necessary to specify them upfront in a configuration file.
1. If it finds a topic that includes the namespace of one of the robots in the system, it will create a subscriber to that topic. Your node can publish to that topic and the message will be sent to the corresponding robot over the XBee network.
1. To differentiate topics coming from other robots in the XBee network from the topics in the local namespace, all external topics are prefixed with a magic keyword defined by `incoming_message_prefix` ("intra_comms" by default). If the XBee bridge finds a topic that includes the magic keyword, it will create a publisher for that topic. Your node can subscribe to that topic and the message will be received when it arrives over the Xbee network.
1. All other topics used for local intra-node communication are ignored.
1. **IMORTANT NOTE**: Topics are scanned every second so there may be delays between discovering a topic and creating a subscriber/publisher. This can lead to messages being lost. To avoid this, all publishers and subscribers should be created when spawning the node and nodes should wait a few seconds before starting to publish.
1. There are 3 ways to send a message:
    - **Unicast**: Send a message to the specific robot using unicast mesh transmission. This message will possibly hop over several devices in the network to reach the destination. Each transmission is acknowledged by the receiving device. If the transmission fails, the message is re-added to the queue and sent again up to `max_transmission_retries` times.
    - **Broadcast**: Send a message to all robots. There are no acknowledgements for broadcast messages.
    - **Unicast to monitoring station**: Some messages may be status messages that are sent out periodically and are not essential to the operation of the system. These messages are usually sent to some kind of a monitoring station which may or may not have an active role in the system. To prevent overloading the network in case the monitoring station is not reachable, these messages are sent using asynchronous unicast transmissions without waiting for an acknowledgement and are not re-added to the queue if the transmission fails. The name of the monitoring station is defined by `monitoring_station_name` ("ground" by default).

### TL; DR:
Run the XBee bridge node.  

If you want to **publish a message**...
- **...to the specific robot**: publish to `/<target_namespace>/<topic>`
- **...to all robots**: publish to `/broadcast/<topic>`

If you want to **listen for messages**...
- **...from the specific robot**: subscribe to `/intra_comms/<source_namespace>/<topic>` (This will ignore all unicasts and broadcasts not coming from the specified robot.)
- **...from all robots**: subscribe to `/intra_comms/<topic>`
- **Note**: You can subscribe to both unicast and broadcast type topics at the same time, but they must be the same type.


## Installation & setup
### Manual setup
1. [Configure your XBee modules](https://xbplib.readthedocs.io/en/latest/getting_started_with_xbee_python_library.html#configure-your-xbee-modules)
1. Install the prerequisites:
    - `python3 -m pip install pyyaml digi-xbee zlib`
1. Clone the repository into your workspace and build it:
    - `cd <your_ws>/src`
    - `git clone git@github.com:mkrizmancic/ros2_xbee_bridge.git`
    - `cd <your_ws> && colcon build` (or whatever other approach you use to build your workspace)
1. Specify the names and addresses of the devices in the system in [config/xbee_device_list.yaml](config/xbee_device_list.yaml).
1. Optionally change the default parameters in [config/ros_params.yaml](config/ros_params.yaml).

### Docker setup
This repository comes with a Dockerfile and easy-to-use scripts to set up a containerized environment with ROS 2 Humble and `ros2_xbee_bridge` package.

1. Install and set up Docker by following [official instructions]()
1. Go to the docker directory: `cd <>/ros2_xbee_bridge/docker`
1. Run: `./build_docker.sh`
1. Run: `./run_docker.sh`
1. In the future, to start the container, run `./start_docker.sh`


## Working with ROS 1
There are two ways this package can be used with ROS 1.
### 1. rosbridge_suite (recommended)
Simpler to set up, but limited. It includes a websocket server that allows communication between ROS 1 and ROS 2 by serializing messages with JSON. The pure Python `roslibpy` client will run in the XBee bridge node when `ros_version` parameter is set to `ros1`. See more details [here](http://wiki.ros.org/rosbridge_suite) and [here](https://roslibpy.readthedocs.io/en/latest/).

To use this approach, additionally install
- `sudo apt install ros-<distro>-rosbridge-server`
- `python3 -m pip install roslibpy`

To run the XBee bridge node with ROS 1, set the `ros_version` parameter to `ros1`, and 
- run `roslaunch rosbridge_server rosbridge_websocket.launch` on your ROS 1 system.

### 2. ros1_bridge
More complex to set up, but more powerful as it allows higher bandwidth. However, since XBee communication has a relatively low bandwidth, `rosbridge_suite` should be sufficient in all use cases. Use this method only if you already require `ros1_bridge` for other purposes. To install, set up and learn more, please visit the official repository [here](https://github.com/ros2/ros1_bridge).

**IMPORTANT NOTE**:   
`ros1_bridge` will not automatically create the appropriate ROS 2 subscriber if you create a ROS 1 subscriber. The ROS 2 subscriber is created only when there is a ROS 2 publisher publishing to the given topic. In other words, initializing ROS 2 -> ROS 1 bridge happens only on the ROS 2 side. Therefore, ROS 2 side is not aware that there are ROS 1 subscribers and the automatic topic discovery fails. 

One workaround is to create a "dummy" publisher on ROS 1 side with the same topic to which you want to subscribe, e.g. `/intra_comms/my_topic`. This will trigger the creation of approapiate brigdes.

Alternatively, use the [parameter_bridge](https://github.com/ros2/ros1_bridge?tab=readme-ov-file#example-4-bridge-only-selected-topics-and-services) instead of the `dynamic_bridge` to manually specify the topics to bridge. You can give the path to the bridge configuration file as a parameter `ros1_bridge_topics_file` to the XBee bridge node. The node will then preload all the specified topics according to the rules explained above. This option does not disable the automatic discovery of remaining topics.

## Usage
### ROS 2
1. Launch the XBee bridge node along with the rest of your system:
   - `ros2 launch mbzirc_xbee_comms xbee_bridge.launch.py namespace:=<your_namespace>`

### ROS 1
1. Launch the rosbridge_suite server node with the rest of your ROS 1 system:
   - `roslaunch rosbridge_server rosbridge_websocket.launch`  

   OR  
   Launch the ros1_bridge with the rest of your ROS 1 system according to the [instructions](https://github.com/ros2/ros1_bridge?tab=readme-ov-file#example-4-bridge-only-selected-topics-and-services)

1. Inside the docker container, launch the XBee bridge node
   - `ros2 launch mbzirc_xbee_comms xbee_bridge.launch.py ros_version:='ros1' namespace:=<your_namespace>`

### Testing
To test that everything works, you can use the provided tmux session with a few preconfigured windows and panes. The session will launch two instances of the XBee bridge node, and start two publishers and subscribers in the terminal using ROS CLI tools. Read the Bonus section to learn more about the tmux session setup and how to navigate within it. The session sets up two separate `ROS_DOMAIN_ID` environment variables to simulate two separate ROS networks. This way you can be sure that communication is established over the XBee network.

Steps for testing:
1. Prepare two XBee devices and set up their parameters using XCTU program.
1. Write down their MAC addresses in the [config/xbee_device_list.yaml](config/xbee_device_list.yaml) file.
1. First plug in the device corresponding to the `xbee0` namespace and then the device corresponding to the `xbee1` namespace.
1. Navigate to the `startup` directory and run the tmux session. (tmux and tmuxinator are installed by default in the Docker container. If you are running the code on your computer, make sure to install them first.):
   - `cd <your_ws>/src/ros2_xbee_bridge/startup`
   - `./start_tmux.sh`


## Bonus section - working with Docker
The provided Docker image comes with a few preinstalled tools and configs which may simplify your life.

**Tmuxinator** is a tool that allows you to start a tmux session with a complex layout and automatically run commands by configuring a simple yaml configuration file. Tmux is a terminal multiplexer - it can run multiple terminal windows inside a single window. This approach is simpler than having to do `docker exec` every time you need a new terminal.

You can move between terminal panes by holding down `Ctrl` key and navigating with arrow keys. Switching between tabs is done with `Shift` and arrow keys. If you have a lot of open panes and tabs in your tmux, you can simply kill everything and exit by pressing `Ctrl+b` and then `k`.

Here are some links: [Tmuxinator](https://github.com/tmuxinator/tmuxinator), [Getting starded with Tmux](https://linuxize.com/post/getting-started-with-tmux/), [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)

**Ranger** is a command-line file browser for Linux. While inside the Docker container, you can run the default file browser `nautilus` with a graphical interface, but it is often easier and quicker to view the files directly in the terminal window. You can start ranger with the command `ra`. Moving up and down the folders is done with arrow keys and you can exit with a `q`. When you exit, the working directory in your terminal will be set to the last directory you opened while in Ranger.

**Htop** is a better version of `top` - command line interface task manager. Start it with the command `htop` and exit with `q`.
