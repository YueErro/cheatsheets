# ROS 2 cheat sheet
Assuming the corresponding ROS 2 environment is already sourced (. install/setup.bash).

## Table of contents
* [Filesystem management](#Filesystem-management)
  * [ros2 pkg](#ros2-pkg)
* [Process launch](Process-launch)
* [Running system](#Running-system)
  * [ros2 node](#ros2-node)
  * [ros2 topic](#ros2-topic)
  * [ros2 service](#ros2-service)
  * [ros2 action](#ros2-action)
  * [ros2 param](#ros2-param)
  * [ros2 msg and ros2 srv](#ros2-msg-and-ros2-srv)
  * [ros2 bag](#ros2-bag)
* [URDF and xacro](#URDF-and-xacro)

### Filesystem management
#### ros2 pkg
```sh
# List available ROS 2 packages
ros2 pkg list
# Create ROS 2 package, use --help to know additional arguments. Make sure you are in ~/ros2_ws/src
ros2 pkg create <pkg>
```

### Process-launch
```sh
# Run an executable from a ROS 2 package
ros2 run <pkg> <executable>
# Start rqt console in a new terminal
ros2 run rqt_console rqt_console
# Launch nodes from a Python3 launch file
ros2 launch <pkg> <launch.pyfile>
```

### Running system
#### ros2 node
```sh
# List active ROS 2 nodes
ros2 node list
# Get information about the specific ROS 2 node
ros2 node info <node>
```

#### ros2 topic
```sh
# List all the ROS 2 topics currently active, use -t to get the type of each topic
ros2 topic list
# Get information about the specific ROS 2 topic
ros2 topic info <topic>
# See the data being published on the specific ROS 2 topic
ros2 topic echo <topic>
# Get publishing rate of the specific ROS 2 topic
ros2 topic hz <topic>
# Display delay of the specific ROS 2 topic from timestamp in header
ros2 topic delay <topic>
# Publish data to a specific ROS 2 topic, use -1 or --once to publish just once
ros2 topic pub -1 <topic> <msgtype> '<args>'
```

#### ros2 service
```sh
# List active ROS 2 services, use -t to get the types as well
ros2 service list
# Get the type of the specific ROS 2 service
ros2 service type <service>
# List ROS 2 services of the given type
ros2 service find <type>
# Call specific ROS 2 service request
ros2 service call <service> <srvtype> <args>
```

#### ros2 action
```sh
# List ROS 2 actions, use -t to get the types as well
ros2 action list
# Get more information about the ROS 2 action
ros2 action info <action>
# Send a goal to a specific ROS 2 action
ros2 action send_goal <action> <actype> <value>
# Get the structure of the goal request, the result and the feedback
ros2 action show <actionfile>
```

#### ros2 param
```sh
# List ROS 2 parameter names
ros2 param list
# Get the current value of the ROS 2 parameter
ros2 param get <node> <param>
# Change the value of the ROS 2 parameter at runtime
ros2 param set <node> <param>
# Delete the specific ROS 2 parameter
ros2 param delete <node> <param>
# Save all current parameter values of a node in a file
ros2 param dump <node>
```

#### ros2 msg and ros2 srv
```sh
# List ROS 2 messages types
ros2 msg list
# List ROS 2 messages type within the given ROS2 package
ros2 msg package <pkg>
# List ROS 2 packages which contain the given type
ros2 msg packages <msgtype>
# Get the fields of the specific ROS 2 message type
ros2 msg show
```
_The commands above can be apply to ROS 2 service as well. Just replace `msg` by `srv`_

#### ros2 bag
```sh
# Record all ROS 2 topics
ros2 bag record -a
# Replay recorded bag file
ros2 bag play <bagfile>
# Print information of the specific bag file
ros2 bag info <bagfile>
```

### URDF and xacro
:information_source: Till xacro is ported to ROS 2:
```sh
cd ~/ros2_ws/src
git clone -b ros2-devel https://github.com/bponsler/xacro.git
colcon build --packages-select xacro --cmake-args -DBUILD_TESTING=0
xacro --inorder <xacropath> -o <urdfpath>
```
