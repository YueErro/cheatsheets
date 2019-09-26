# ros cheat sheet
Assuming the corresponding ROS environment is already sourced (. devel/setup.bash).

## Table of contents
* [Filesystem management](#Filesystem-management)
* [Start-up and process launch](#Start---up-and-procees-launch)
* [Running system](#Running-system)
  * [rosnode](#rosnode)
  * [rostopic](#rostopic)
  * [rosservice](#rosservice)
  * [rosparam](#rosparam)
  * [rosmsg and rossrv](#rosmsg-and-rossrv)
  * [rosbag](#rosbag)
  * [rqt](#rqt)
* [URDF and xacro](#URDF-and-xacro)

### Filesystem management
```sh
# Go to the directory
roscd <pkg>
# List content
rosls <pkg>
# Edit the file with vim or specify EDITR=<editor> before
rosed <pkg> <file>
# Copy a file to the current directory
roscp <pkg> <file> .
# Search for a package listed in ROS_PACKAGE_PATH
rospack find <pkg>
# Check potential issues in the the specific .launch file or in the current pkg if no file is specified
roswtf <file>
# Install all the pkgs that the packages in the workspace depend upon but are missing in the PC
rosdep install --from-paths src --ignore-src -r -y
```

### Start-up and process launch
```sh
# If you specify a port, -p <port>, export ROS_MASTER_URI=http://lohalhost:<port>
roscore
# Run an executable
rosrun <pkg> <executable> <parameter>:=<value>
# Launch nodes form launch file and it starts a roscore if needed
roslaunch <pkg> <launchfile>
```

### Running system
#### rosnode
```sh
# List active nodes
rosnode list
# Test connectivity, use --all to ping all
rosnode ping
# Get information about the specific node
rosnode info <node>
# Kill the specific node, use -a to kill all
rosnode kill <node>
```

#### rostopic
```sh
# List currently published or subscribed topics, use -v to list publishers and subscribers as well
rostopic list
# Get information about the specific topic
rostopic info <topic>
# Display messages published to the specific topic
rostopic echo <topic>
# Get puvlishing rate of the specific topic
rostopic hz <topic>
# Get type of the specific topic
rostopic type <topic>
# Find topic by type
rostopic find <type>
# Publish data to a specific topic, use -1 to publish only once
rostopic pub -1 <topic> <msgtype> <args>
```

#### rosservice
```sh
# List active services
rosservice list
# Get information about the specific service
rosservice info <service>
# Get the name of the node that provides the specific service
rosservice node <service>
# Get type of the specific service
rosservice type <service>
# Find service by type
rosservice find <type>
# List arguments of the specific service
rosservice args <service>
# Call specific service request
rosservice call <service> <args>
```

#### rosparam
```sh
# List parameter names
rosparam list
# Get the specific parameter
rosparam get <param>
# Set the specific parameter to the specific value
rosparam set <param> <value>
# Delete the specific parameter
rosparam delete <param>
```

#### rosmsg and rossrv
```sh
# List of all messages types
rosmsg list
# Get the fields of the specific message type
rosmsg show <type>
# List all the packages containing the specific message type
rosmsg packages <type>
```
_The commands above can be apply to ROS service as well. Just replace `rosmsg` by `rossrv`._

#### rosbag
```sh
# Record all messages from specific topics into specific (-0) file, use -l <val> to set a maximum messages to be recorded from each topics
rosbag record -0 <file> <topic1> <topic2>
# Replay all the messages in the specific .bag files, use -r <value> to change the rate
rosbag play <file1> <file2>
# Get information about the specific .bag file
rosbag info <file>
# Compress the specific .bag file using BZ2
rosbag compress <file>
# Decompress the specific .bag file
rosbag decompress <file>
```

#### rqt
```sh
# GUI tool to choose any available plugin
rqt
# GUI for displaying and filtering ROS messages
rqt_console
# GUI to visualize, inspect and replay the specific .bag file
rqt_bag <file>
# Provides a GUI for vizualizing the ROS computation graph
rqt_graph

```

### URDF and xacro
```sh
# Generate .from the specific .urdf.xacro file the corresponding .urdf file
xacro <inputurdfxacrofile> > <outputurdffile>
# Get tree from the specific .urdf file
check_urdf <file>
# Generate pdf with the graph generated from the specific .urdf file
urdf_to_graphiz <file>
```
