# ROS cheat sheet
Assuming the corresponding ROS environment is already sourced (. devel/setup.bash).

## Table of contents
* [Filesystem management](#Filesystem-management)
* [Start-up and process launch](#Start---up-and-process-launch)
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
# Run an executable from a ROS package
rosrun <pkg> <executable> <parameter>:=<value>
# Launch nodes form launch file and it starts a roscore if needed
roslaunch <pkg> <launchfile>
```

### Running system
#### rosnode
```sh
# List active ROS nodes
rosnode list
# Test connectivity, use --all to ping all
rosnode ping
# Get information about the specific ROS node
rosnode info <node>
# Kill the specific ROS node, use -a to kill all
rosnode kill <node>
```

#### rostopic
```sh
# List currently published or subscribed ROS topics, use -v to list publishers and subscribers as well
rostopic list
# Get information about the specific ROS topic
rostopic info <topic>
# Display messages published to the specific ROS topic
rostopic echo <topic>
# Get publishing rate of the specific ROS topic
rostopic hz <topic>
# Get type of the specific ROS topic
rostopic type <topic>
# Find ROS topic by type
rostopic find <type>
# Publish data to a specific ROS topic, use -1 to publish only once
rostopic pub -1 <topic> <msgtype> <args>
```

#### rosservice
```sh
# List active ROS services
rosservice list
# Get information about the specific ROS service
rosservice info <service>
# Get the name of the node that provides the specific ROS service
rosservice node <service>
# Get type of the specific ROS service
rosservice type <service>
# Output ROS services of the given type
rosservice find <type>
# List arguments of the specific ROS service
rosservice args <service>
# Call specific ROS service request
rosservice call <service> <args>
```

#### rosparam
```sh
# List ROS parameter names
rosparam list
# Get the specific ROS parameter
rosparam get <param>
# Set the specific ROS parameter to the specific value
rosparam set <param> <value>
# Delete the specific ROS parameter
rosparam delete <param>
```

#### rosmsg and rossrv
```sh
# List ROS messages types
rosmsg list
# Get the fields of the specific ROS message type
rosmsg show <type>
# List all the packages containing the specific ROS message type
rosmsg packages <type>
```
_The commands above can be apply to ROS service as well. Just replace `rosmsg` by `rossrv`._

#### rosbag
```sh
# Record all ROS messages from specific ROS topics into specific (-0) file, use -l <val> to set a maximum messages to be recorded from each topics
rosbag record -0 <file> <topic1> <topic2>
# Replay all the ROS messages in the specific ROS .bag files, use -r <value> to change the rate
rosbag play <file1> <file2>
# Get information about the specific ROS .bag file
rosbag info <file>
# Compress the specific ROS .bag file using BZ2
rosbag compress <file>
# Decompress the specific ROS .bag file
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
