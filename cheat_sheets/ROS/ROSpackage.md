# ROS package cheat sheet
`catkin_lint` is a useful tool which detects possible problems between `package.xml` and `CMakeLists.txt`. Installation and usage:
```sh
sudo apt install python-catkin-lint
# Make sure the catking workspace is already sourced (. devel/setup.bash)
catkin_lint --pkg <pkg>
```

## Table of contents
* [actionlib](#actionlib)
* [config files](#config-files)
* [include directory](#include-directory)
* [roslaunch XML](#roslaunch-XML)
* [rosmsg](#rosmsg)
* [python scripts](#python-scripts)
* [src](#src)
* [rossrv](#rossrv)
* [CMakeLists.txt](#CMakeLists-.-txt)
* [package.xml](#package-.-xml)
* [Summary structure](#Summary-structure)

### actionlib
For generating a new ROS action (action definition), create a new `action/` directory and place there the actions, `<my_action>.action`:
```cpp
// define the goal
<type1> <name1>
// ...
---
// define the result
<type2> <name2>
// ...
---
// define a feedback message
<type3> <name3>
// ...
```

### config files
All configuration files that are used in the ROS package are kept in `config/` folder as `<my_config>.yaml`.

### include directory
The corresponding C++ headers are located in `include/<my_ROSpackage>/` folder as <my_header>.h.

```cpp
#ifndef ROSPACKAGE_H
#define ROSPACKAGE_H
// ...
#endif
```

### roslaunch XML
This folder keeps the launch files that are used to launch one or more ROS nodes, `launch/<my_launch>.launch`:

```xml
<!--Example-->
<launch>
  <arg name="myarg" default="argvalue" />
  <arg name="myboolarg" default="myboolarg" />

  <!--<group unless="$(arg myboolarg)">-->
  <group if="$(arg myboolarg)">
    <include file="$(find mypkg)/launch/file.launch">
      <arg name="myotherarg" value="otherargvalue" />
    </include>
  </group>

  <!--<param name="robot_description" command="xacro '$(find mypkg)/urdf/file.urdf.xacro'" unless="$(arg myboolarg)" />-->
  <param name="robot_description" command="xacro '$(find mypkg)/urdf/file.urdf.xacro'" />
  <rosparam command="load" file="myyamelfile" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
    <remap from="robot_description" to="myrobot_robot_description" />
    <remap from="joint_states" to="myrobot_joint_states" />
  </node>
</launch>
```
Run the roslaunch file: `roslaunch <pkg> <launchfile> <argname>:=<value>`

### rosmsg
For generating a new ROS message (message definition), create a new `msg/` directory and place there the messages, `<my_msg>.msg`:
```cpp
<type1> <name1>
// ...
```

### Python scripts
Python scripts are placed in `scripts/'` as `<my_py>.py`.

### src
C++ cpp files containing the implementation are placed in `src/` as `<my_cpp>.cpp`.

### rossrv
For generating a new ROS srv (service definition), creat a new `srv/` directory and place there the services, `<my_srv>.srv`:
```cpp
// request const
<type1> <CONST_NAME2>=<value1>
// ...
// request non-const
<type1> <non_const_name1>
// ...
---
// response const
<type2> <CONST_NAME2>=<value2>
// ...
// response non-const
<type2> <non_const_name2>
// ...
```

### CMakeLists.txt
```cmake
# EXAMPLE
cmake_minimum_required(VERSION 2.8.3)

# Equals to package.xml name tag and ROS package name
project(mypkg)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS message_generation roscpp std_msgs
)
find_package(Boost REQUIRED
  COMPONENTS thread
)

# If the package provides some Python modules (scripts/setup.py)
catkin_python_setup()

set(${PROJECT_NAME}_MSGS
  MyMsg1.msg
  MyMsg2.msg
)

add_message_files(DIRECTORY msg FILES
  ${${PROJECT_NAME}_MSGS}
)
add_service_files(DIRECTORY srv FILES
  MySrv1.srv
)
add_action_files(DIRECTORY action FILES
  MyAc1.action
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include # mypkg/include/myotherpkg/*.h
  LIBRARIES ${PROJECT_NAME} # mypkg
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# usually one per main function
add_executable(myfile src/myfile.cpp)
add_library(myplugin SHARED
  src/myplugin.cpp
)
target_link_libraries(myfile myplugin ${catkin_LIBRARIES})

# ${${PROJECT_NAME}_EXPORTED_TARGETS} using MyMsg1 etc.
# ${catkin_EXPORTED_TARGETS} using MyMsg1 etc from different pkg.
add_dependencies(myfile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/mypyfile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mytest test/mytest.cpp)
endif()
```

### package.xml
```xml
<!--EXAMPLE-->
<!--Using recommended format-->
<package format="2">
  <!--Equals to project()-->
  <name>mypkg</name>
  <version>0.0.1</version>
  <description>mypkg provides...</description>
  <author email="yue.trbj@gmail.com">Yue Erro</author>
  <maintainer email="yue.trbj@gmail.com">Yue Erro</maintainer>
  <!--BDS, Apache 2.0, GPLv3, ...-->
  <license>Proprietary</license>

  <!--Build Tool Dependencies, build system tools which this pkg needs to build itself, typically only catkin-->
  <buildtool_depend>catkin</buildtool_depend>
  <!--Build Dependencies, build time pkgs to build this pkg, especially the ones in find_package()-->
  <build_depend>message_generation</build_depend>
  <!--Build Export Dependencies, pkgs needed to build libraries against this pkg, especially the catkin_package(DEPENDS)-->
  <build_export_depend></build_export_depend>
  <!--Execution Dependencies, pkgs needed to run code in this pkg, especially the catkin_package(DEPENDS)-->
  <!--Equivalent to <run_depend></run_depend> in format="1"-->
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>rospy</exec_depend>
  <!--If three of the previous dependencies are needed use this one-->
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <!--Using custom message from different pkg-->
  <depend>msgpkg</depend>
  <!--Test Dependencies, for unit tests, never duplicate mentioned as build or exec dependencies-->
  <test_depend>rosbag</test_depend>
  <!--Documentation Tool Dependencies, documentation tools which this pkg needs to generate documentation-->
  <doc_depend>doxygen</doc_depend>
  <!--Container for additional information for various packages and subsystems need to embed-->
  <export>
  </export>
</package>
```

Install system dependencies required by the ROS package in package.xml:
`rosdep install <pkg>`

### Summary structure
```
.
|__ <my_ROSpackage>
    |__ config
    |   |__ <my_config>.yaml
    |__ include
    |   |__ <my_ROSpackage>
    |       |__ <my_header>.h
    |__ msg
    |   |__ <my_msg>.msg
    |__ srv
    |   |__ <my_srv>.srv
    |__ action
    |   |__ <my_action>.action
    |__ launch
    |   |__ <my_launch>.launch
    |__ scripts
    |   |__ <my_py>.py
    |__ src
    |   |__ <my_cpp>.cpp
    |__ CMakeLists.txt
    |__ package.xml
```
