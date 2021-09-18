# ROS package cheat sheet
`catkin_lint` is a useful tool which detects possible problems between `package.xml` and `CMakeLists.txt`. Installation and usage:
```sh
sudo apt-get install python-catkin-lint
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
* [tests](#tests)
  * [gtest](#gtest)
  * [unittest](#unittest)
* [CMakeLists.txt](#cmakeliststxt)
* [package.xml](#packagexml)
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
Python scripts are placed in `scripts/` as `<my_py>.py`.

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

### tests

#### gtest
C++ tests for ROS packages involve Google's gtest framework (without rostest) and will be located at `test/` folder as `<my_test>.cpp`:
```cpp
#include "<my_ROSpackage>/<my_header>.h"

#include <gtest/gtest.h>

TEST(<MyTestGroup>, <myTestName>){
  // ASSERT_* and/or EXPECT_* calls been first parameter commonly expected value
}

// more TESTs ...

int main(int argc, char **argv){
  ros::init(argc, argv, "<my_test>");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

```

ASSERT_* aborts executions while EXPECT_* don't.

#### unittest
Python tests for ROS packages are based on Python unittest framework and will be placed at `test/` folder as `<my_test>.py`:

```py
#!/usr/bin/env python

PKG = 'my_ROSpackage'
NAME = 'test_name'

import sys
import rosunit
import unittest

class TestName(unittest.TestCase):
  def test_name(self): // only functions with 'test_' prefix will be run
    // self.assert*

if __name__ == '__main__':
  rostest.rosrun(PKG, NAME, TestName, sys.argv)
```

`rosrun rosunit rosunit --name <test_name>`

### CMakeLists.txt
```cmake
# EXAMPLE
cmake_minimum_required(VERSION 2.8.3)

# Equals to package.xml name tag and ROS package name
project(mypkg)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# pkgs that you have #include in your .h and they have to be added as <build_depend>
# setting as components will be added for example to catkin_INCLUDE_DIRS and catkin_LIBRARIES -> linked to include directories()
find_package(catkin REQUIRED actionlib_msgs COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
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

# always after add_message_files(), add_service_files() and add_action_files() and before catkin_package() and there, CATKIN_DEPENS need message_runtime and find_package a catkin REQUIRED COMPONENT message_generation and possible add_dependencies() to catkin_EXPORTED_TARGETS
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

# CATKIN_DEPENDS pkgs have to be added as <build_export_depend>, they are #include in a header that others can use. Add because they have to be present to build/run this pkg
# DEPENDS same as CATKIN_DEPENDS but for non catkin pkgs
# Alwas before add_library() or add_executable()
catkin_package(
  INCLUDE_DIRS include # the folder "include" within the package folder is where exported headers go
  LIBRARIES ${PROJECT_NAME} # mypkg, libraries at catkin_LIBRARIES and ${PROJECT_NAME}_LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS Boost
)

# *_INCLUDE_DIRS variables generated by find_package() call
include_directories(
  include # include/ directory within the pkg
  ${catkin_INCLUDE_DIRS}
)

# by default is SHARED
add_library(myplugin SHARED
  src/myplugin.cpp
)
# usually one per main function
add_executable(myfile src/myfile.cpp)
# link the executable against the library, typically after an add_executable()
target_link_libraries(myfile myplugin ${catkin_LIBRARIES})

# ${${PROJECT_NAME}_EXPORTED_TARGETS} using MyMsg1 etc.
# ${catkin_EXPORTED_TARGETS} using MyMsg1 etc from different pkg.
add_dependencies(myfile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# installing target to the system instead of placing them into devel folder of the ws, it is also common to use ${CATKIN_GLOBAL_BIN_DESTINATION}, ${CATKIN_PACKAGE_PYTHON_DESTINATION}
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# installing header files
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# installing roslaunch files
install(DIRECTORY launch DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

# installing python executable scripts
catkin_install_python(PROGRAMS
  scripts/mypyfile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test test/my_test.cpp)
endif()
if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

catkin_add_nosetests(test/my_test.py)
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
  <url type="website">http://wiki.ros.org/mypkg</url>
  <url type="repository">https://github.com/YueErro/cheatsheets</url>
  <url type="bugtracker">https://github.com/YueErro/cheatsheets/issues</url>
  <!--BDS, Apache 2.0, GPLv3, ...-->
  <license>Proprietary</license>
  <!--Build Tool Dependencies, build system tools which this pkg needs to build itself, typically only catkin-->
  <buildtool_depend>catkin</buildtool_depend>
  <!--Build Dependencies, build time pkgs to build this pkg, especially the ones in find_package()-->
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <!--Execution Dependencies, pkgs needed to run code in this pkg, especially the catkin_package(CATKIN_DEPENDS)-->
  <!--Equivalent to <run_depend></run_depend> in format="1"-->
  <exec_depend>rospy</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <!--Build Export Dependencies, pkgs needed to build libraries against this pkg, especially the catkin_package(CATKIN_DEPENDS)-->
  <build_export_depend></build_export_depend>
  <!--If three of the previous dependencies are needed use this one-->
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <!--Using custom message from different pkg-->
  <depend>msgpkg</depend>
  <!--Test Dependencies, for unit tests, never duplicate mentioned as build or exec dependencies-->
  <test_depend>rosbag</test_depend>
  <test_depend>rosunit</test_depend>
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
    |__ action
    |   |__ <my_action>.action
    |__ config
    |   |__ <my_config>.yaml
    |__ include
    |   |__ <my_ROSpackage>
    |       |__ <my_header>.h
    |__ launch
    |   |__ <my_launch>.launch
    |__ msg
    |   |__ <my_msg>.msg
    |__ scripts
    |   |__ <my_py>.py
    |__ src
    |   |__ <my_cpp>.cpp
    |__ srv
    |   |__ <my_srv>.srv
    |__ test
    |   |__ <my_test>.cpp
    |   |__ <my_test>.py
    |__ CMakeLists.txt
    |__ package.xml
```
