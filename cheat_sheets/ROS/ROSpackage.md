# ROS package cheat sheet
`catkin_lint` is a useful tool which detects possible problems between `package.xml` and `CMakeLists.txt`. Installation and usage:
```sh
sudo apt install python-catkin-lint
# Make sure the catking workspace is already sourced (. devel/setup.bash)
catkin_lint --pkg <pkg>
```

## Table of contents
* [package.xml](#package-.-xml)
* [CMakeLists.txt](#CMakeLists-.-txt)
* [roslaunch XML](#roslaunch-XML)

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
  <!--Apache 2.0, GPLv3, ...-->
  <license>BDS</license>

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
  <exec_depend>urdf</exec_depend>
  <!--If build and execution dependencies are needed use this one-->
  <depend>gazebo_ros</depend>
  <depend>gazebo_dev</depend>
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
    <gazebo_ros plugin_path="${prefix}/lib" gazebo_media_path="${prefix}" />
  </export>
</package>
```

Install system dependencies required by the ROS package in package.xml:
`rosdep install <pkg>`

### CMakeLists.txt
```cmake
# EXAMPLE
cmake_minimum_required(VERSION 2.8.3)

# Equals to package.xml name tag
project(mypkg)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS message_generation roscpp std_msgs gazebo_dev gazebo_ros
)
find_package(Boost REQUIRED
  COMPONENTS thread
)

# If the package provides some Python modules (setup.py)
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
  MySrv2.srv
)
add_action_files(DIRECTORY action FILES
  MyAc1.action
  MyAc2.action
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include # mypkg/include/*.hpp
  LIBRARIES ${PROJECT_NAME} # mypkg
  CATKIN_DEPENDS message_runtime roscpp std_msgs gazebo_dev gazebo_ros
  DEPENDS opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(myfile src/myfile.cpp)
add_library(myplugin SHARED
  src/myplugin.cpp
)
target_link_libraries(myfile myplugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# ${${PROJECT_NAME}_EXPORTED_TARGETS} using MyMsg1 etc.
# ${catkin_EXPORTED_TARGETS} using MyMsg1 etc from different pkg.
add_dependencies(myfile ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES visualize.rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/mypyfile.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mytest test/mytest.cpp)
endif()
```

### roslaunch XML
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
