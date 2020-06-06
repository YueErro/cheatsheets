# ROS 2 package cheat sheet

## Table of contents
* [package.xml](#packagexml)
* [CMakeLists.txt](#cmakeliststxt)
* [ros2 launch Python3](#ros2-launch-Python3)

### package.xml
```xml
<!--EXAMPLE-->
<!--Using recommended format-->
<package format="2">
  <name>mypkg</name>
  <!--MAJOR_API.MINOR_ABI.PATCH -->
  <version>0.0.1</version>
  <description>mypkg provides...</description>
  <author email="yue.trbj@gmail.com">Yue Erro</author>
  <maintainer email="yue.trbj@gmail.com">Yue Erro</maintainer>
  <url type="website">http://wiki.ros.org/mypkg</url>
  <url type="repository">https://github.com/YueErro/cheatsheets</url>
  <url type="bugtracker">https://github.com/YueErro/cheatsheets/issues</url>
  <!--BDS, GPLv3, ...-->
  <license>Apache License 2.0</license>
  <!--Build Tool Dependencies, build system tools which this pkg needs to build itself-->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <!--Simplifyed version of build_depend and exec_depend-->
  <depend>rclcpp</depend>
  <depend>gazebo_dev</depend>
  <depend>gazebo_ros</depend>
  <depend>geometry_msgs</depend>
  <depend>urdf</depend>
  <!--Test Dependencies, for unit tests-->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <!--Documentation Tool Dependencies, documentation tools which this pkg needs to generate documentation-->
  <doc_depend>doxygen</doc_depend>
  <!--Container for additional information for various packages and subsystems need to embed-->
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt
```cmake
# EXAMPLE
project(mypkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS} )

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_executable(myfile src/myfile.cpp)

add_library(myplugin SHARED
  src/myplugin.cpp)

ament_target_dependencies( ${PROJECT_NAME}
  "rclcpp"
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs" )

install( PROGRAMS scripts/mypyfile1.py scripts/mypyfile2.py
  DESTINATION lib/$(PROJECT_NAME) )

install( DIRECTORY meshes urdf material rviz launch
  DESTINATION share/${PROJECT_NAME} )

install( TARGETS mynode
  DESTINATION lib/${PROJECT_NAME} )

install( TARGETS myplugin1 myplugin2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install( PROGRAMS config/cfg1.yaml config/cfg2.yaml
  DESTINATION share/${PROJECT_NAME} )

if(BUILD_TESTING)
  find_package(ament_gtest)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(mytest test/mytest.cpp)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(myplugin)
ament_export_dependencies(urdf)
ament_export_dependencies(material)

ament_package()
```

### ros2 launch Python3
In `launch/<my_launch>.launch.py`:

```python
# EXAMPLE
import os

from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python import packages

def generate_launch_description():
  urdf = os.path.join(package.get_package_share_directory('mypkg'), 'urdf/file.urdf')

  try:
    envs = {}
    for key in os.environ.__dict__["_data"]:
      key = key.decode("urf-8")
      if key.isupper():
        envs[key] = os.environ[key]
  except Exception as e:
    print( "Error with Envs: " + str(e) )
    return None

  return LaunchDescription([
    ExecuteProcess(
      cmd=['gazebo', '--verbose', '-s', 'libgazeob_ros_factory.so'], output='screen', env=envs),
    Node(package='robot_state_publisher', node_executable='robot_state_publisher', arguments=[urdf], output='screen'),
    Node(package='spawnpkg', node_executable='spawn.py', arguments=[urdf], output='screen'),
```
Run the ros2 launch python3 file: `ros2 launch <pkg> <launch.pyfile>`
