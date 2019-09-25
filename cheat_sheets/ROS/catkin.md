# catkin cheat sheet
Assuming the catkin workespace is in `~/catkin_ws`, you are located there and the corresponding ROS distribution is already sourced (. devel/setup.bash) .

## Table of contents
* [catkin_make](#catkin_make)
* [catkin_tools](#catkin_tools)

**Note:** To ignore a package, place a `CATKIN_IGNORE` file in that package.

### catkin_make
```sh
# To build any package located in ~/catkin_ws/src
catkin_make
# To build specific packages in ~/catkin_ws/src
catkin_make --pkg pkg1 pkg2
# To install target
catkin_make install
```
_Replace `catkin_make` by `catkin_make_isolated` if isolated build process wanted, wherein each package is independently configured, built, and loaded into the environment. It allows `catkin_make_isolated --merge` for build each catkin package into a common devel space._

### catkin_tools
```sh
# To build any package located in ~/catkin_ws/src
catkin build
# To build specific packages in ~/catkin_ws/src
catkin build pkg1 pkg2
# To clean specific packages in ~/catkin_ws/build and ~/catkin_ws/devel
catkin clean pkg1 pkg2
```
Installation:
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install python-catkin-tools
```
