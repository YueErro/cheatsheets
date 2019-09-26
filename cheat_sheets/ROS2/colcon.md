# colcon cheat sheet
```sh
sudo apt install python3-colcon-common-extensions
```

Assuming the ROS 2 workespace is in `~/ros2_ws`, you are located there and the corresponding ROS 2 distribution is already sourced (. /opt/ros/<distro>/setup.bash).

**Note:** To ignore a package, place a `COLCON_IGNORE` file in that package.

```sh
# To build any package located in ~/ros2_ws/src
colcon build
# To build specific packages in ~/ros2_ws/src
colcon build --packages-select <pkg1> <pkg2>
# To build all except specific packages in ~/ros2_ws/src
colcon build --packages-skip <pkg1> <pkg2>
# To merge all install prefixed into a single location
catkin_make --merge-install
```
_Use `--packages-select-regex` or `--package-skip-regex` instead of their corresponding ones for apply to all packages that have that pkg1 pattern in the package name_
