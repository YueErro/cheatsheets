# colcon cheat sheet
```sh
sudo apt-get install python3-colcon-common-extensions
```

Assuming the corresponding ROS 2 distribution is already sourced (`. /opt/ros/<distro>/setup.bash`).

## Table of contents
- [colcon cheat sheet](#colcon-cheat-sheet)
  - [Table of contents](#table-of-contents)
    - [Create a workspace for colcon](#create-a-workspace-for-colcon)
      - [colcon build](#colcon-build)

### Create a workspace for colcon
```sh
mkdir -p  ~/ros2_ws/src
cd ~/ros2_ws
```

**Note:** To ignore a package, place a `COLCON_IGNORE` file in that package.

#### colcon build
```sh
# To build any package located in ~/ros2_ws/src
colcon build
# To build specific packages in ~/ros2_ws/src
colcon build --packages-select <pkg1> <pkg2>
# To build all except specific packages in ~/ros2_ws/src
colcon build --packages-skip <pkg1> <pkg2>
# To merge all install prefixed into a single location
colcon build --merge-install
# Avoid configuring and building tests
colcon build --cmake-args -DBUILD_TESTING=0
```
_Use `--packages-select-regex` or `--package-skip-regex` instead of their corresponding ones for apply to all packages that have that pkg1 pattern in the package name_
