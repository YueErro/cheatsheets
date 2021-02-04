# catkin cheat sheet
Assuming the corresponding ROS distribution is already sourced (. /opt/ros/<distro>/setup.bash).

## Table of contents
* [Create a workspace for catkin](#Create-a-workspace-for-catkin)
  * [catkin_make](#catkin_make)
  * [catkin_tools](#catkin_tools)
* [Create a catkin package](#Create-a-catkin-package)
* [Release new version of catkin package](#Release-new-version-of-catkin-package)

### Create a workspace for catkin
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
```

**Note:** To ignore a package, place a `CATKIN_IGNORE` file in that package.

#### catkin_make
```sh
# To build any package located in ~/catkin_ws/src
catkin_make
# To build specific packages in ~/catkin_ws/src
catkin_make --pkg <pkg1> <pkg2>
# To install target
catkin_make install
```
_Replace `catkin_make` by `catkin_make_isolated` if isolated build process wanted, wherein each package is independently configured, built, and loaded into the environment. It allows `catkin_make_isolated --merge` for build each catkin package into a common devel space._

#### catkin_tools
```sh
# To build any package located in ~/catkin_ws/src
catkin build
# To build specific packages in ~/catkin_ws/src
catkin build <pkg1> <pkg2>
# To clean all the build products
catkin clean
# To clean specific packages in ~/catkin_ws/build and ~/catkin_ws/devel
catkin clean <pkg1> <pkg2>
# To run all the tests in the packages
catkin run_tests
# To run the current package
catkin run_tests --no-deps --this
# Make a new profile in order to build with a different configuration
catkin config --profile <profile> -x _<profile>
# Build a specific profile
catkin build --profile <profile>
# Clean a specific profile
catkin clean --profile <profile>
# Remove profile (you can also list, set, add and rename a profile)
catkin profile remove <profile>
```

Installation:
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt install python-catkin-tools
```

### Create a catkin package
```sh
cd ~/catkin_ws/src
catkin_create_pkg <package> <depend1> <depend2>
```

### Release new version of catkin package
```sh
# Generate CHANGELOG.rst file in each catkin package
catkin_generate_changelog
# Check each catkin package has its changelog, increment the version in the package.xml and commit/tag the changes
catkin_prepare_release                # PATCH
catkin_prepare_release --bump minor   # ABI
catkin_prepare_release --bump major   # API
# If there it's still 0 tags in the repositoy's website
git push origin --tags
# PR of the release
bloom-release <repo> --rosdistro <distro>
```
