# cmake cheat sheet

```cmd
winget install Kitware.CMake
```

- [cmake cheat sheet](#cmake-cheat-sheet)
    - [CMakeLists.txt](#cmakeliststxt)
      - [Global setup](#global-setup)
      - [Declare module](#declare-module)
      - [Declare flags](#declare-flags)
      - [Declare dependencies](#declare-dependencies)
      - [Header-only libraries](#header-only-libraries)
      - [Anti patterns](#anti-patterns)
      - [Cross platform pitfalls](#cross-platform-pitfalls)
    - [Building](#building)
    - [Installing](#installing)
    - [Running](#running)
      - [App](#app)
      - [Tests](#tests)

### CMakeLists.txt

#### Global setup

```cmake
# If you cannot have the required minimum features because you have a more recent CMake version, CMake may disable them because he wants to be compatible
cmake_minimum_required(VERSION <x.y.z>)
# Use some flags, specially Werror and W something, if you use a dependency more restrictive than you that causes having a break
if(MSVC)
  # W3: Warning level to 3(production quality)
  # WX: Treats all warnings as errors
  add_compile_options(/W3 /WX)
else()
  # W: Warning about constructions
  # Wall: Enables most warning messages
  # Werror: Treats all warnings as errors
  add_compile_options(-W -Wall -Werror)
  endif()
```

#### Declare module

```cmake
add_library(<LIB_NAME>
  src/<file1>.cpp
  src/<file2>.cpp
  # ...
)
```

#### Declare flags

```cmake
# Public headers
target_include_directories(<LIB_NAME> PUBLIC include)
# Private headers
target_include_directories(<LIB_NAME> PRIVATE src)
# Something that depends on the config of CMake
if(<SOME_SETTINGS>)
  target_compile_definitions(<LIB_NAME> PUBLIC <WITH_SOME_SETTINGS>)
  # If the setting only affects implementation
  target_compile_definitions(<LIB_NAME> PRIVATE <WITH_SOME_SETTINGS>)
endif()
```

#### Declare dependencies

```cmake
# Public (interface) dependencies
target_link_libraries(<LIB_NAME> PUBLIC <DEPS>)
# Private (implementation) dependencies
target_link_libraries(<LIB_NAME> PRIVATE <DEPS>)
```

#### Header-only libraries

Something that belongs to your public interface (you want your clients to see it), but should not be used to build your library.

```cmake
add_library(<LIB_NAME> INTERFACE)
target_include_directories(<LIB_NAME> INTERFACE include)
target_link_libraries(<LIB_NAME> INTERFACE <DEPS>)
```

#### Anti patterns

```cmake
# Don't use macros that affect all targets
INCLUDE_DIRECTORIES()
ADD_DEFINITIONS()
LINK_LIBRARIES()
# Don't use it with a path outside your module
TARGET_INCLUDE_DIRECTORIES()
# Don't use it without specifying PUBLIC, PRIVATE or INTERFACE
TARGET_LINK_LIBRARIES()
# Don't use it to set flags that affect the ABI
TARGET_COMPILE_OPTIONS()
```

#### Cross platform pitfalls

```cmake
# Command line tools
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${filepath} ${sympath})
# Instead of
execute_process(COMMAND mklink ${filepath} ${sympath}) # windows
execute_process(COMMAND ln -s ${filepath} ${sympath}) # unix

# Independent paths
target_include_directories(<LIB_NAME>
  PUBLIC
    $<INSTALL_INTERFACE:include/<LIB_PATH>>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/<LIB_PATH>>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
```

### Building

```cmd
REM clone or download the repository/project
REM cd to the repository/project where the CMakeLists.txt is
mkdir build && cd build
cmake -G <GENERATOR> -T <BUILD_TOOLS> -A <x64/Win32> -DBUILD_TESTING=<ON/OFF> ..
cmake --build . --config <Release/Debug/MinSizeRel/RelWithDebInfo>
REM if you want to build it again after making changes in the `CMakeLists.txt`, you can clean the target
cmake --build . --target clean
```

### Installing

```cmd
cmake --install . --config <Release/Debug/MinSizeRel/RelWithDebInfo>
```

### Running

#### App

```cmd
.\<Debug/Release/MinSizeRel/RelWithDebInfo>\<EXECUTABLE>.exe
```

#### Tests

```cmd
ctest
REM or directly specify the test executable
.\<Debug/Release/MinSizeRel/RelWithDebInfo>\<TEST_EXECUTABLE>.exe
```