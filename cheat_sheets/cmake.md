# cmake cheat sheet

Mainly based on [Professional CMake A Practical Guide (Craig Scott)](https://github.com/YueErro/programming_books/blob/main/books/Professional%20CMake%20A%20Practical%20Guide%20(Craig%20Scott)%20(z-lib.org).pdf) book.

```cmd
sudo apt install cmake
winget install Kitware.CMake
```

- [cmake cheat sheet](#cmake-cheat-sheet)
    - [Building](#building)
    - [Installing](#installing)
    - [Running](#running)
      - [App](#app)
      - [Tests](#tests)
    - [CMakeLists.txt](#cmakeliststxt)
      - [Anti patterns](#anti-patterns)
      - [Cross platform pitfalls](#cross-platform-pitfalls)
      - [Language requirements](#language-requirements)
      - [Example](#example)

### Building

```bash
# Clone or download the repository/project
# cd to the repository/project where the CMakeLists.txt is
mkdir build && cd build
cmake -G <GENERATOR> -T <BUILD_TOOLS> -A <x64/Win32> -DBUILD_TESTING=<ON/OFF> ..
# Debug: With no optimizations and full debug information
# Release: With full optimization and no debug information
# RelWithDebInfo: Compromise of the previous two
# MinRizeRel: Optimized for size rather than speed and no debug information is created, used for embedded devices
cmake --build . --config <Release/Debug/MinSizeRel/RelWithDebInfo>
# If you want to build it again after making changes in the `CMakeLists.txt`, you can clean the target
cmake --build . --target clean
```

### Installing

```bash
cmake --install . --config <Release/Debug/MinSizeRel/RelWithDebInfo>
```

### Running

#### App

```bash
.\<Debug/Release/MinSizeRel/RelWithDebInfo>/<EXECUTABLE>.exe
```

#### Tests

```bash
ctest
# Or directly specify the test executable
.\<Debug/Release/MinSizeRel/RelWithDebInfo>/<TEST_EXECUTABLE>.exe
```

### CMakeLists.txt

#### Anti patterns

```cmake
# Don't use macros that affect all targets, prefer to use the target_...() instead
include_directories() # list of directories to be used as header search paths [AFTER|BEFORE] [SYSTEM]
add_definitions()
link_libraries()
# Don't use it with a path outside your module
target_include_directories()
# Don't use it without specifying PUBLIC, PRIVATE or INTERFACE
target_link_libraries()
# Don't use it to set flags that affect the ABI
target_compile_options() # BEFORE and PRIVATE|PUBLIC|INTERFACE options
```

#### Cross platform pitfalls

```cmake
# Command line tools
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${filepath} ${sympath}
  RESULT_VARIABLE symlink_result
)
if(NOT symlink_result EQUAL 0)
  message(FATAL_ERROR "Couldn't create symlink: ${filepath} -> ${sympath}")
  return(FATAL_ERROR)
endif()
# Instead of
execute_process(COMMAND mklink ${filepath} ${sympath}) # windows
execute_process(COMMAND ln -s ${filepath} ${sympath}) # unix
# All options
execute_process(COMMAND ...
  [COMMAND ...]
  [WORKING_DIRECTORY dir_path]
  [RESULT_VARIABLE result_var]
  [RESULTS_VARIABLE results_var]
  [OUTPUT_VARIABLE out_var]
  [ERROR_VARIABLE err_var]
  [OUTPUT_STRIP_TRAILING_WHITESPACE]
  [ERROR_STRIP_TRAILING_WHITESPACE]
  [INPUT_FILE in_file]
  [OUTPUT_FILE out_file]
  [ERROR_FILE err_file]
  [OUTPUT_QUIET]
  [ERROR_QUIET]
  [TIMEOUT secs]
)
# Make us of the available not CMake-specific flags such as:
# -E: Extended features, enhanced output, error handling and expression evaluation
# -P: Preprocessing, path specification, project settings
# -D: Define a variable, preprocessor macro

# Independent paths
target_include_directories(<LIB_NAME> # BEFORE | SYSTEM: to prepend to existing ones or treat as system include paths
  # Adds header search paths to INCLUDE_DIRECTORIES and INTERFACE_INCLUDE_DIRECTORIES
  PUBLIC
    $<INSTALL_INTERFACE:include/<LIB_PATH>>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/<LIB_PATH>>
  # Adds only to INCLUDE_DIRECTORIES
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  # INTERFACE adds only to INTERFACE_INCLUDE_DIRECTORIES
)
```

#### Custom targets

```cmake
# Nonexistent target
add_custom_target(target_name [ALL] # ALL: Target is always built, otherwise only on demand
  [COMMAND ...] # Multiple commands are allowed, each of them preceded by the COMMAND KEYWORD
  [COMMAND ...] # Typically running a script or a system-provided executable or so on.
  [DEPENDS dep1 ...] # Relationship in terms of file dependencies, to indicate that the other custom commands need to be executed 1st
  [BYPRODUCTS files ...] # To indicate that you are generating files from command, i.e. form IDL, docs, dat
  [WORKING_DIRECTORY dir_path] # By default ${CMAKE_CURRENT_BINARY_DIR}
  [COMMENT comment] # Log message, prefer to use command with -E echo
  [VERBATIM] # To indicate that the only escaping does is by CMake itself
  [USES_TERMINAL] # (CMake 3.2) To use a new terminal
  [SOURCES src1 ...] # Only for listing files with the target so that they can be shown them in an appropriate context
)

# Existing target
add_custom_command(TARGET target_name [PRE_BUILD|PRE_LINK|POST_BUILD] # If not specified it won't be executed if the target is already built
  # Recommended to avoid using PRE_BUILD
  # Same as add_custom_target(), but it doesn't create a new target
)

# Use execute_process() if commands needs to be executed in a particular time of the build stage

# For OUTPUT command see section "17.3 Commands That Generate Files" in the book mentioned above
```

#### Language requirements

`<LANG>_STANDARD`:

- `C_STANDARD`: 90, 99 or 11
- `CXX_STANDARD`: 98, 11 or 14
- `CXX_STANDARD` (CMake 3.8): 98, 11, 14 or 17
- `CXX_STANDARD` (CMake 3.12): 98, 11, 14, 17 or 20
- `CUDA_STANDARD` (CMAKE 3.8): 98, 11

For a minimum required language use `<LANG>_STANDARD_REQUIRED`.
Many compilers support their own extensions to the language standard and `<LANG>_EXTENSIONS` controls whether those extensions are enabled or not. If extensions are enabled `<LANG>_STANDARD` must be set, otherwise the extensions will be ignored.

#### Example

```cmake
cmake_minimum_required(VERSION 3.22)

project(proj_name VERSION 1.0.0 LANGUAGES CXX)

# Right after project definition
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Enabling it allows GNU __attribute__((aligned)) and Microsoft __declspec(dllexport)) for instance
set(CMAKE_CXX_EXTENSIONS OFF) # It is enabled by default

# For a particular target use:
# set_target_properties(target_name PROPERTIES
#   CXX_STANDARD          20
#   CXX_STANDARD_REQUIRED ON
#   CXX_EXTENSIONS        OFF
# )
# If you need a more specific standard feature use:
# target_compile_features(target_name PRIVATE|PUBLIC|INTERFACE cxx_std_20)
# If multiple standards are set, it will enforce the stronger one


find_package(yaml-cpp CONFIG REQUIRED)
find_package(eigen3 CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)

# STATUS: Incidental information, preceded by two hyphens
# WARNING: Highlighted in red, processing will continue
# AUTHOR-WARNING: Like WARNING, but only enable if with -Wdev flag
# SEND_ERROR: Error message highlighted in red, processing will continue until the configure
#             stage completes, but generation will not be performed
# FATAL_ERROR: Hard error, processing will stop immediately
# DEPRECATION: Used to log deprecation message, if neither CMAKE_ERROR_DEPRECATED nor
#              CMAKE_WARN_DEPRECATED are set to true, the message won't be shown
message(STATUS "The Project name is: ${PROJECT_NAME}")
# Alternatively
set(print_var "print")
include(CMakePrintHelpers)
cmake_print_variables(print_var CMAKE_VERSION) # print_var="print" ; CMAKE_VERSION="3.22"
# Note: Always use double quotes to append new values to existing flags
set(watchdog_var "It logs all accesses to it")
variable_watch(watchdog_var)

# Compiler such as Microsoft Visual C++
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
  # For a general case, not recommended, to preserve exiting cache variable
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")
endif()


# Filesystem
#   if(EXISTS pathToFileOrDir)
#   if(IS_DIRECTORY pathToDir)
#   if(IS_SYMLINK filename)
#   if(IS_ABSOLUTE path)
#   if(file1 IS_NEWER_THAN file2)
# Existence
#   if(DEFINED name)
#   if(COMMAND name)
#   if(POLICY name)
#   if(TARGET name)
#   if(TEST name)
#   if(value IN_LIST listVar)
if($watchdog_var)
# Numeric operators: LESS, GREATER, EQUAL, LESS_EQUAL, GREATER_EQUAL
elseif(${PROJECT_NAME})
# String: STRLESS, STRGREATER, STREQUAL, STRLESS_EQUAL, STRGREATER_EQUAL
# Any variable can be converted to lower or upper case as follows:
# $<LOWER_CASE:${watchdog_var}> STREQUAL $<UPPER_CASE:${watchdog_var}> --> False
else()
# Version numbers: VERSION_LESS, VERSION_GREATER, VERSION_EQUAL, VERSION_LESS_EQUAL, VERSION_GREATER_EQUAL
endif()

# STATIC: Static library, on Windows .lib and on Unix .a
add_library(${PROJECT_NAME}_private STATIC
  src/private.cpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
    PROJECT_DIR="${PROJECT_SOURCE_DIR}"
)
# PRIVATE: Uses it internally in private implementation
target_link_libraries(${PROJECT_NAME}_private PRIVATE yaml-cpp)

# SHARED: Dynamic linked library, on Windows .dll and on Unix .so
add_library(${PROJECT_NAME}_public SHARED
  src/public.cpp
)
# PUBLIC: Uses it internally in private implementation, but also in public headers
target_link_libraries(${PROJECT_NAME}_public PUBLIC Eigen3::Eigen)

# MODULE: Like shared but intended tob e loaded dynamically at run-time
#         Typically plugins or optional components the user may choose to be loaded or not
add_library(${PROJECT_NAME}_interface MODULE
  src/interface.cpp
)
# INTERFACE: Uses it in public headers, i.e. header-only library
target_link_libraries(${PROJECT_NAME}_interface INTERFACE ${PROJECT_NAME}_interface fmt::fmt-header-only)

# (CMake 3.9) Also exists OBJECT type (rarely used) and UNKNOWN (mostly used for imported third-party libraries)
add_library(${PROJECT_NAME}_imported UNKNOWN IMPORTED GLOBAL)
set(THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/extern")
set_target_properties(${PROJECT_NAME}_imported PROPERTIES
  IMPORTED_LOCATION "${THIRD_PARTY_DIR}/lib"
  INTERFACE_INCLUDE_DIRECTORIES "${THIRD_PARTY_DIR}/include"
  IMPORTED_GLOBAL TRUE # If necessary
)
# More details in the book mentioned above, section 16.2. Libraries

set(${PROJECT_NAME}_LIBRARIES ${PROJECT_NAME}_private ${PROJECT_NAME}_public ${PROJECT_NAME}_interface)
foreach(projLib IN LISTS ${PROJECT_NAME}_LIBRARIES)
# projLib is proj_name_private, then proj_name_public and then proj_name_interface
endforeach()
foreach(dir config data)
# dir value is config, then data, then ...
endforeach()
foreach(var IN LISTS ${PROJECT_NAME}_LIBRARIES ITEMS config data)
# var is a union of the two previous cases
endforeach()

# WIN32 (Windows platform): builds it as Windows GUI with WinMain() instead of main() and with the /SUBSYSTEM:WINDOWS
# MACOSX_BUNDLE (Apple platform): builds an app bundle
add_executable(${PROJECT_NAME}_exec # WIN32|MACOSX_BUNDLE
  src/main.cpp
)
# To reference an executable that is built outside the project's CMake, GLOBAL outside somewhere in the project
add_executable(${PROJECT_NAME}_exec IMPORTED GLOBAL)
# read-only, it doesn't create a new build target, the alias points to the real target, alias of an alias not supported
add_executable(${PROJECT_NAME}_exec_alias ALIAS ${PROJECT_NAME}_exec)

# TODO

```
