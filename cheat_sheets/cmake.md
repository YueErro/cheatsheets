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
      - [CDash](#cdash)
    - [CMakeLists.txt](#cmakeliststxt)
      - [Anti patterns](#anti-patterns)
      - [Cross platform pitfalls](#cross-platform-pitfalls)
      - [Custom targets](#custom-targets)
      - [Files](#files)
      - [Language requirements](#language-requirements)
      - [Symbols visibility](#symbols-visibility)
      - [Example](#example)
      - [`ctest`](#ctest)

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
.\<Debug/Release/MinSizeRel/RelWithDebInfo>/<EXECUTABLE>
```

#### Tests

```bash
# If you need to re-build a particular test
cmake --build build --config <Debug/Release/MinSizeRel/RelWithDebInfo> --target <TEST_EXECUTABLE>
# Execute all the tests:
#   -V: Verbosity instead of only on failure
#   -N: Print existing tests
#   -R: Include tests
#   -E: Exclude tests
#   -I: Include tests by number rather name
#   -L: Include tests by label
#   -LE: Exclude tests by label
#   --repeat-until-fail num
#   -j num: Parallel execution
ctest --output-on-failure --test-dir build -C <Debug/Release/MinSizeRel/RelWithDebInfo>
# Or directly specify the test executable
.\<Debug/Release/MinSizeRel/RelWithDebInfo>/<TEST_EXECUTABLE>
# It is also possible to build and test with ctest
# Checkout the 24.7. Build And Test Mode section in the book mentioned above
# Test results report (CDash below)
```

#### CDash

Web-based dashboard which collects results from a SW build and test pipeline driven by `ctest`.
There are three important concepts that tie together how CTest and CDash execute pipelines and report results:
  * Steps (or actions): sequence of actions that a pipeline performs
    * Start
    * Update
    * Configure
    * Build
    * Test
    * Coverage
    * MemCheck
    * Submit
  * Models (or modes): define certain behaviors, such as whether or not to continue with later step after a particular step fails
    * Nightly: intended to be invoked once per day, usually by a job during a time when the executing machine is less busy and includes all the steps above except *MemCheck*, if the *Update* step fails, the rest of the steps will still be executed
    * Continuous: similar to *Nightly* except that it's intended to be run multiple times a day as needed, normally in response to a change being committed and defines the same steps, but if the *Update* fails, the later steps will not be executed
    * Experimental: hoc experiments executed by developers (this is the default model set) as needed and it includes all steps except *Update* and *MemCheck*
  * Tracks: controls which group the pipeline results will be shown under in the dashboard results, the *Coverage* and *MemCheck* steps are shown in *Coverage* and *Dynamic Analysis* dedicated groups respectively


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

#### Files

```cmake
set(BASE_PATH /base)
set(FOOBASE_PATH /base/foo/bar)
set(OTHER_PATH /other/place)

file(RELATIVE_PATH foobar ${BASE_PATH} ${FOOBAR_PATH})  # foobar = foo/bar
file(RELATIVE_PATH other ${BASE_PATH} ${OTHER_PATH})    # other = ../other/place

# Avoid mixing forward slashes and backslashes, use CMake convention
set(CUSTOM_PATH /usr/local/bin:/usr/bin:/bin)
file(TO_CMAKE_PATH ${CUSTOM_PATH} custom_path) # custom_path = /usr/local/bin:/usr/bin:/bin

# Copy files
configure_file( src dest [COPYONLY|@ONLY] [ESCAPE_QUOTES]) # without the 1st option anything in the form of CMake variable will be replaced by its value
# COPYONLY: Substitution of CMake variables in file not needed
# @ONLY: Limit substitution to only @var@ form
# ESCAPE_QUOTES: \" will appear as it is
set(BAR "Some \"quoted\" value")
configure_file(quoting.txt.in quoting.txt)
configure_file(quoting.txt.in quoting.txt ESCAPE_QUOTES)
# quoting.txt.in
#   A:  @BAR@
#   B: "@BAR@"
# quoting.txt
#   A:  Some "quoted" value
#   B: "Some "quoted" value"
# quoting_escaped.txt
#   A:  Some \"quoted\" value
#   B: "Some \"quoted\" value"
set(USER_FILE whoami.txt)
configure_file(whoami.sh.in whoami.txt)
configure_file(whoami.sh.in whoami.txt COPYONLY)
configure_file(whoami.sh.in whoami.txt @ONLY)
# whoami.sh.in
#   #!/bin/sh
#   echo ${USER} > "@USER_FILE@"
# whoami.sh
#   #!/bin/sh
#   echo  > ""
# whoami_copyonly.sh
#   #!/bin/sh
#   echo ${USER} > "@USER_FILE@"
# whoami_only.sh
#   #!/bin/sh
#   echo ${USER} > "whoami.txt"

# If no substitution is needed, another alternative
file(COPY|INSTALL file_path # To all the files in path don't forget the forward slash at the end
# Preserves original timestamp unless it already exists in destination, in such a case it's ignored
# COPY preserves original permissions but no print status messages
# INSTALL doesn't preserve original permissions but prints status messages
  DESTINATION dir_path
  # Available permissions (Unix): OWNER_READ, OWNER_WRITE, OWNER_EXECUTE
  #                               GROUP_READ, GROUP_WRITE, GROUP_EXECUTE
  #                               WORLD_READ, WORLD_WRITE, WORLD_EXECUTE
  #                               SETUID,     SETGID
  # If not available in the platform, then it is ignored
  [NO_SOURCE_PERMISSIONS|USE_SOURCE_PERMISSIONS|FILE_PERMISSIONS file_perm ...|DIRECTORY_PERMISSIONS dir_perm ...]
  [FILES_MATCHING]
  [PATTERN pattern|REGEX regex|] [EXCLUDE]
  [PERMISSIONS perm ...]
  [...]
)
# One more alternative
add_custom_target(copy_target_name
  COMMAND ${CMAKE_COMMAND} -E make_directory output/textfiles
  COMMAND ${CMAKE_COMMAND} -E copy a.txt b.txt output/textfiles
)
# Don't forget to check RESULT_VARIABLE if it should stop if the command fails

# Write files
file(WRITE|APPEND file_path content) # Writes truncates
# Example
file(WRITE multi.txt [=[
  1st line
  2nd line
]=])
# Replace [=[ and ]=] with [[ and ]] if there is no CMake variable get the value from
# If you want to add a condition use GENERATE
file(GENERATE
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/outfile-$<CONFIG>.txt
  INPUT ${CMAKE_CURRENT_SOURCE_DIR}/input.txt.in # If in-place use CONTENT instead of INPUT
  CONDITION $<NOT:$<CONFIG:Release>> # Only if in release mode
)

# Read files
file(READ file_path out_var
  # If no more options the content will be stored in a single string
  [OFFSET offset] # Read only from the offset specified
  [LIMIT byte] # The maximum number of bytes to read
  [HEX] # Store in hexadecimal representation, useful if it's a binary data
)
# For a more complex read use STRINGS
file(STRINGS file_path out_var
  [LENGTH_MAXIMUM max_bytes_line] # To exclude longer than
  [LENGTH_MINIMUM min_bytes_line] # To exclude shorter than
  [LIMIT_INPUT max_read_bytes]
  [LIMIT_OUTPUT max_stored_bytes]
  [LIMIT_COUNT max_stored_lines] # Limits the total number of lines rather than bytes
  [REGEX regex] # For a even more complex readings
)

# File system manipulation
file(RENAME src_path dst_path) # If it exists, like Unix mv command
file(REMOVE file_path ...) # If file doesn't exists it doesn't report error
file(REMOVE_RECURSE dir_path ...)
file(MAKE_DIRECTORY dir_path ...) # Does't report error if dir already exists

# To list particular files use GLOB
file(GLOB|GLOB_RECURSE out_var
  [LIST_DIRECTORIES true|false]
  [RELATIVE dir_path]
  [CONFIGURE_DEPENDS] # (CMake 3.12)
  expression ...
)
# Example
set(USR_SHARE_PATH /usr/share)
file(GLOB_RECURSE images
  RELATIVE ${USR_SHARE_PATH}
  ${USR_SHARE_PATH}/*/*.png
)
# Recommended to use them just for debugging purposes

# Download files
file(DOWNLOAD url file_path [option ...])
# Available options:
#   EXPECTED_HASH MD5|SHA1|...=val: Checksum
#   TLS_VERIFY true|false: Perform server certificate verification, if false checks for CMAKE_TLS_VERIFY
#   TLS_CAINFO file_path: A custom Certificate Authority file
# Upload files
file(UPLOAD file_path url [option ...])
# Available options in both:
#   LOG out_var: Saves logged output from the operation
#   SHOW_PROGRESS: Logs progress information as status messages
#   TIMEOUT secs: Abort if the secs have elapsed
#   INACTIVITY_TIMEOUT secs: Abort only if in some progress stage the secs have elapsed
# (CMake 3.7):
#   USERPWD username:password: Authentication details, passwords shouldn't be present perse
#   HTTPHEADER: Can be used multiple times, i.e.:
#     HTTPHEADER "Host: somebucket.s3.amazonaws.com"
#     HTTPHEADER "Date: ${timestamp}"
#     HTTPHEADER "Content-Type: application/x-compressed-tar"
#     HTTPHEADER "Authorization: AWS ${s3key}:${signature}"
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

#### Symbols visibility

```cmake
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINE_HIDDEN YES) # Applies to all languages
# Windows symbols are not exported by default, whereas in GCC and Clang they are
set(WINDOWS_EXPORT_ALL_SYMBOLS ON) # Brute force

# Specify individual symbol visibilities
target_include_directories(shared_lib_name PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")
# Writes out shared_lib_name_export.h to the current binary directory
include(GenerateExportHeader)
# The header must contain __declspec(...) on Windows or __attribute__(...) with name uppercase(shared_lib_name) + "_EXPORT"
generate_export_header(shared_lib_name)
# If a different name is desired
generate_export_header(shared_lib_name # BASE_NAME custom_base_name
  # If BASE_NAME: custom_base_name_export.h and uppercase(custom_base_name) + "_EXPORT"
  EXPORT_FILE_NAME export_shared_lib_name.hpp
  EXPORT_MACRO_NAME SHARED_LIB_NAME_API
  # NO_EXPORT_MACRO_NAME REALLY_PRIVATE: For a deprecated symbol, uppercase(shared_lib_name) + "_DEPRECATED"
  # NO_DEPRECATED_MACRO_NAME MACRO_NAME_DEPRECATED: Give the desired name
)
# For STATIC and SHARED libs
generate_export_header(shared_lib_name)
# Treat static libraries as being sub-groups within the shared library, outside targets only ever linking to the shared one
target_link_libraries(shared_lib_name PRIVATE static_lib_name)
target_include_directories(shared_lib_name PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")
target_include_directories(static_lib_name PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(static_lib_name PRIVATE
  # Comes from the uppercase(BASE_NAME)
  SHARED_LIB_NAME_EXPORTS # As not changed above it is uppercase(shared_lib_name) + "_EXPORTS"
  # WARNING: Not sure if it is EXPORT OR EXPORTS if its by default
)
```

#### Example

```cmake
cmake_minimum_required(VERSION 3.22)

# CMake 3.12:
#   CMAKE_PROJECT_VERSION
#     CMAKE_PROJECT_VERSION_MAJOR
#     CMAKE_PROJECT_VERSION_MINOR
#     CMAKE_PROJECT_VERSION_PATCH
#     CMAKE_PROJECT_VERSION_TWEAK
project(proj_name VERSION 1.0.0 LANGUAGES CXX)

# To add MACROS coming from CMake use:
add_definitions(-DCUSTOM_MACRO=\"${PROJECT_NAME}\") # Use it in the code as CUSTOM_MACRO

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

# More details in the book mentioned above, section 23.5. Finding Packages
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

if(BUILD_TESTING)
  enable_testing()
  # See ctest section
endif()

```

#### `ctest`

```cmake
include(CTest)

add_executable(${PROJECT_NAME}_test test/proj_name.test.cpp)
# It also accepts:
#   CONFIGURATIONS Debug|RelWithDebInfo|""|... --> "" means when no configuration is specified
#   WORKING_DIRECTORY <path> --> To make the test run in some other location, highly recommended to be an absolute path
add_test(${PROJECT_NAME}.test ${PROJECT_NAME}_test) # Executed for all configurations from CMAKE_CURRENT_BINARY_DIR
set_tests_properties(${PROJECT_NAME}.test PROPERTIES
  # Disable this particular test (can be done from command line as explained in the Tests section above)
  DISABLED YES
  # Marked as failed if it hasn't finished in that time
  TIMEOUT secs
  # Different tests can be labeled with the same name to be executed/excluded with -L and -LE respectively
  LABELS "test_label"
  # Condition to start executing this one in parallel executions (-j)
  RESOURCE_LOCK rsrc # For multiple resources separate them with ;
  # To control the test order execution
  DEPENDS other_test_name # For multiple dependencies separate them with ;
  # For test fixtures (CMake 3.7) see section 24.5. Test Dependencies in the book mentioned above
)
```