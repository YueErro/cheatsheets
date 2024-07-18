# cmake cheat sheet

```cmd
winget install Kitware.CMake
```

- [cmake cheat sheet](#cmake-cheat-sheet)
  - [Building](#building)
  - [Installing](#installing)
  - [Running](#running)
    - [App](#app)
    - [Tests](#tests)

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