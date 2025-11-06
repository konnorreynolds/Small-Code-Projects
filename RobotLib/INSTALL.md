# RobotLib Installation Guide

## Overview
RobotLib is a header-only C++ library, which means you don't need to compile it separately. Just copy the headers and include them in your project!

## Requirements
- **C++ Standard**: C++11 or newer
- **Compiler**: GCC 4.8+, Clang 3.4+, MSVC 2015+, or any C++11-compliant compiler

---

## Installation Methods

### Method 1: Copy to System Include Directory (Linux/macOS)

This allows you to include RobotLib like a system library.

```bash
# Copy the include folder to a system location
sudo cp -r RobotLib/include/* /usr/local/include/

# Now you can use it in your code:
# #include <RobotLib.h>
```

**In your CMakeLists.txt:**
```cmake
# No special configuration needed!
# Just include normally
```

**In your code:**
```cpp
#include <RobotLib.h>

int main() {
    auto dist = units::m(10.0);
    return 0;
}
```

---

### Method 2: Copy to Your Project (Recommended for Portability)

Copy the RobotLib folder into your project:

```
YourProject/
├── src/
│   └── main.cpp
├── lib/
│   └── RobotLib/           # Copy the entire RobotLib folder here
│       └── include/
│           ├── RobotLib.h
│           ├── units_core.h
│           ├── units_physics.h
│           └── ...
└── CMakeLists.txt
```

**In your CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.10)
project(YourProject)

set(CMAKE_CXX_STANDARD 11)

# Add RobotLib include directory
include_directories(${CMAKE_SOURCE_DIR}/lib/RobotLib/include)

add_executable(your_program src/main.cpp)
```

**In your code:**
```cpp
#include <RobotLib.h>

int main() {
    auto dist = units::m(10.0);
    return 0;
}
```

---

### Method 3: Direct Include Path (Quick Testing)

Point your compiler directly to the include folder:

**Command line compilation:**
```bash
# GCC/Clang
g++ -std=c++11 -I/path/to/RobotLib/include main.cpp -o program

# Example:
g++ -std=c++11 -I./lib/RobotLib/include src/main.cpp -o robot_program
```

**In your code:**
```cpp
#include <RobotLib.h>

int main() {
    auto dist = units::m(10.0);
    return 0;
}
```

---

### Method 4: CMake FetchContent (Modern CMake)

Add RobotLib as a dependency in your CMakeLists.txt:

```cmake
cmake_minimum_required(VERSION 3.14)
project(YourProject)

set(CMAKE_CXX_STANDARD 11)

include(FetchContent)

# If RobotLib is in a git repository:
FetchContent_Declare(
  RobotLib
  GIT_REPOSITORY https://github.com/yourusername/RobotLib.git
  GIT_TAG main
)

# Or if you have it locally:
FetchContent_Declare(
  RobotLib
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/lib/RobotLib
)

FetchContent_MakeAvailable(RobotLib)

# Add include directory
include_directories(${robotlib_SOURCE_DIR}/include)

add_executable(your_program src/main.cpp)
```

---

## Quick Test

Create a test file to verify installation:

**test_install.cpp:**
```cpp
#include <RobotLib.h>
#include <iostream>

int main() {
    using namespace units;

    // Test basic units
    auto distance = m(10.0);
    auto velocity = mps(2.5);
    auto time = distance / velocity;

    std::cout << "RobotLib v" << robotlib::VERSION_STRING << " works!" << std::endl;
    std::cout << "Distance: " << distance.toMeters() << " m" << std::endl;
    std::cout << "Velocity: " << velocity.toMetersPerSecond() << " m/s" << std::endl;
    std::cout << "Time: " << time.toSeconds() << " s" << std::endl;

    // Test PID controller
    units::robotics::PIDController pid(1.0, 0.1, 0.05);
    double output = pid.calculate(10.0, 5.0, 0.02);
    std::cout << "PID output: " << output << std::endl;

    std::cout << "\nAll tests passed!" << std::endl;
    return 0;
}
```

**Compile and run:**
```bash
# Method 2 (project-local):
g++ -std=c++11 -I./lib/RobotLib/include test_install.cpp -o test_install
./test_install

# Method 1 (system-wide):
g++ -std=c++11 test_install.cpp -o test_install
./test_install
```

**Expected output:**
```
RobotLib v2.2.0 works!
Distance: 10 m
Velocity: 2.5 m/s
Time: 4 s
PID output: [some value]

All tests passed!
```

---

## Include Options

### Full Library (Recommended):
```cpp
#include <RobotLib.h>  // Includes everything
```

### Individual Modules:
```cpp
#include <units_core.h>      // Just basic units
#include <units_robotics.h>  // + Control algorithms
#include <units_estimation.h> // + Kalman filters
// etc.
```

---

## Troubleshooting

### Error: "RobotLib.h: No such file or directory"

**Solution**: The compiler can't find the include directory.

- **CMake**: Add `include_directories(/path/to/RobotLib/include)`
- **Command line**: Add `-I/path/to/RobotLib/include`
- **IDE**: Add include path in project settings

### Error: "error: 'constexpr' does not name a type"

**Solution**: Your compiler doesn't support C++11.

- Add `-std=c++11` to compilation flags
- **CMake**: Add `set(CMAKE_CXX_STANDARD 11)`

### Error: "multiple definition of [something]"

**Solution**: You may have included a `.cpp` file or defined non-inline functions in headers.

- RobotLib is header-only; don't compile any RobotLib files
- Only compile your own `.cpp` files

### Compile time is slow

**Solution**: RobotLib is heavily templated which can slow compilation.

- Use precompiled headers (PCH) in your build system
- Only include the modules you need instead of `RobotLib.h`
- Enable compiler optimizations (`-O2` or `-O3`)

---

## Next Steps

1. ✅ Installation complete!
2. 📖 Read the [Quick Start Guide](README.md)
3. 💻 Check out [examples/](examples/)
4. 📚 Browse the comprehensive inline documentation
5. 🧪 Run the test suite: `tests/`

---

## Platform-Specific Notes

### Windows (Visual Studio)
1. Copy `RobotLib/include` to your project
2. Right-click project → Properties → C/C++ → General → Additional Include Directories
3. Add: `$(ProjectDir)\lib\RobotLib\include`

### macOS (Xcode)
1. Drag `RobotLib/include` folder into your project
2. Select "Create folder references"
3. In Build Settings → Header Search Paths, add: `$(PROJECT_DIR)/lib/RobotLib/include`

### Linux (Any IDE)
1. Copy to `/usr/local/include/` (system-wide)
2. Or add to project and configure include paths
3. No additional libraries needed (header-only!)

---

## Support

For issues, questions, or contributions:
- Check the inline documentation in headers
- See examples in `examples/` directory
- Review test cases in `tests/` directory
