# RobotLib Include Path Troubleshooting Guide

## Quick Test

First, let's verify RobotLib works on your system:

```bash
cd RobotLib
g++ -std=c++11 test_include.cpp -o test_include
./test_include
```

If this works, RobotLib itself is fine. The issue is with YOUR project's include path.

---

## Common Issues & Solutions

### ❌ Error: "RobotLib.h: No such file or directory"

**Problem:** Compiler can't find the RobotLib headers.

**Solution 1: Check your project structure**

Your project should look like this:
```
MyProject/
├── src/
│   └── main.cpp
├── lib/                      ← You need this folder!
│   └── RobotLib/            ← Entire RobotLib folder here
│       └── include/
│           ├── RobotLib.h   ← Main header
│           ├── units_core.h
│           └── ...
└── CMakeLists.txt (or Makefile)
```

**Solution 2: Verify the copy was complete**

```bash
# Check if the main header exists
ls lib/RobotLib/include/RobotLib.h

# Should list the file. If not, you didn't copy it correctly!
```

**Solution 3: Add include path to compilation**

```bash
# Command line (adjust path as needed):
g++ -std=c++11 -I./lib/RobotLib/include src/main.cpp -o program

# Or absolute path:
g++ -std=c++11 -I/full/path/to/RobotLib/include src/main.cpp -o program
```

---

### ❌ Error: "units_core.h: No such file or directory"

**Problem:** RobotLib.h is found, but its internal includes aren't.

**Cause:** You only copied RobotLib.h, not the entire include folder!

**Solution:** Copy the ENTIRE `RobotLib/include/` directory:

```bash
# Wrong (only copies one file):
cp RobotLib/include/RobotLib.h lib/

# Correct (copies everything):
cp -r RobotLib/include lib/RobotLib/

# Or copy entire RobotLib folder:
cp -r RobotLib lib/
```

---

### ❌ CMake Error: Can't find headers

**Problem:** CMakeLists.txt doesn't know where RobotLib is.

**Solution:** Add include directory to CMakeLists.txt:

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyRobot)

set(CMAKE_CXX_STANDARD 11)

# THIS LINE IS REQUIRED:
include_directories(${CMAKE_SOURCE_DIR}/lib/RobotLib/include)

add_executable(myrobot src/main.cpp)
```

**Or use target-specific includes:**
```cmake
add_executable(myrobot src/main.cpp)
target_include_directories(myrobot PRIVATE ${CMAKE_SOURCE_DIR}/lib/RobotLib/include)
```

---

### ❌ Makefile Error

**Problem:** Makefile doesn't have include path.

**Solution:** Add -I flag to CXXFLAGS:

```makefile
CXX = g++
CXXFLAGS = -std=c++11 -I./lib/RobotLib/include
SRCS = src/main.cpp
TARGET = myrobot

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)
```

---

### ❌ Visual Studio / IDE Issues

**Visual Studio (Windows):**
1. Right-click project → Properties
2. C/C++ → General → Additional Include Directories
3. Add: `$(ProjectDir)\lib\RobotLib\include`
4. Apply and rebuild

**VSCode:**
Add to `.vscode/c_cpp_properties.json`:
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "${workspaceFolder}/lib/RobotLib/include"
            ],
            "cStandard": "c11",
            "cppStandard": "c++11"
        }
    ]
}
```

**CLion:**
Ensure CMakeLists.txt has `include_directories()` line.

---

### ❌ Error: Multiple definition of...

**Problem:** Linking errors about multiple definitions.

**Cause:** You're accidentally compiling RobotLib headers as source files.

**Solution:** RobotLib is header-only! Only compile YOUR .cpp files:

```bash
# Wrong:
g++ src/main.cpp lib/RobotLib/include/*.h -o program

# Correct:
g++ -std=c++11 -I./lib/RobotLib/include src/main.cpp -o program
```

---

### ❌ Error: 'constexpr' does not name a type

**Problem:** C++11 not enabled.

**Solution:**

```bash
# Command line:
g++ -std=c++11 ...

# CMake:
set(CMAKE_CXX_STANDARD 11)

# Or:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
```

---

## Step-by-Step Setup Guide

### Method 1: Copy to Project (Recommended)

```bash
# 1. Create lib folder in your project
mkdir -p MyProject/lib

# 2. Copy ENTIRE RobotLib folder
cp -r /path/to/RobotLib MyProject/lib/

# 3. Verify structure
ls MyProject/lib/RobotLib/include/RobotLib.h
# Should show the file

# 4. Create test file
cat > MyProject/test.cpp << 'EOF'
#include <RobotLib.h>
#include <iostream>

int main() {
    std::cout << "RobotLib v" << robotlib::VERSION_STRING << std::endl;
    auto dist = units::m(5.0);
    std::cout << "Distance: " << dist.toMeters() << " m" << std::endl;
    return 0;
}
EOF

# 5. Compile and test
cd MyProject
g++ -std=c++11 -I./lib/RobotLib/include test.cpp -o test
./test

# Should output:
# RobotLib v2.2.0
# Distance: 5 m
```

### Method 2: System-Wide Installation

```bash
# Copy to system includes (requires sudo)
sudo cp -r RobotLib/include/* /usr/local/include/

# Verify
ls /usr/local/include/RobotLib.h

# Now compile without -I flag:
g++ -std=c++11 test.cpp -o test
```

---

## Quick Diagnostic Script

Save this as `check_robotlib.sh`:

```bash
#!/bin/bash

echo "🔍 RobotLib Installation Checker"
echo "================================"
echo ""

# Check 1: Is RobotLib in project?
echo "1. Checking project structure..."
if [ -f "lib/RobotLib/include/RobotLib.h" ]; then
    echo "   ✅ Found: lib/RobotLib/include/RobotLib.h"
else
    echo "   ❌ Missing: lib/RobotLib/include/RobotLib.h"
    echo "   → Copy RobotLib to lib/ directory"
fi

# Check 2: Count header files
echo ""
echo "2. Counting header files..."
COUNT=$(find lib/RobotLib/include -name "*.h" 2>/dev/null | wc -l)
if [ $COUNT -gt 10 ]; then
    echo "   ✅ Found $COUNT header files"
else
    echo "   ❌ Only found $COUNT header files (should be ~12)"
    echo "   → Incomplete copy, re-copy entire RobotLib/include/"
fi

# Check 3: Test compilation
echo ""
echo "3. Testing compilation..."
cat > /tmp/robotlib_test.cpp << 'EOF'
#include <RobotLib.h>
int main() { auto d = units::m(5); return 0; }
EOF

if g++ -std=c++11 -I./lib/RobotLib/include /tmp/robotlib_test.cpp -o /tmp/robotlib_test 2>/dev/null; then
    echo "   ✅ Compilation successful!"
    rm /tmp/robotlib_test /tmp/robotlib_test.cpp
else
    echo "   ❌ Compilation failed"
    echo "   → Check error messages above"
fi

echo ""
echo "================================"
```

Run it:
```bash
chmod +x check_robotlib.sh
./check_robotlib.sh
```

---

## Still Having Issues?

### 1. Show me your exact error

```bash
# Compile with verbose output:
g++ -std=c++11 -v -I./lib/RobotLib/include src/main.cpp -o program 2>&1 | tee error.log
```

Post the contents of `error.log`.

### 2. Show me your structure

```bash
tree -L 3 MyProject
# Or:
find MyProject -type f -name "*.h" -o -name "*.cpp" | head -20
```

### 3. Minimal Test Case

Create a file `minimal_test.cpp`:

```cpp
#include <RobotLib.h>
#include <iostream>

int main() {
    std::cout << "Version: " << robotlib::VERSION_STRING << std::endl;
    return 0;
}
```

Try compiling:
```bash
g++ -std=c++11 -I./lib/RobotLib/include minimal_test.cpp -o minimal_test
```

If this fails, show me the EXACT error message!

---

## Common Mistakes Checklist

- [ ] I copied the ENTIRE `RobotLib/include/` folder, not just `RobotLib.h`
- [ ] My include path points to the `include/` folder (not `RobotLib/` or deeper)
- [ ] I'm using `-std=c++11` or higher
- [ ] I'm NOT trying to compile .h files
- [ ] My project structure matches the examples
- [ ] I'm using `#include <RobotLib.h>` with angle brackets
- [ ] I added `include_directories()` to CMakeLists.txt (if using CMake)

---

## Working Example

**Project Structure:**
```
MyRobot/
├── main.cpp
├── lib/
│   └── RobotLib/
│       └── include/
│           └── RobotLib.h (and all other .h files)
└── CMakeLists.txt
```

**main.cpp:**
```cpp
#include <RobotLib.h>
#include <iostream>

int main() {
    using namespace units;
    auto dist = m(10.0);
    std::cout << "Works! " << dist.toMeters() << " m" << std::endl;
    return 0;
}
```

**CMakeLists.txt:**
```cmake
cmake_minimum_required(VERSION 3.10)
project(MyRobot)
set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_SOURCE_DIR}/lib/RobotLib/include)

add_executable(myrobot main.cpp)
```

**Build:**
```bash
mkdir build && cd build
cmake ..
make
./myrobot
```

**Or compile directly:**
```bash
g++ -std=c++11 -I./lib/RobotLib/include main.cpp -o myrobot
./myrobot
```

Should output: `Works! 10 m`

---

If you're still stuck, please provide:
1. Your exact error message
2. Your project directory structure (use `tree` or `ls -R`)
3. Your compilation command
4. Your OS and compiler version

We'll get it working! 🚀
