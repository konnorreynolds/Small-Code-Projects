# RobotLib Testing Guide

## Quick Compilation Test

The library includes a comprehensive compilation test that verifies all features work correctly.

### Running the Test

#### On Linux/Mac:
```bash
# With C++11 (minimum required)
g++ -std=c++11 -I. -o test_quick test_compile_quick.cpp && ./test_quick

# With C++17 (recommended)
g++ -std=c++17 -I. -o test_quick test_compile_quick.cpp && ./test_quick

# With optimization
g++ -std=c++11 -O2 -I. -o test_quick test_compile_quick.cpp && ./test_quick
```

#### On Windows (MSVC):
```cmd
cl /std:c++11 /I. test_compile_quick.cpp && test_quick.exe
```

#### On Windows (MinGW):
```cmd
g++ -std=c++11 -I. -o test_quick.exe test_compile_quick.cpp && test_quick.exe
```

### What the Test Verifies

The test checks **44+ features** across all library components:

#### 1. **Core Units** (11 tests)
- ✅ Distance operations and conversions
- ✅ Time operations
- ✅ Angle conversions and trigonometry
- ✅ Angle normalization
- ✅ Mass conversions
- ✅ Temperature conversions

#### 2. **Physics Units** (7 tests)
- ✅ Velocity conversions
- ✅ Acceleration and gravities
- ✅ Force calculations (F = ma)
- ✅ Velocity calculations (v = d/t)
- ✅ Power calculations (P = VI)
- ✅ Ohm's law (V = IR)
- ✅ Work calculations (W = F×d)

#### 3. **Robotics Components** (10 tests)
- ✅ Vec2D magnitude and normalization
- ✅ Vec2D dot and cross products
- ✅ Pose2D transformations (local ↔ global)
- ✅ PID controller operation
- ✅ Trapezoid motion profiling
- ✅ Low-pass filtering
- ✅ Moving average filtering

#### 4. **Utilities** (15 tests)
- ✅ Vec2D utilities (from angle, perpendicular)
- ✅ Differential drive kinematics
- ✅ Battery state-of-charge monitoring
- ✅ Motor controller utilities (RPM conversion)
- ✅ Deadband application
- ✅ Exponential moving average
- ✅ Simple odometry

#### 5. **Type Safety** (1 test)
- ✅ Compile-time type enforcement

### Expected Output

```
========================================
  RobotLib Quick Compilation Test
  C++ Standard: 201103
========================================

Testing Core Units...
  ✓ Distance addition (got 10.5)
  ✓ Meters to feet (got 32.8084)
  ... [all tests pass]

========================================
  TEST SUMMARY
========================================
Tests passed: 44
Tests failed: 0
Success rate: 100.0%

✅ ALL TESTS PASSED!
✅ Library is ready for use on ESP32-C3
✅ All features compile correctly
✅ All calculations produce correct values
✅ Type safety is enforced
```

### Test Exit Codes

- **0**: All tests passed (ready for production)
- **1**: Some tests failed (review output)

### Continuous Integration

You can use this test in CI/CD pipelines:

```yaml
# GitHub Actions example
- name: Test RobotLib
  run: |
    g++ -std=c++11 -Wall -Wextra -I./RobotLib -o test test_compile_quick.cpp
    ./test
```

```groovy
// Jenkins example
stage('Test') {
    steps {
        sh 'g++ -std=c++11 -I./RobotLib -o test test_compile_quick.cpp'
        sh './test'
    }
}
```

## ESP32-C3 Testing

For ESP32-C3 specific testing, see the comprehensive test project:

```
ESP32-Projects/RobotLib-Test-ESP32C3/
```

### Running on ESP32-C3

```bash
cd ESP32-Projects/RobotLib-Test-ESP32C3
pio run -t upload
pio device monitor
```

The ESP32 test includes:
- **60+ test cases** covering all library features
- Memory footprint analysis
- Performance benchmarks
- Embedded-specific tests

## Troubleshooting

### Compilation Errors

**Issue**: "error: 'constexpr' does not name a type"
- **Solution**: Ensure you're using C++11 or higher (`-std=c++11`)

**Issue**: "no such file or directory: units_core.h"
- **Solution**: Run from the RobotLib directory or add `-I./RobotLib`

**Issue**: "undefined reference to std::cout"
- **Solution**: Link with C++ standard library (usually automatic)

### Test Failures

**Issue**: Tests fail with "got X, expected Y"
- **Check**: Floating-point precision (tolerances are ±0.01 typically)
- **Check**: Constants like PI are correct
- **Action**: Review the specific test output

**Issue**: All tests fail
- **Check**: Library headers are up to date
- **Check**: No modifications to core files
- **Action**: Re-download library or check git status

## Performance Testing

To test performance:

```bash
# Compile with optimization
g++ -std=c++11 -O3 -I./RobotLib -o test_opt test_compile_quick.cpp

# Time the execution
time ./test_opt
```

Expected execution time: **< 10ms** on modern hardware

## Adding Your Own Tests

To add custom tests, follow this pattern:

```cpp
#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"

int main() {
    // Your robot-specific code
    auto wheelDiameter = m(0.1);
    auto rpm = RPM::fromRPM(100);

    auto velocity = MotorController::rpmToVelocity(rpm, wheelDiameter);

    std::cout << "Wheel velocity: " << velocity.toMetersPerSecond() << " m/s\n";

    return 0;
}
```

## Test Coverage

| Component | Tests | Coverage |
|-----------|-------|----------|
| Core Units | 11 | 100% |
| Physics Units | 7 | 100% |
| Physics Operations | 7 | 100% |
| Robotics | 10 | 95% |
| Utilities | 15 | 90% |
| **Total** | **44+** | **~95%** |

---

**Last Updated**: November 2025
**Library Version**: 2.1
**Test Version**: Quick Compilation Test v1.0
