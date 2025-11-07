# Contributing to RobotLib

Thank you for your interest in contributing to RobotLib!

## ⚠️ Important: AI-Assisted Development

**This library was developed with significant AI assistance (Claude by Anthropic).**

When contributing, please:
- ✅ **Review all code thoroughly** - Don't assume AI-generated code is correct
- ✅ **Test extensively** - Verify changes work on target platforms
- ✅ **Add tests** - Include examples or unit tests for new features
- ✅ **Document clearly** - Explain what code does and why

## How to Contribute

### Reporting Bugs

Before creating bug reports, please check existing issues. When creating a bug report, include:

- **Description**: Clear description of the problem
- **Steps to reproduce**: Detailed steps to reproduce the issue
- **Expected behavior**: What you expected to happen
- **Actual behavior**: What actually happened
- **Environment**:
  - Platform (Arduino, ESP32, STM32, desktop, etc.)
  - Compiler version
  - RobotLib version
  - Hardware details (if applicable)
- **Code sample**: Minimal code that reproduces the issue

**Example:**
```markdown
## Bug: Motor velocity calculation incorrect on ESP32

**Environment:**
- Platform: ESP32-DevKitC
- Compiler: Arduino IDE 2.2.1 / ESP32 core 2.0.14
- RobotLib: v2.2.0

**Steps to Reproduce:**
1. Create DifferentialDrive with wheelbase 0.15m
2. Set motor PWM to 0.5, 0.5
3. Read velocity with getVelocity()

**Expected:** ~0.3 m/s
**Actual:** 0.0 m/s

**Code:**
\`\`\`cpp
#include <RobotLib.h>
// ... minimal code here
\`\`\`
```

### Suggesting Enhancements

Enhancement suggestions are welcome! Please include:

- **Clear description** of the enhancement
- **Use case**: Why is this useful?
- **Example code**: How would it be used?
- **Alternatives**: Have you considered other approaches?

### Pull Requests

1. **Fork the repository**
   ```bash
   git clone https://github.com/konnorreynolds/RobotLib.git
   cd RobotLib
   git checkout -b feature/my-enhancement
   ```

2. **Make your changes**
   - Follow existing code style
   - Maintain C++11 compatibility
   - Keep header-only design (no .cpp files in include/)
   - Add examples if adding new features
   - Update documentation

3. **Test thoroughly**
   - Test on multiple platforms if possible
   - Verify examples compile
   - Check CI/CD passes (GitHub Actions)

4. **Commit with clear messages**
   ```bash
   git commit -m "Add differential drive velocity estimation"
   ```

5. **Push and create PR**
   ```bash
   git push origin feature/my-enhancement
   ```
   Then create Pull Request on GitHub

### Code Style Guidelines

#### C++ Style

```cpp
// Use clear, descriptive names
class DifferentialDrive {  // PascalCase for classes
    double wheelRadius;     // camelCase for members

public:
    // Clear method names
    double getVelocity() const;  // camelCase for methods
    void setMotorPWM(double left, double right);
};

// Constants in UPPER_CASE
constexpr double PI = 3.14159265358979323846;
constexpr double MAX_VELOCITY = 2.0;  // m/s

// Use const and constexpr appropriately
constexpr double calculateCircumference(double radius) {
    return 2.0 * PI * radius;
}
```

#### Header-Only Requirements

- All implementations must be in headers
- Use `inline` or `constexpr` for functions
- No `.cpp` files in `include/` directory
- Template implementations in same header

#### Documentation

Use clear comments:

```cpp
/**
 * Calculate wheel velocity from encoder counts
 *
 * @param encoderCount Change in encoder ticks
 * @param dt Time interval (seconds)
 * @param countsPerMeter Encoder resolution (ticks/meter)
 * @return Velocity in m/s
 */
inline double calculateVelocity(long encoderCount, double dt, double countsPerMeter) {
    return (encoderCount / countsPerMeter) / dt;
}
```

#### Platform Compatibility

Ensure code works on all platforms:

```cpp
// ✅ Good - Platform-agnostic
#include <cmath>
double distance = std::sqrt(dx*dx + dy*dy);

// ❌ Bad - Desktop-only
#include <iostream>
std::cout << "Value: " << value << std::endl;

// ✅ Good - Use output utilities
#include "units_output.h"
println("Value: ", value);
```

### Example Template

When adding examples, use this structure:

```cpp
// ============================================================================
// Example: Brief Description
// ============================================================================
// Purpose: What this example demonstrates
//
// Hardware:
// - Platform (Arduino/ESP32/STM32/Desktop)
// - Required components
//
// Demonstrates:
// - Feature 1
// - Feature 2
// ============================================================================

#include <RobotLib.h>

using namespace robotlib;
using namespace robotlib::output;

int main() {
    printHeader("Example Name");
    println("Description of what this does");
    println();

    // Example code here

    println();
    println("✓ Example completed successfully!");

    return 0;
}
```

### Testing Checklist

Before submitting PR, verify:

- [ ] Code compiles with C++11, C++14, C++17
- [ ] Examples compile and run correctly
- [ ] Works on at least 2 platforms (if applicable)
- [ ] Documentation updated (README, examples, comments)
- [ ] CHANGELOG.md updated
- [ ] No new compiler warnings
- [ ] CI/CD tests pass
- [ ] Code follows style guidelines

### Platforms to Test

If possible, test on:

**Embedded:**
- Arduino (Uno, Mega, Nano)
- ESP32
- Teensy 4.x
- STM32 (Nucleo boards)

**Desktop:**
- Linux (Ubuntu 22.04+)
- macOS (13+)
- Windows (with MinGW or MSVC)

### What to Contribute

**High Priority:**
- Bug fixes for existing features
- Platform compatibility improvements
- More real-world robot examples
- Performance optimizations
- Documentation improvements

**Medium Priority:**
- New sensor/actuator support
- Additional algorithms (path planning, SLAM, etc.)
- Visualization improvements
- Unit tests

**Please Discuss First:**
- Major API changes
- New dependencies
- Breaking changes
- Large refactoring

## Development Setup

### Prerequisites

```bash
# Ubuntu/Debian
sudo apt-get install build-essential cmake git
sudo apt-get install libsdl2-dev  # For simulation only

# macOS
brew install cmake git sdl2

# Windows
# Install MinGW or Visual Studio
# Download SDL2 from libsdl.org
```

### Building Examples

```bash
# Clone repository
git clone https://github.com/konnorreynolds/RobotLib.git
cd RobotLib

# Build simulation examples
cd simulation
mkdir build && cd build
cmake ..
make

# Run examples
./basic_simulation
./pid_navigation
./line_follower
```

### Running Tests

```bash
# Compile all examples
cd examples
for dir in */; do
    cd "$dir"
    for file in *.cpp; do
        g++ -std=c++11 -I../../include "$file" -o test || exit 1
    done
    cd ..
done
```

## Code of Conduct

### Our Standards

- **Be respectful** - Treat everyone with respect
- **Be constructive** - Focus on improving the library
- **Be patient** - Remember this is AI-assisted; mistakes happen
- **Be helpful** - Help others learn and contribute

### Unacceptable Behavior

- Harassment or discrimination
- Trolling or insulting comments
- Publishing others' private information
- Other unprofessional conduct

## Questions?

- **General questions**: Use [GitHub Discussions](https://github.com/konnorreynolds/RobotLib/discussions)
- **Bug reports**: Use [GitHub Issues](https://github.com/konnorreynolds/RobotLib/issues)
- **Security issues**: Email konnorreynolds directly (see GitHub profile)

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

## Recognition

Contributors will be recognized in:
- README.md (if significant contribution)
- Release notes
- Git commit history

Thank you for contributing to RobotLib! 🤖✨
