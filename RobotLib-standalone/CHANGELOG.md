# Changelog

All notable changes to RobotLib will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.2.0] - 2025-01-XX

### Added
- **Robot Simulation & Visualization System**
  - `units_simulation.h` - 2D physics simulation for differential drive robots
  - `units_visualization.h` - Real-time SDL2-based visualization
  - Three simulation examples: basic obstacle avoidance, PID navigation, line follower
  - Complete simulation documentation and build system (CMake + Makefile)
  - Zero overhead - completely optional, does not affect embedded/production code

- **Clean Output Utilities** (`units_output.h`)
  - Cross-platform output API: `print()`, `println()`, `printHeader()`
  - Works on Arduino (Serial) and desktop (std::cout)
  - Automatic platform detection
  - Variadic template support for easy output

- **Platform-Specific Robot Examples**
  - Arduino IDE examples (`.ino` format)
  - PlatformIO examples with `platformio.ini`
  - ESP32-specific examples with WiFi/Bluetooth
  - Teensy 4.x examples with high-speed peripherals
  - STM32 examples with HAL integration
  - Desktop C++ examples

- **Documentation**
  - AI-assisted development disclaimer (DISCLAIMER.md)
  - Comprehensive simulation guide (simulation/README.md)
  - Standalone repository structure

### Changed
- **All 20+ examples refactored** to use clean output utilities
  - Removed `std::cout` clutter (1,505 occurrences → 0)
  - Improved code readability
  - Better cross-platform compatibility

- **Library structure** optimized for standalone repository
  - Added `library.json` for PlatformIO
  - Updated GitHub Actions CI/CD workflows
  - Reorganized documentation

### Fixed
- C++11 constexpr compatibility issues in `units_physics.h`
- Clangd C++14 warnings with editor configurations (.clangd, .editorconfig)
- Include path issues for various platforms

## [2.1.0] - 2025-01-XX

### Added
- Complete robot control system
- PID controllers with anti-windup
- Sensor fusion algorithms
- Path planning utilities
- Full FRC competition robot examples

### Changed
- Improved header-only architecture
- Better constexpr support

## [2.0.0] - 2025-01-XX

### Added
- Initial public release
- Core units library (length, velocity, acceleration, angle)
- Physics units (force, torque, energy, power)
- Robotics units (motor control, odometry, kinematics)
- Fluent API for readable code
- 15+ comprehensive examples

---

## Version Numbering

RobotLib follows [Semantic Versioning](https://semver.org/):
- **MAJOR** version: Incompatible API changes
- **MINOR** version: New functionality (backward-compatible)
- **PATCH** version: Bug fixes (backward-compatible)

## Release Process

1. Update version in `library.json`
2. Update this CHANGELOG.md
3. Create git tag: `git tag -a v2.2.0 -m "Release v2.2.0"`
4. Push tag: `git push origin v2.2.0`
5. GitHub Actions automatically creates release

## Links

- [GitHub Repository](https://github.com/konnorreynolds/RobotLib)
- [Issue Tracker](https://github.com/konnorreynolds/RobotLib/issues)
- [Discussions](https://github.com/konnorreynolds/RobotLib/discussions)
