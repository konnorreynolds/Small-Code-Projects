# Library Registry Submission Guide

This guide explains how to submit RobotLib to various package registries.

## ⚠️ Important Disclaimers

When submitting to registries, **ALWAYS include the AI-assisted development disclaimer**:

> **This library was developed with significant AI assistance (Claude by Anthropic). Users should review and test code thoroughly before use in production systems. Use at your own discretion and risk. See DISCLAIMER.md for full details.**

## Supported Registries

1. **PlatformIO Registry** ⭐ Recommended
2. **Arduino Library Manager** ⭐ Recommended
3. **GitHub Releases** (Automatic via GitHub Actions)

---

## 1. PlatformIO Registry

### Prerequisites

- `library.json` file (✅ already included)
- GitHub repository published
- Valid semantic version tag

### Submission Process

#### Option A: Automatic (Recommended)

1. **Publish to GitHub**
   ```bash
   git tag -a v2.2.0 -m "Release v2.2.0"
   git push origin v2.2.0
   ```

2. **Register with PlatformIO**

   PlatformIO automatically indexes public GitHub repositories with `library.json`.

   - Wait 24-48 hours for automatic indexing
   - Check at: https://registry.platformio.org/search?q=RobotLib

3. **Manual registration** (if not auto-indexed):
   ```bash
   pio pkg publish
   ```

#### Option B: Manual Submission

1. Create account at https://platformio.org
2. Go to https://registry.platformio.org/
3. Click "Publish Library"
4. Enter GitHub URL: `https://github.com/konnorreynolds/RobotLib`
5. Wait for validation

### Verification

After submission, verify listing:
```bash
pio pkg search RobotLib
pio pkg show konnorreynolds/RobotLib
```

Users can then install with:
```bash
pio pkg install --library "konnorreynolds/RobotLib@^2.2.0"
```

### Updating

New versions are automatically detected when you push new tags:
```bash
git tag -a v2.3.0 -m "Release v2.3.0"
git push origin v2.3.0
# Wait 24-48 hours for automatic update
```

---

## 2. Arduino Library Manager

### Prerequisites

Arduino Library Manager requires specific structure:

- `library.properties` file (needs to be created)
- Examples in `examples/` folder (✅ already have)
- `LICENSE` file (✅ already have)
- Public GitHub repository

### Setup

1. **Create `library.properties`**

   Add this file to repository root:

   ```properties
   name=RobotLib
   version=2.2.0
   author=Konnor Reynolds <konnorreynolds@github>
   maintainer=Konnor Reynolds <konnorreynolds@github>
   sentence=Type-safe units library for robotics control systems
   paragraph=Header-only C++11 library providing type-safe units (length, velocity, angle, force, torque) with zero runtime overhead. Includes PID controllers, differential drive, odometry, and complete robot examples. AI-assisted development - use at your own discretion.
   category=Device Control
   url=https://github.com/konnorreynolds/RobotLib
   architectures=*
   includes=RobotLib.h
   depends=
   ```

   **Important**: Include AI disclaimer in `paragraph` field!

2. **Verify structure**

   Arduino expects:
   ```
   RobotLib/
   ├── library.properties
   ├── LICENSE
   ├── README.md
   ├── examples/
   │   ├── BasicMotor/
   │   │   └── BasicMotor.ino
   │   └── ...
   └── src/
       └── RobotLib.h  (or include/)
   ```

   **Current structure uses `include/`** - may need to create symlink:
   ```bash
   ln -s include src
   ```
   Or add `src/` directory with single header that includes from `include/`:
   ```cpp
   // src/RobotLib.h
   #include "../include/RobotLib.h"
   ```

3. **Create GitHub Release**
   ```bash
   git tag -a v2.2.0 -m "Release v2.2.0"
   git push origin v2.2.0
   ```

### Submission Process

1. **Fork Arduino Library Registry**

   Go to: https://github.com/arduino/library-registry

2. **Add your library**

   Edit `repositories.txt` and add:
   ```
   https://github.com/konnorreynolds/RobotLib
   ```

3. **Create Pull Request**

   - Branch name: `add-robotlib`
   - PR title: "Add RobotLib - Type-safe units for robotics"
   - PR description:
     ```markdown
     ## Library: RobotLib

     **Author**: Konnor Reynolds
     **Repository**: https://github.com/konnorreynolds/RobotLib
     **Version**: 2.2.0

     ### Description
     Type-safe C++11 units library for robotics control systems. Provides length, velocity, angle, force, torque units with zero runtime overhead. Includes PID controllers, differential drive, and complete robot examples.

     ⚠️ **Important**: This library was developed with AI assistance. Users should review code before production use.

     ### Checklist
     - [x] library.properties file exists
     - [x] Valid semantic versioning
     - [x] Examples included
     - [x] MIT License
     - [x] Public repository
     - [x] Compiles with Arduino IDE
     ```

4. **Wait for review**

   Arduino team reviews PRs. May take 1-2 weeks.

### Updating

For updates, just create new GitHub releases:
```bash
# Update library.properties version first!
git tag -a v2.3.0 -m "Release v2.3.0"
git push origin v2.3.0
```

Arduino Library Manager automatically detects new releases.

---

## 3. GitHub Releases (Automated)

### Setup

GitHub Actions workflow (`.github/workflows/release.yml`) automatically:
- Creates releases when you push version tags
- Generates source archives (.tar.gz and .zip)
- Extracts changelog
- Publishes release on GitHub

### Usage

```bash
# 1. Update version in library.json and library.properties
# 2. Update CHANGELOG.md
# 3. Commit changes
git add library.json library.properties CHANGELOG.md
git commit -m "Bump version to 2.3.0"

# 4. Create and push tag
git tag -a v2.3.0 -m "Release v2.3.0"
git push origin main
git push origin v2.3.0

# 5. GitHub Actions automatically creates release!
```

### Verify

Check releases at: `https://github.com/konnorreynolds/RobotLib/releases`

---

## Version Numbering

Follow [Semantic Versioning](https://semver.org/):

- **MAJOR** (v3.0.0): Breaking API changes
  - Example: Removing public methods, changing function signatures

- **MINOR** (v2.3.0): New features, backward-compatible
  - Example: Adding new unit types, new examples

- **PATCH** (v2.2.1): Bug fixes, backward-compatible
  - Example: Fixing calculation errors, documentation typos

---

## Pre-Submission Checklist

Before submitting to any registry:

### Code Quality
- [ ] All examples compile successfully
- [ ] Works on Arduino, ESP32, STM32, desktop
- [ ] No compiler warnings with `-Wall -Wextra`
- [ ] C++11, C++14, C++17 compatibility verified
- [ ] CI/CD tests pass (GitHub Actions)

### Documentation
- [ ] README.md updated with new features
- [ ] **AI disclaimer prominently displayed**
- [ ] CHANGELOG.md updated with version changes
- [ ] Examples have clear comments
- [ ] API documentation complete

### Repository
- [ ] LICENSE file present (MIT)
- [ ] DISCLAIMER.md included
- [ ] .gitignore properly configured
- [ ] No unnecessary files in repository
- [ ] Clean git history

### Metadata
- [ ] library.json version matches git tag
- [ ] library.properties version matches (for Arduino)
- [ ] CHANGELOG.md version matches
- [ ] Git tag created: `vX.Y.Z`

### Testing
- [ ] Compile test on at least 3 platforms
- [ ] Run at least 5 examples successfully
- [ ] Verify simulation examples work (if SDL2 available)
- [ ] Check memory usage reasonable for embedded

---

## Registry-Specific Requirements

### PlatformIO
✅ Requirements:
- `library.json` with valid fields
- Semantic version tag (vX.Y.Z)
- Public GitHub repository

⚠️ Recommendations:
- Add keywords in library.json
- Include good README with examples
- Respond to issues promptly

### Arduino
✅ Requirements:
- `library.properties` file
- Proper folder structure (src/ or include/)
- Examples in examples/ folder
- Semantic versioning

⚠️ Recommendations:
- Keep library size small (<1MB)
- Test on actual Arduino hardware
- Follow Arduino Style Guide
- Include .ino examples

---

## Maintaining Multiple Registries

### Workflow

1. **Develop locally**
   ```bash
   # Make changes, test thoroughly
   ```

2. **Update metadata**
   ```bash
   # Edit library.json version: "2.3.0"
   # Edit library.properties version: 2.3.0
   # Update CHANGELOG.md
   ```

3. **Commit and tag**
   ```bash
   git add .
   git commit -m "Release v2.3.0: Add new feature"
   git tag -a v2.3.0 -m "Release v2.3.0"
   ```

4. **Push**
   ```bash
   git push origin main
   git push origin v2.3.0
   ```

5. **Automatic updates**
   - GitHub Releases: Immediate (GitHub Actions)
   - PlatformIO: 24-48 hours (auto-indexed)
   - Arduino: Next manual sync (or immediate if urgent)

---

## Troubleshooting

### PlatformIO Issues

**Problem**: Library not appearing after 48 hours

**Solution**:
```bash
# Manual registration
pio pkg publish
# Or contact PlatformIO support
```

**Problem**: Version not updating

**Solution**:
- Verify git tag is semantic version (vX.Y.Z)
- Check library.json version matches tag
- Wait 48 hours before re-checking

### Arduino Issues

**Problem**: Library rejected during review

**Common reasons**:
- Missing library.properties
- Invalid folder structure
- Examples don't compile
- Missing LICENSE

**Solution**: Fix issues and update PR

**Problem**: Examples not showing in Arduino IDE

**Solution**:
- Ensure examples/ folder structure correct
- Each example must be in own folder
- .ino file must match folder name
  ```
  examples/BasicMotor/BasicMotor.ino  ✅
  examples/BasicMotor/example.ino     ❌
  ```

---

## Support

### Getting Help

- **PlatformIO**: https://community.platformio.org/
- **Arduino**: https://forum.arduino.cc/
- **GitHub Issues**: For library-specific problems

### Monitoring

Check these regularly:
- GitHub Issues: https://github.com/konnorreynolds/RobotLib/issues
- PlatformIO: https://registry.platformio.org/libraries/konnorreynolds/RobotLib
- Arduino: Search "RobotLib" in Arduino IDE Library Manager

---

## Next Steps

1. **Create library.properties** (for Arduino support)
2. **Test on real hardware** (Arduino, ESP32, etc.)
3. **Submit to PlatformIO** (automatic after tagging)
4. **Submit to Arduino** (manual PR to library-registry)
5. **Announce release** (GitHub Discussions, social media)
6. **Monitor feedback** (respond to issues and questions)

---

## Resources

- [PlatformIO Registry](https://docs.platformio.org/en/latest/librarymanager/index.html)
- [Arduino Library Specification](https://arduino.github.io/arduino-cli/latest/library-specification/)
- [Semantic Versioning](https://semver.org/)
- [Keep a Changelog](https://keepachangelog.com/)

---

**Remember**: Always include AI disclaimer in registry submissions! 🤖⚠️
