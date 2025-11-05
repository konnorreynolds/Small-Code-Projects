# 🤝 Contributing to RobotLib

Thank you for your interest in contributing to RobotLib! This document provides guidelines and information for contributors.

## 🎯 Ways to Contribute

### 1. 🐛 Report Bugs
Found a bug? Please open an issue with:
- Clear description of the problem
- Steps to reproduce
- Expected vs actual behavior
- Your environment (compiler, OS, C++ version)
- Minimal code example demonstrating the issue

### 2. 💡 Suggest Features
Have an idea? We'd love to hear it! Open an issue describing:
- The feature you'd like to see
- Why it would be useful
- How it might work
- Any implementation ideas

### 3. 📝 Improve Documentation
Documentation improvements are always welcome:
- Fix typos or clarify explanations
- Add more examples
- Improve code comments
- Translate documentation

### 4. 🔧 Submit Code
Ready to code? Great! Please follow the guidelines below.

---

## 🚀 Getting Started

### 1. Fork and Clone
```bash
# Fork the repository on GitHub, then:
git clone https://github.com/YOUR_USERNAME/Small-Code-Projects.git
cd Small-Code-Projects/RobotLib
```

### 2. Create a Branch
```bash
git checkout -b feature/your-feature-name
# or
git checkout -b fix/your-bug-fix
```

### 3. Make Your Changes
Follow our coding standards (see below)

### 4. Test Your Changes
```bash
# Compile and run tests
g++ -std=c++11 -Wall -Wextra -Wpedantic -Werror -O2 test_compile_quick.cpp -o test_quick
./test_quick

g++ -std=c++11 -Wall -Wextra -Wpedantic -Werror -O2 test_math_features.cpp -o test_math
./test_math
```

### 5. Commit and Push
```bash
git add .
git commit -m "Brief description of changes"
git push origin feature/your-feature-name
```

### 6. Open a Pull Request
- Go to the original repository
- Click "New Pull Request"
- Select your branch
- Fill out the PR template

---

## 📋 Coding Standards

### C++ Version
- **Must be C++11 compatible**
- No C++14/17/20 features unless behind feature detection
- Test on multiple compilers if possible (GCC, Clang, MSVC)

### Code Style

#### Naming Conventions
```cpp
// Classes: PascalCase
class PIDController { };

// Functions: camelCase
double calculateValue();

// Variables: camelCase
double velocityError;

// Constants: UPPER_SNAKE_CASE
constexpr double MAX_VELOCITY = 10.0;

// Template parameters: PascalCase
template<typename Derived, typename Ratio>

// Namespaces: lowercase
namespace units { }
```

#### Formatting
```cpp
// Use 4 spaces for indentation (no tabs)
class Example {
public:
    void function() {
        if (condition) {
            // code
        }
    }
};

// Opening brace on same line
void function() {
    // code
}

// Space after control keywords
if (condition) { }
for (int i = 0; i < n; i++) { }
while (condition) { }

// No space before function call parentheses
function(arg1, arg2);
```

#### Comments
```cpp
// Use // for single-line comments
// Prefer explaining WHY, not WHAT

// Good: Explains reasoning
// Reverse speed to avoid >90° rotation (faster)
speed = -speed;

// Bad: Just describes code
// Negate speed
speed = -speed;

// Use /** */ for documentation blocks
/**
 * Calculate PID output
 * @param error Current error value
 * @param dt Time step in seconds
 * @return Control output
 */
double calculate(double error, double dt);
```

### Design Principles

#### 1. Zero-Overhead Abstraction
```cpp
// Prefer compile-time computation
constexpr double calculate() { return 2.0 * PI; }

// Use inline for small functions
inline double square(double x) { return x * x; }

// Avoid runtime polymorphism (virtual functions)
// Use CRTP instead
```

#### 2. Type Safety
```cpp
// Strong typing prevents errors
Meters distance = m(5.0);
Seconds time = s(2.0);
auto velocity = distance / time;  // Type: MetersPerSecond

// This won't compile (good!)
// auto bad = distance + time;  // Can't add distance to time!
```

#### 3. Const Correctness
```cpp
// Mark non-modifying functions as const
class Vector {
    double magnitude() const { return /* ... */; }
};

// Use const references for parameters
void process(const Vector& v);
```

#### 4. Header-Only Library
```cpp
// All code in headers for simplicity
// Mark functions inline or constexpr
inline double helper() { return 42.0; }
constexpr double constant() { return 3.14; }
```

---

## 🧪 Testing Requirements

### For New Features
1. Add tests to `test_compile_quick.cpp`
2. Ensure all existing tests still pass
3. Add example code demonstrating usage
4. Update documentation

### Test Coverage
- Test basic functionality
- Test edge cases (zero, negative, large values)
- Test type safety (compilation failures expected)
- Test all unit conversions

### Example Test
```cpp
// In test_compile_quick.cpp
ASSERT_APPROX("Feature name", actualValue, expectedValue, tolerance);
```

---

## 📚 Documentation Requirements

### For New Features
1. **Code Comments**: Explain complex algorithms
2. **Header Comments**: Describe purpose and usage
3. **Example Code**: Add to examples/ directory
4. **README Update**: Update feature list
5. **Changelog**: Add entry to docs/CHANGES.md

### Documentation Style
- Use clear, simple language
- Provide code examples
- Explain the "why" not just the "how"
- Include real-world use cases

---

## 🎨 Example Contributions

### Adding a New Unit Type
```cpp
// 1. Define the unit in units_physics.h
template<typename Ratio = std::ratio<1>>
using Pressure = /* ... */;

// 2. Add helper functions
inline Pressure<> pascals(double value) { return Pressure<>(value); }
inline Pressure<std::ratio<1000>> kilopascals(double value) { /* ... */ }

// 3. Add tests
ASSERT_APPROX("100 kPa to Pa", kpa(100).toPascals(), 100000, 0.1);

// 4. Add example usage in examples/
// 5. Update documentation
```

### Adding a New Example
```cpp
// 1. Create examples/XX_example_name.cpp
// 2. Follow existing example structure
// 3. Add comprehensive comments
// 4. Include expected output
// 5. Add to examples/README.md
// 6. Test compilation with strict flags
```

---

## ✅ Pull Request Checklist

Before submitting, ensure:
- [ ] Code compiles with `-std=c++11 -Wall -Wextra -Wpedantic -Werror`
- [ ] All tests pass (184/184)
- [ ] New tests added for new features
- [ ] Code follows style guidelines
- [ ] Documentation updated
- [ ] Examples added/updated if needed
- [ ] Changelog updated (docs/CHANGES.md)
- [ ] No warnings generated
- [ ] Works on embedded platforms (if applicable)

---

## 🚫 What We Don't Accept

- C++14/17/20-only features without C++11 fallback
- Dynamic memory allocation (malloc, new)
- Exceptions for control flow
- Virtual functions (use CRTP instead)
- Platform-specific code without guards
- Breaking changes without major version bump
- Undocumented features
- Code without tests

---

## 💬 Communication

### Getting Help
- Open an issue for questions
- Check existing issues first
- Be respectful and patient

### Code Review Process
1. Maintainer reviews your PR
2. Feedback provided (if needed)
3. You make requested changes
4. Once approved, PR is merged
5. Your contribution is celebrated! 🎉

---

## 📄 License

By contributing, you agree that your contributions will be licensed under the same license as the project (MIT License).

---

## 🙏 Recognition

All contributors will be:
- Listed in the project README
- Credited in release notes
- Part of the RobotLib community!

---

## 🎓 Learning Resources

### C++ Best Practices
- [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)
- [Effective C++ by Scott Meyers](https://www.aristeia.com/books.html)

### Template Metaprogramming
- [Modern C++ Design by Andrei Alexandrescu](https://en.wikipedia.org/wiki/Modern_C%2B%2B_Design)

### Robotics
- [FRC Documentation](https://docs.wpilib.org/)
- [Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)

---

## 📞 Contact

Questions? Suggestions? Found this helpful?
- Open an issue on GitHub
- Tag us in discussions
- Share your robot projects using RobotLib!

---

**Thank you for contributing to RobotLib! Together we're building better robotics software! 🤖✨**
