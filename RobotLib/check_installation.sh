#!/bin/bash

# RobotLib Installation Diagnostic Script
# Run this to check if RobotLib is properly installed

echo "╔══════════════════════════════════════════════════════════╗"
echo "║        RobotLib Installation Diagnostic Tool             ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

ERRORS=0

# Check 1: Main header exists
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 1: Checking for RobotLib.h"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ -f "include/RobotLib.h" ]; then
    echo "✅ Found: include/RobotLib.h"
else
    echo "❌ Missing: include/RobotLib.h"
    echo "   Are you running this from the RobotLib directory?"
    ERRORS=$((ERRORS + 1))
fi

# Check 2: Core headers exist
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 2: Checking for required headers"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

REQUIRED_HEADERS=(
    "units_core.h"
    "units_physics.h"
    "units_robotics.h"
    "units_3d.h"
    "units_utilities.h"
    "robotlib_api.h"
)

MISSING=0
for header in "${REQUIRED_HEADERS[@]}"; do
    if [ -f "include/$header" ]; then
        echo "✅ Found: $header"
    else
        echo "❌ Missing: $header"
        MISSING=$((MISSING + 1))
    fi
done

if [ $MISSING -gt 0 ]; then
    echo "❌ Missing $MISSING required headers!"
    ERRORS=$((ERRORS + 1))
fi

# Check 3: Test compilation
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 3: Compilation test"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Create test file
cat > /tmp/robotlib_diagnostic_test.cpp << 'EOF'
#include <RobotLib.h>
#include <iostream>

int main() {
    std::cout << "RobotLib v" << robotlib::VERSION_STRING << std::endl;

    using namespace units;
    auto distance = m(10.0);
    auto velocity = mps(2.5);

    robotics::PIDController pid(1.0, 0.1, 0.05);
    double output = pid.calculate(10.0, 5.0, 0.02);

    std::cout << "All features working!" << std::endl;
    return 0;
}
EOF

echo "Compiling test program..."
if g++ -std=c++11 -I./include /tmp/robotlib_diagnostic_test.cpp -o /tmp/robotlib_diagnostic_test 2>/tmp/robotlib_diagnostic_errors.txt; then
    echo "✅ Compilation successful!"

    # Run the test
    echo ""
    echo "Running test program..."
    /tmp/robotlib_diagnostic_test

    # Clean up
    rm -f /tmp/robotlib_diagnostic_test /tmp/robotlib_diagnostic_test.cpp /tmp/robotlib_diagnostic_errors.txt
else
    echo "❌ Compilation failed!"
    echo ""
    echo "Error output:"
    cat /tmp/robotlib_diagnostic_errors.txt
    echo ""
    ERRORS=$((ERRORS + 1))
    rm -f /tmp/robotlib_diagnostic_test.cpp /tmp/robotlib_diagnostic_errors.txt
fi

# Check 4: Documentation
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 4: Documentation files"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

DOCS=("INSTALL.md" "USAGE.md" "TROUBLESHOOTING.md")
for doc in "${DOCS[@]}"; do
    if [ -f "$doc" ]; then
        echo "✅ Found: $doc"
    else
        echo "⚠️  Missing: $doc (optional)"
    fi
done

# Final summary
echo ""
echo "╔══════════════════════════════════════════════════════════╗"
if [ $ERRORS -eq 0 ]; then
    echo "║                  ✅ ALL TESTS PASSED!                    ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo ""
    echo "RobotLib is properly installed and ready to use!"
    echo ""
    echo "Quick start:"
    echo "  1. Copy RobotLib folder to your project: cp -r RobotLib /path/to/project/lib/"
    echo "  2. Add to your code: #include <RobotLib.h>"
    echo "  3. Compile with: g++ -std=c++11 -I/path/to/RobotLib/include your_code.cpp"
    echo ""
    echo "See INSTALL.md and USAGE.md for detailed instructions."
else
    echo "║              ❌ $ERRORS TEST(S) FAILED                      ║"
    echo "╚══════════════════════════════════════════════════════════╝"
    echo ""
    echo "Please fix the errors above."
    echo "See TROUBLESHOOTING.md for common solutions."
fi
echo ""

exit $ERRORS
