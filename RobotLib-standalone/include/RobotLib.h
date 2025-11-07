// ============================================================================
// RobotLib - Complete Robotics Library with Type-Safe Units
// ============================================================================
// Main header file - Include this to access all RobotLib functionality
//
// Features:
// - Type-safe physical units (compile-time dimensional analysis)
// - Robotics algorithms (kinematics, control, planning)
// - State estimation (EKF, filters)
// - 3D math (quaternions, rotations)
// - Planning algorithms (A*, RRT, Dubins)
// - Advanced control (PID, MPC, LQR)
// - Fluent API for easy usage
//
// Usage:
//   #include <RobotLib.h>
//   using namespace units;
//
//   auto distance = m(5.0);
//   auto velocity = mps(2.0);
//   auto time = distance / velocity;  // Type-safe!
//
// Version: 2.2
// C++ Standard: C++11 or newer
// License: [Your License]
// ============================================================================

#ifndef ROBOTLIB_H
#define ROBOTLIB_H

// ============================================================================
// Core Module (Required - Base types and unit system)
// ============================================================================
#include "units_core.h"

// ============================================================================
// Physics Module (Physical quantities and laws)
// ============================================================================
#include "units_physics.h"

// ============================================================================
// Robotics Module (Control algorithms, filters, vectors)
// ============================================================================
#include "units_robotics.h"

// ============================================================================
// 3D Math Module (Quaternions, 3D vectors, rotations)
// ============================================================================
#include "units_3d.h"

// ============================================================================
// Math Utilities Module (Interpolation, statistics, numerical methods)
// ============================================================================
#include "units_math.h"

// ============================================================================
// Utilities Module (Motor control, odometry, conversions)
// ============================================================================
#include "units_utilities.h"

// ============================================================================
// Output Module (Clean logging and printing for examples)
// ============================================================================
#include "units_output.h"

// ============================================================================
// Estimation Module (Kalman filters, state estimation)
// ============================================================================
#include "units_estimation.h"

// ============================================================================
// Planning Module (Path planning, A*, RRT, Dubins)
// ============================================================================
#include "units_planning.h"

// ============================================================================
// Control Module (MPC, LQR, advanced control)
// ============================================================================
#include "units_control.h"

// ============================================================================
// Fluent API Module (High-level chainable interface)
// ============================================================================
#include "robotlib_api.h"

// ============================================================================
// Optional: Interop Modules (Uncomment if needed)
// ============================================================================
// Uncomment these if you need ROS2 or MATLAB integration:
// #include "units_ros2_interop.h"
// #include "units_matlab_interop.h"

// ============================================================================
// Version Information
// ============================================================================
namespace robotlib {
    constexpr int VERSION_MAJOR = 2;
    constexpr int VERSION_MINOR = 2;
    constexpr int VERSION_PATCH = 0;

    constexpr const char* VERSION_STRING = "2.2.0";
    constexpr const char* BUILD_DATE = __DATE__;
    constexpr const char* BUILD_TIME = __TIME__;
}

// ============================================================================
// Quick Start Guide
// ============================================================================
/*

QUICK START:

1. Include the library:
   #include <RobotLib.h>

2. Use type-safe units:
   auto distance = m(10.0);          // 10 meters
   auto speed = mps(2.5);            // 2.5 m/s
   auto time = distance / speed;     // Automatically 4 seconds

3. Vector math:
   Vec2D position(1.0, 2.0);
   Vec2D velocity(0.5, -0.3);
   double speed = velocity.magnitude();

4. PID Control:
   PIDController pid(1.0, 0.1, 0.05);  // kP, kI, kD
   double output = pid.calculate(setpoint, current, dt);

5. Motion Profiling:
   TrapezoidProfile profile(maxVel, maxAccel);
   auto state = profile.calculate(target, current, dt);

6. Fluent API (Easy chaining):
   Arm myArm = Arm()
       .withPID(1.5, 0.1, 0.05)
       .withLimits(deg(-90), deg(90))
       .withFeedforward(0.3, 0.002);

   myArm.moveTo(deg(45));
   myArm.update(dt);

7. State Estimation:
   ExtendedKalmanFilter<4, 2> ekf(...);
   ekf.predict(dt);
   ekf.update(measurement);

8. Path Planning:
   AStarPlanner planner(width, height);
   auto path = planner.plan(start, goal);

For more examples, see:
- RobotLib/examples/
- RobotLib/tests/

For documentation, see header comments in each module.

*/

#endif // ROBOTLIB_H
