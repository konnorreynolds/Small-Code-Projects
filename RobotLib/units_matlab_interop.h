// ============================================================================
// RobotLib - MATLAB/Simulink Interoperability
// ============================================================================
// Helper functions for interfacing RobotLib with MATLAB and Simulink.
// Provides code generation helpers and data conversion utilities.
//
// Features:
// - Convert RobotLib units to/from MATLAB doubles
// - Generate Simulink-compatible C code
// - Export data to MATLAB-readable formats
// - Type-safe wrappers for Simulink Coder
//
// Usage Scenarios:
// 1. Exporting robot data to MATLAB for analysis
// 2. Integrating RobotLib controllers in Simulink models
// 3. Hardware-in-the-loop (HIL) testing
// 4. Rapid control prototyping
//
// Part of RobotLib v2.2
// C++11 compatible, header-only, zero-overhead design
// ============================================================================

#ifndef ROBOTLIB_UNITS_MATLAB_INTEROP_H
#define ROBOTLIB_UNITS_MATLAB_INTEROP_H

#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"
#include <array>
#include <cstdio>

namespace units {
namespace matlab {

// ============================================================================
// Unit Conversion Helpers for MATLAB
// ============================================================================
// MATLAB uses doubles for all numeric data. These functions convert
// RobotLib units to/from MATLAB-compatible doubles with explicit units.

// Distance conversions
inline double toMatlabMeters(const Meters& m) { return m.toMeters(); }
inline Meters fromMatlabMeters(double m) { return Meters::fromMeters(m); }

// Angle conversions
inline double toMatlabRadians(const Radians& rad) { return rad.toRadians(); }
inline Radians fromMatlabRadians(double rad) { return Radians::fromRadians(rad); }

inline double toMatlabDegrees(const Radians& rad) { return rad.toDegrees(); }
inline Radians fromMatlabDegrees(double deg) {
    return Radians::fromRadians(deg * M_PI / 180.0);
}

// Velocity conversions
inline double toMatlabMetersPerSecond(const MetersPerSecond& mps) {
    return mps.toMetersPerSecond();
}
inline MetersPerSecond fromMatlabMetersPerSecond(double mps) {
    return MetersPerSecond::fromMetersPerSecond(mps);
}

inline double toMatlabRadiansPerSecond(const RadiansPerSecond& radps) {
    return radps.toRadiansPerSecond();
}
inline RadiansPerSecond fromMatlabRadiansPerSecond(double radps) {
    return RadiansPerSecond::fromRadiansPerSecond(radps);
}

// Acceleration conversions
inline double toMatlabMetersPerSecondSquared(const MetersPerSecondSquared& a) {
    return a.toMetersPerSecondSquared();
}
inline MetersPerSecondSquared fromMatlabMetersPerSecondSquared(double a) {
    return MetersPerSecondSquared::fromMetersPerSecondSquared(a);
}

// Time conversions
inline double toMatlabSeconds(const Seconds& s) { return s.toSeconds(); }
inline Seconds fromMatlabSeconds(double s) { return Seconds::fromSeconds(s); }

// ============================================================================
// Data Export Functions
// ============================================================================
// Export robot data to MATLAB-readable text formats

// Export trajectory to CSV (MATLAB can import with csvread)
template<size_t N>
inline bool exportTrajectoryToCSV(const char* filename,
                                   const std::array<double, N>& time,
                                   const std::array<Meters, N>& position,
                                   const std::array<MetersPerSecond, N>& velocity) {
    FILE* f = fopen(filename, "w");
    if (!f) return false;

    fprintf(f, "time,position,velocity\n");
    for (size_t i = 0; i < N; ++i) {
        fprintf(f, "%.6f,%.6f,%.6f\n",
                time[i],
                position[i].toMeters(),
                velocity[i].toMetersPerSecond());
    }

    fclose(f);
    return true;
}

// Export 2D path to CSV
template<size_t N>
inline bool exportPath2DToCSV(const char* filename,
                               const std::array<Vec2D, N>& waypoints) {
    FILE* f = fopen(filename, "w");
    if (!f) return false;

    fprintf(f, "x,y\n");
    for (size_t i = 0; i < N; ++i) {
        fprintf(f, "%.6f,%.6f\n", waypoints[i].x(), waypoints[i].y());
    }

    fclose(f);
    return true;
}

// Export PID tuning data
inline bool exportPIDTuningData(const char* filename,
                                const std::array<double, 100>& time,
                                const std::array<double, 100>& setpoint,
                                const std::array<double, 100>& measured,
                                const std::array<double, 100>& control) {
    FILE* f = fopen(filename, "w");
    if (!f) return false;

    fprintf(f, "time,setpoint,measured,control\n");
    for (size_t i = 0; i < 100; ++i) {
        fprintf(f, "%.6f,%.6f,%.6f,%.6f\n",
                time[i], setpoint[i], measured[i], control[i]);
    }

    fclose(f);
    return true;
}

// ============================================================================
// Simulink Coder Compatible Functions
// ============================================================================
// These functions follow Simulink Coder conventions for embedded code gen

// Simulink-style state structure for PID controller
struct SimulinkPIDState {
    double integral;
    double prev_error;
    double prev_derivative;
};

// Simulink-style PID step function
// Matches Simulink block interface: (input, state) -> output
inline double simulinkPIDStep(double error,
                               double dt,
                               double kP,
                               double kI,
                               double kD,
                               SimulinkPIDState* state) {
    // Proportional term
    double P = kP * error;

    // Integral term with anti-windup
    state->integral += error * dt;
    if (state->integral > 10.0) state->integral = 10.0;
    if (state->integral < -10.0) state->integral = -10.0;
    double I = kI * state->integral;

    // Derivative term with filtering (alpha = 0.1)
    double raw_derivative = (error - state->prev_error) / dt;
    double derivative = 0.1 * raw_derivative + 0.9 * state->prev_derivative;
    double D = kD * derivative;

    // Update state
    state->prev_error = error;
    state->prev_derivative = derivative;

    // Return control output
    return P + I + D;
}

// Simulink-style complementary filter
struct SimulinkCompFilterState {
    double angle;
};

inline double simulinkCompFilterStep(double gyro_rate,
                                      double accel_angle,
                                      double dt,
                                      double alpha,
                                      SimulinkCompFilterState* state) {
    // Complementary filter: alpha*gyro + (1-alpha)*accel
    double gyro_angle = state->angle + gyro_rate * dt;
    state->angle = alpha * gyro_angle + (1.0 - alpha) * accel_angle;
    return state->angle;
}

// ============================================================================
// MATLAB MEX Function Helpers
// ============================================================================
// Helper macros and functions for creating MEX functions

// Check number of inputs (for MEX functions)
#define MATLAB_CHECK_INPUTS(nrhs, expected) \
    if (nrhs != expected) { \
        return; /* In MEX, would call mexErrMsgTxt */ \
    }

// Extract scalar from MATLAB (placeholder, real MEX uses mxGetScalar)
inline double extractMatlabScalar(const void* prhs, int index) {
    (void)prhs;  // Suppress unused warning
    (void)index;
    return 0.0;  // Placeholder
}

// ============================================================================
// Code Generation Templates
// ============================================================================
// Generate C code snippets for Simulink Coder integration

inline void generateSimulinkPIDCode(const char* output_file) {
    FILE* f = fopen(output_file, "w");
    if (!f) return;

    fprintf(f, "/* Auto-generated PID Controller for Simulink Coder */\n");
    fprintf(f, "/* Generated by RobotLib v2.2 */\n\n");
    fprintf(f, "#include <math.h>\n\n");
    fprintf(f, "typedef struct {\n");
    fprintf(f, "    double integral;\n");
    fprintf(f, "    double prev_error;\n");
    fprintf(f, "    double prev_derivative;\n");
    fprintf(f, "} PIDState;\n\n");
    fprintf(f, "double pid_step(double error, double dt,\n");
    fprintf(f, "                double kP, double kI, double kD,\n");
    fprintf(f, "                PIDState* state) {\n");
    fprintf(f, "    double P = kP * error;\n");
    fprintf(f, "    state->integral += error * dt;\n");
    fprintf(f, "    if (state->integral > 10.0) state->integral = 10.0;\n");
    fprintf(f, "    if (state->integral < -10.0) state->integral = -10.0;\n");
    fprintf(f, "    double I = kI * state->integral;\n");
    fprintf(f, "    double raw_derivative = (error - state->prev_error) / dt;\n");
    fprintf(f, "    double derivative = 0.1 * raw_derivative + 0.9 * state->prev_derivative;\n");
    fprintf(f, "    double D = kD * derivative;\n");
    fprintf(f, "    state->prev_error = error;\n");
    fprintf(f, "    state->prev_derivative = derivative;\n");
    fprintf(f, "    return P + I + D;\n");
    fprintf(f, "}\n");

    fclose(f);
}

inline void generateSimulinkOdometryCode(const char* output_file) {
    FILE* f = fopen(output_file, "w");
    if (!f) return;

    fprintf(f, "/* Auto-generated Odometry for Simulink Coder */\n");
    fprintf(f, "/* Generated by RobotLib v2.2 */\n\n");
    fprintf(f, "#include <math.h>\n\n");
    fprintf(f, "typedef struct {\n");
    fprintf(f, "    double x;\n");
    fprintf(f, "    double y;\n");
    fprintf(f, "    double heading;\n");
    fprintf(f, "} Pose2D;\n\n");
    fprintf(f, "void odometry_update(Pose2D* pose,\n");
    fprintf(f, "                     double left_distance,\n");
    fprintf(f, "                     double right_distance,\n");
    fprintf(f, "                     double wheelbase) {\n");
    fprintf(f, "    double distance = (left_distance + right_distance) / 2.0;\n");
    fprintf(f, "    double delta_heading = (right_distance - left_distance) / wheelbase;\n");
    fprintf(f, "    pose->heading += delta_heading;\n");
    fprintf(f, "    pose->x += distance * cos(pose->heading);\n");
    fprintf(f, "    pose->y += distance * sin(pose->heading);\n");
    fprintf(f, "}\n");

    fclose(f);
}

// ============================================================================
// MATLAB Script Generation
// ============================================================================
// Generate MATLAB scripts for data visualization

inline void generateMatlabPlotScript(const char* output_file,
                                      const char* csv_file) {
    FILE* f = fopen(output_file, "w");
    if (!f) return;

    fprintf(f, "%% Auto-generated MATLAB plotting script\n");
    fprintf(f, "%% Generated by RobotLib v2.2\n\n");
    fprintf(f, "clear all; close all;\n\n");
    fprintf(f, "%% Load data\n");
    fprintf(f, "data = csvread('%s', 1, 0);\n", csv_file);
    fprintf(f, "time = data(:, 1);\n");
    fprintf(f, "position = data(:, 2);\n");
    fprintf(f, "velocity = data(:, 3);\n\n");
    fprintf(f, "%% Plot position\n");
    fprintf(f, "figure(1);\n");
    fprintf(f, "subplot(2,1,1);\n");
    fprintf(f, "plot(time, position, 'b-', 'LineWidth', 2);\n");
    fprintf(f, "xlabel('Time (s)');\n");
    fprintf(f, "ylabel('Position (m)');\n");
    fprintf(f, "title('Robot Position vs Time');\n");
    fprintf(f, "grid on;\n\n");
    fprintf(f, "%% Plot velocity\n");
    fprintf(f, "subplot(2,1,2);\n");
    fprintf(f, "plot(time, velocity, 'r-', 'LineWidth', 2);\n");
    fprintf(f, "xlabel('Time (s)');\n");
    fprintf(f, "ylabel('Velocity (m/s)');\n");
    fprintf(f, "title('Robot Velocity vs Time');\n");
    fprintf(f, "grid on;\n");

    fclose(f);
}

// ============================================================================
// Usage Examples and Documentation
// ============================================================================

/*
BASIC USAGE - Data Export:
--------------------------
#include "units_matlab_interop.h"

// Create trajectory data
std::array<double, 100> time;
std::array<Meters, 100> position;
std::array<MetersPerSecond, 100> velocity;

// ... fill with data ...

// Export to CSV for MATLAB analysis
units::matlab::exportTrajectoryToCSV("trajectory.csv", time, position, velocity);

// In MATLAB:
// >> data = csvread('trajectory.csv', 1, 0);
// >> plot(data(:,1), data(:,2));


SIMULINK INTEGRATION:
--------------------
1. Generate Simulink-compatible code:
   units::matlab::generateSimulinkPIDCode("pid_controller.c");

2. In Simulink:
   - Add S-Function block
   - Point to generated code
   - Configure inputs/outputs
   - Build and run

3. For custom S-Functions, use simulinkPIDStep() directly:
   double control = simulinkPIDStep(error, dt, kP, kI, kD, &state);


CODE GENERATION EXAMPLE:
------------------------
// Generate PID controller code
units::matlab::generateSimulinkPIDCode("pid.c");

// Generate odometry code
units::matlab::generateSimulinkOdometryCode("odometry.c");

// Generate MATLAB visualization script
units::matlab::generateMatlabPlotScript("plot_trajectory.m", "trajectory.csv");


HARDWARE-IN-THE-LOOP (HIL):
---------------------------
1. Compile RobotLib code for target (e.g., Arduino, ESP32)
2. Generate Simulink model with actuator/sensor models
3. Use units::matlab conversion functions for data exchange
4. Run real-time simulation with physical hardware


RAPID PROTOTYPING WORKFLOW:
----------------------------
1. Develop controller in C++ with RobotLib
2. Test in software simulation
3. Export data to MATLAB for analysis:
   exportTrajectoryToCSV("results.csv", ...);
4. Generate MATLAB plots:
   generateMatlabPlotScript("plot.m", "results.csv");
5. Integrate in Simulink for HIL testing
6. Deploy to embedded hardware
*/

} // namespace matlab
} // namespace units

#endif // ROBOTLIB_UNITS_MATLAB_INTEROP_H
