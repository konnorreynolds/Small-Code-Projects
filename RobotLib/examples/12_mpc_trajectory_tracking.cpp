// ============================================================================
// Example 12: Model Predictive Control (MPC) for Trajectory Tracking
// ============================================================================
// Demonstrates Model Predictive Control for smooth trajectory tracking with
// constraints. MPC predicts future behavior and optimizes control inputs while
// respecting velocity and acceleration limits.
//
// Topics covered:
// - Model Predictive Control fundamentals
// - Trajectory tracking with constraints
// - Prediction horizon and optimization
// - Comparison with PID control
// - Handling velocity/acceleration limits
// - Smooth motion profiles
// ============================================================================

#include "../include/units_core.h"
#include "../include/units_physics.h"
#include "../include/units_robotics.h"
#include "../include/units_control.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace units;
using namespace units::control;

// ============================================================================
// Trajectory Generators
// ============================================================================

class SinusoidalTrajectory {
private:
    double amplitude_;
    double frequency_;
    double offset_;

public:
    SinusoidalTrajectory(double amplitude, double frequency, double offset = 0.0)
        : amplitude_(amplitude), frequency_(frequency), offset_(offset) {}

    double position(double t) const {
        return amplitude_ * std::sin(2.0 * M_PI * frequency_ * t) + offset_;
    }

    double velocity(double t) const {
        return amplitude_ * 2.0 * M_PI * frequency_ * std::cos(2.0 * M_PI * frequency_ * t);
    }

    double acceleration(double t) const {
        return -amplitude_ * std::pow(2.0 * M_PI * frequency_, 2) * std::sin(2.0 * M_PI * frequency_ * t);
    }
};

// ============================================================================
// Demonstration Functions
// ============================================================================

void demonstrateMPCBasics() {
    std::cout << "========================================\n";
    std::cout << "  MPC Basics - Point-to-Point Motion\n";
    std::cout << "========================================\n\n";

    std::cout << "Moving from position 0 to position 5 with constraints:\n";
    std::cout << "  Max velocity: 2.0 m/s\n";
    std::cout << "  Max acceleration: 1.0 m/s²\n\n";

    // Create MPC controller for double integrator (position + velocity)
    MPCDoubleIntegrator mpc(0.1);  // 0.1s time step
    mpc.setLimits(2.0, 1.0);  // max velocity, max acceleration

    // Initial state: [position=0, velocity=0]
    std::array<double, 2> state = {{0.0, 0.0}};

    // Target state: [position=5, velocity=0]
    std::array<double, 2> target = {{5.0, 0.0}};

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time(s) | Position(m) | Velocity(m/s) | Accel(m/s²) | Status\n";
    std::cout << "--------|-------------|---------------|-------------|--------\n";

    double t = 0.0;
    double dt = 0.1;

    for (int step = 0; step <= 60; step++) {
        // Solve MPC problem
        auto control = mpc.solve(state, target);
        double acceleration = control[0];

        // Apply control (simulate dynamics)
        state[0] += state[1] * dt + 0.5 * acceleration * dt * dt;  // position
        state[1] += acceleration * dt;  // velocity

        // Print every 10 steps
        if (step % 10 == 0) {
            std::cout << std::setw(7) << t << " | ";
            std::cout << std::setw(11) << state[0] << " | ";
            std::cout << std::setw(13) << state[1] << " | ";
            std::cout << std::setw(11) << acceleration << " | ";

            double error = std::abs(state[0] - target[0]);
            if (error < 0.01) {
                std::cout << "At target";
            } else if (std::abs(state[1]) > 1.9) {
                std::cout << "Max velocity";
            } else {
                std::cout << "Moving";
            }
            std::cout << "\n";
        }

        // Stop if reached target
        if (std::abs(state[0] - target[0]) < 0.01 && std::abs(state[1]) < 0.01) {
            std::cout << "\nReached target at t = " << t << " seconds!\n";
            break;
        }

        t += dt;
    }

    std::cout << "\n";
}

void demonstrateTrajectoryTracking() {
    std::cout << "========================================\n";
    std::cout << "  MPC Trajectory Tracking\n";
    std::cout << "========================================\n\n";

    std::cout << "Tracking sinusoidal trajectory:\n";
    std::cout << "  y(t) = 3*sin(0.5*t)\n";
    std::cout << "  Amplitude: 3 meters\n";
    std::cout << "  Frequency: 0.5 Hz\n\n";

    // Create trajectory
    SinusoidalTrajectory trajectory(3.0, 0.5);

    // Create MPC controller
    MPCDoubleIntegrator mpc(0.1);
    mpc.setLimits(5.0, 3.0);  // Sufficient for this trajectory

    // Initial state
    std::array<double, 2> state = {{0.0, 0.0}};

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time(s) | Reference | Actual | Error  | Velocity | Control\n";
    std::cout << "--------|-----------|--------|--------|----------|--------\n";

    double t = 0.0;
    double dt = 0.1;
    double total_error = 0.0;
    int count = 0;

    for (int step = 0; step <= 100; step++) {
        // Get reference from trajectory
        double ref_pos = trajectory.position(t);
        double ref_vel = trajectory.velocity(t);
        std::array<double, 2> reference = {{ref_pos, ref_vel}};

        // Solve MPC
        auto control = mpc.solve(state, reference);
        double acceleration = control[0];

        // Apply control
        state[0] += state[1] * dt + 0.5 * acceleration * dt * dt;
        state[1] += acceleration * dt;

        // Compute tracking error
        double error = state[0] - ref_pos;
        total_error += std::abs(error);
        count++;

        // Print every 10 steps
        if (step % 10 == 0) {
            std::cout << std::setw(7) << t << " | ";
            std::cout << std::setw(9) << ref_pos << " | ";
            std::cout << std::setw(6) << state[0] << " | ";
            std::cout << std::setw(6) << error << " | ";
            std::cout << std::setw(8) << state[1] << " | ";
            std::cout << std::setw(7) << acceleration << "\n";
        }

        t += dt;
    }

    double mae = total_error / count;
    std::cout << "\nMean Absolute Error: " << mae << " meters\n";
    std::cout << "Tracking performance: " << (mae < 0.1 ? "Excellent" : mae < 0.5 ? "Good" : "Fair") << "\n\n";
}

void compareMPCvsPID() {
    std::cout << "========================================\n";
    std::cout << "  MPC vs PID Comparison\n";
    std::cout << "========================================\n\n";

    std::cout << "Scenario: Move from 0 to 10m with velocity limit\n";
    std::cout << "  Max velocity: 2.0 m/s\n";
    std::cout << "  Max acceleration: 1.5 m/s²\n\n";

    // Setup MPC
    MPCDoubleIntegrator mpc(0.1);
    mpc.setLimits(2.0, 1.5);

    // Setup PID (no constraint handling)
    PIDController::Gains pid_gains(2.0, 0.1, 0.5);
    pid_gains.outputMin = -1.5;
    pid_gains.outputMax = 1.5;
    PIDController pid(pid_gains);

    // Initial states
    std::array<double, 2> mpc_state = {{0.0, 0.0}};
    std::array<double, 2> pid_state = {{0.0, 0.0}};
    std::array<double, 2> target = {{10.0, 0.0}};

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time(s) | MPC Pos | MPC Vel | PID Pos | PID Vel | Winner\n";
    std::cout << "--------|---------|---------|---------|---------|-------\n";

    double t = 0.0;
    double dt = 0.1;

    for (int step = 0; step <= 80; step++) {
        // MPC control
        auto mpc_control = mpc.solve(mpc_state, target);
        double mpc_accel = mpc_control[0];
        mpc_state[0] += mpc_state[1] * dt + 0.5 * mpc_accel * dt * dt;
        mpc_state[1] += mpc_accel * dt;

        // PID control (position error only, no velocity limiting)
        double pid_error = target[0] - pid_state[0];
        double pid_accel = pid.calculate(pid_error, dt);
        pid_state[0] += pid_state[1] * dt + 0.5 * pid_accel * dt * dt;
        pid_state[1] += pid_accel * dt;

        // Print every 10 steps
        if (step % 10 == 0) {
            std::cout << std::setw(7) << t << " | ";
            std::cout << std::setw(7) << mpc_state[0] << " | ";
            std::cout << std::setw(7) << mpc_state[1] << " | ";
            std::cout << std::setw(7) << pid_state[0] << " | ";
            std::cout << std::setw(7) << pid_state[1] << " | ";

            // Determine winner (better tracking, respects constraints)
            bool mpc_at_target = std::abs(mpc_state[0] - 10.0) < 0.1;
            bool pid_at_target = std::abs(pid_state[0] - 10.0) < 0.1;
            bool pid_violates = std::abs(pid_state[1]) > 2.0;

            if (mpc_at_target && !pid_at_target) {
                std::cout << "MPC";
            } else if (pid_violates) {
                std::cout << "MPC (safe)";
            } else if (pid_at_target && !mpc_at_target) {
                std::cout << "PID";
            } else {
                std::cout << "Tie";
            }
            std::cout << "\n";
        }

        t += dt;
    }

    std::cout << "\nKey Observations:\n";
    std::cout << "  • MPC respects velocity constraints\n";
    std::cout << "  • MPC provides smoother motion profile\n";
    std::cout << "  • MPC considers future trajectory\n";
    std::cout << "  • PID may violate constraints\n";
    std::cout << "  • MPC is more computationally intensive\n\n";
}

void demonstrateConstraintHandling() {
    std::cout << "========================================\n";
    std::cout << "  MPC Constraint Handling\n";
    std::cout << "========================================\n\n";

    std::cout << "Testing with tight constraints:\n";
    std::cout << "  Target: Move 5 meters\n";
    std::cout << "  Constraint 1: Max velocity = 0.5 m/s (tight!)\n";
    std::cout << "  Constraint 2: Max acceleration = 0.2 m/s²\n\n";

    MPCDoubleIntegrator mpc(0.1);
    mpc.setLimits(0.5, 0.2);  // Very tight constraints

    std::array<double, 2> state = {{0.0, 0.0}};
    std::array<double, 2> target = {{5.0, 0.0}};

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Time(s) | Position | Velocity | Accel | Vel Limit% | Acc Limit%\n";
    std::cout << "--------|----------|----------|-------|------------|-----------\n";

    double t = 0.0;
    double dt = 0.1;
    double max_vel = 0.5;
    double max_acc = 0.2;

    for (int step = 0; step <= 150; step++) {
        auto control = mpc.solve(state, target);
        double acceleration = control[0];

        state[0] += state[1] * dt + 0.5 * acceleration * dt * dt;
        state[1] += acceleration * dt;

        if (step % 15 == 0) {
            double vel_usage = (std::abs(state[1]) / max_vel) * 100.0;
            double acc_usage = (std::abs(acceleration) / max_acc) * 100.0;

            std::cout << std::setw(7) << t << " | ";
            std::cout << std::setw(8) << state[0] << " | ";
            std::cout << std::setw(8) << state[1] << " | ";
            std::cout << std::setw(5) << acceleration << " | ";
            std::cout << std::setw(10) << vel_usage << " | ";
            std::cout << std::setw(10) << acc_usage << "\n";
        }

        if (std::abs(state[0] - target[0]) < 0.01 && std::abs(state[1]) < 0.01) {
            std::cout << "\nReached target at t = " << t << " seconds\n";
            std::cout << "MPC successfully handled tight constraints!\n";
            break;
        }

        t += dt;
    }

    std::cout << "\n";
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║  Model Predictive Control (MPC)       ║\n";
    std::cout << "║  Optimal Control with Constraints      ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    std::cout << "\n";

    demonstrateMPCBasics();
    demonstrateTrajectoryTracking();
    compareMPCvsPID();
    demonstrateConstraintHandling();

    std::cout << "========================================\n";
    std::cout << "  Key Takeaways\n";
    std::cout << "========================================\n\n";

    std::cout << "What is MPC?\n";
    std::cout << "  • Model Predictive Control optimizes control actions\n";
    std::cout << "  • Predicts future behavior over a horizon\n";
    std::cout << "  • Handles constraints naturally\n";
    std::cout << "  • Receding horizon approach (solves repeatedly)\n\n";

    std::cout << "When to Use MPC:\n";
    std::cout << "  ✓ Constraints are critical (safety, actuator limits)\n";
    std::cout << "  ✓ Multi-step ahead planning is valuable\n";
    std::cout << "  ✓ Smooth trajectories are important\n";
    std::cout << "  ✓ System model is reasonably accurate\n";
    std::cout << "  ✓ Sufficient computation available\n\n";

    std::cout << "When to Use PID Instead:\n";
    std::cout << "  ✓ Simple regulation problems\n";
    std::cout << "  ✓ Very limited computation\n";
    std::cout << "  ✓ No hard constraints\n";
    std::cout << "  ✓ Fast response critical\n\n";

    std::cout << "MPC Advantages:\n";
    std::cout << "  1. Constraint Handling\n";
    std::cout << "     • Velocity, acceleration, position limits\n";
    std::cout << "     • Automatically respects all constraints\n\n";

    std::cout << "  2. Optimal Performance\n";
    std::cout << "     • Minimizes specified cost function\n";
    std::cout << "     • Balances multiple objectives (tracking, control effort)\n\n";

    std::cout << "  3. Predictive Planning\n";
    std::cout << "     • Looks ahead to avoid problems\n";
    std::cout << "     • Smoother motion profiles\n\n";

    std::cout << "  4. Multi-Variable Control\n";
    std::cout << "     • Naturally handles MIMO systems\n";
    std::cout << "     • Considers coupling between variables\n\n";

    std::cout << "MPC Disadvantages:\n";
    std::cout << "  • Requires system model\n";
    std::cout << "  • Computationally intensive\n";
    std::cout << "  • Tuning complexity (horizon, weights)\n";
    std::cout << "  • May need warm-starting for real-time\n\n";

    std::cout << "Implementation Tips:\n";
    std::cout << "  1. Start with simple models (linear)\n";
    std::cout << "  2. Choose horizon length carefully\n";
    std::cout << "     • Too short: Poor performance\n";
    std::cout << "     • Too long: Excessive computation\n";
    std::cout << "  3. Tune weights (Q, R matrices)\n";
    std::cout << "     • Q: State error penalty\n";
    std::cout << "     • R: Control effort penalty\n";
    std::cout << "  4. Use warm-starting (previous solution)\n";
    std::cout << "  5. Consider real-time constraints\n\n";

    std::cout << "Real-World Applications:\n";
    std::cout << "  • Autonomous vehicle trajectory planning\n";
    std::cout << "  • Robot arm motion control\n";
    std::cout << "  • Drone stabilization and navigation\n";
    std::cout << "  • Industrial process control\n";
    std::cout << "  • Chemical plant optimization\n";
    std::cout << "  • Power system management\n\n";

    std::cout << "Further Reading:\n";
    std::cout << "  • \"Model Predictive Control\" by Camacho & Bordons\n";
    std::cout << "  • \"Predictive Control\" by Maciejowski\n";
    std::cout << "  • IEEE TAC, Automatica journals\n\n";

    return 0;
}

/*
Expected Output:
    - Basic MPC demonstration (point-to-point)
    - Sinusoidal trajectory tracking
    - MPC vs PID comparison
    - Constraint handling demonstration

This example demonstrates:
    - Model Predictive Control fundamentals
    - Trajectory tracking with MPC
    - Constraint satisfaction (velocity, acceleration)
    - Comparison with classical PID control
    - Practical tuning and implementation

Perfect for:
    - Understanding MPC concepts
    - Learning when to use MPC vs PID
    - Implementing constrained control
    - Advanced robotics applications
*/
