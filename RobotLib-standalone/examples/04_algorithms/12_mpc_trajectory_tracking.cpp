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

#include "../../include/units_core.h"
#include "../../include/units_physics.h"
#include "../../include/units_robotics.h"
#include "../../include/units_control.h"

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
    println("========================================");
    println("  MPC Basics - Point-to-Point Motion");
    println("========================================\n");

    println("Moving from position 0 to position 5 with constraints:");
    println("  Max velocity: 2.0 m/s");
    println("  Max acceleration: 1.0 m/s²\n");

    // Create MPC controller for double integrator (position + velocity)
    MPCDoubleIntegrator mpc(0.1);  // 0.1s time step
    mpc.setLimits(2.0, 1.0);  // max velocity, max acceleration

    // Initial state: [position=0, velocity=0]
    std::array<double, 2> state = {{0.0, 0.0}};

    // Target state: [position=5, velocity=0]
    std::array<double, 2> target = {{5.0, 0.0}};

    print(, );
    println("Time(s) | Position(m) | Velocity(m/s) | Accel(m/s²) | Status");
    println("--------|-------------|---------------|-------------|--------");

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
            print(, t, " | ");
            print(, state[0], " | ");
            print(, state[1], " | ");
            print(, acceleration, " | ");

            double error = std::abs(state[0] - target[0]);
            if (error < 0.01) {
                print("At target");
            } else if (std::abs(state[1]) > 1.9) {
                print("Max velocity");
            } else {
                print("Moving");
            }
            println("");
        }

        // Stop if reached target
        if (std::abs(state[0] - target[0]) < 0.01 && std::abs(state[1]) < 0.01) {
            println("\nReached target at t = ", t, " seconds!");
            break;
        }

        t += dt;
    }

    println("");
}

void demonstrateTrajectoryTracking() {
    println("========================================");
    println("  MPC Trajectory Tracking");
    println("========================================\n");

    println("Tracking sinusoidal trajectory:");
    println("  y(t) = 3*sin(0.5*t)");
    println("  Amplitude: 3 meters");
    println("  Frequency: 0.5 Hz\n");

    // Create trajectory
    SinusoidalTrajectory trajectory(3.0, 0.5);

    // Create MPC controller
    MPCDoubleIntegrator mpc(0.1);
    mpc.setLimits(5.0, 3.0);  // Sufficient for this trajectory

    // Initial state
    std::array<double, 2> state = {{0.0, 0.0}};

    print(, );
    println("Time(s) | Reference | Actual | Error  | Velocity | Control");
    println("--------|-----------|--------|--------|----------|--------");

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
            print(, t, " | ");
            print(, ref_pos, " | ");
            print(, state[0], " | ");
            print(, error, " | ");
            print(, state[1], " | ");
            println(, acceleration, "");
        }

        t += dt;
    }

    double mae = total_error / count;
    println("\nMean Absolute Error: ", mae, " meters");
    println("Tracking performance: ", (mae < 0.1 ? "Excellent" : mae < 0.5 ? "Good" : "Fair"), "\n");
}

void compareMPCvsPID() {
    println("========================================");
    println("  MPC vs PID Comparison");
    println("========================================\n");

    println("Scenario: Move from 0 to 10m with velocity limit");
    println("  Max velocity: 2.0 m/s");
    println("  Max acceleration: 1.5 m/s²\n");

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

    print(, );
    println("Time(s) | MPC Pos | MPC Vel | PID Pos | PID Vel | Winner");
    println("--------|---------|---------|---------|---------|-------");

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
            print(, t, " | ");
            print(, mpc_state[0], " | ");
            print(, mpc_state[1], " | ");
            print(, pid_state[0], " | ");
            print(, pid_state[1], " | ");

            // Determine winner (better tracking, respects constraints)
            bool mpc_at_target = std::abs(mpc_state[0] - 10.0) < 0.1;
            bool pid_at_target = std::abs(pid_state[0] - 10.0) < 0.1;
            bool pid_violates = std::abs(pid_state[1]) > 2.0;

            if (mpc_at_target && !pid_at_target) {
                print("MPC");
            } else if (pid_violates) {
                print("MPC (safe)");
            } else if (pid_at_target && !mpc_at_target) {
                print("PID");
            } else {
                print("Tie");
            }
            println("");
        }

        t += dt;
    }

    println("\nKey Observations:");
    println("  • MPC respects velocity constraints");
    println("  • MPC provides smoother motion profile");
    println("  • MPC considers future trajectory");
    println("  • PID may violate constraints");
    println("  • MPC is more computationally intensive\n");
}

void demonstrateConstraintHandling() {
    println("========================================");
    println("  MPC Constraint Handling");
    println("========================================\n");

    println("Testing with tight constraints:");
    println("  Target: Move 5 meters");
    println("  Constraint 1: Max velocity = 0.5 m/s (tight!)");
    println("  Constraint 2: Max acceleration = 0.2 m/s²\n");

    MPCDoubleIntegrator mpc(0.1);
    mpc.setLimits(0.5, 0.2);  // Very tight constraints

    std::array<double, 2> state = {{0.0, 0.0}};
    std::array<double, 2> target = {{5.0, 0.0}};

    print(, );
    println("Time(s) | Position | Velocity | Accel | Vel Limit% | Acc Limit%");
    println("--------|----------|----------|-------|------------|-----------");

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

            print(, t, " | ");
            print(, state[0], " | ");
            print(, state[1], " | ");
            print(, acceleration, " | ");
            print(, vel_usage, " | ");
            println(, acc_usage, "");
        }

        if (std::abs(state[0] - target[0]) < 0.01 && std::abs(state[1]) < 0.01) {
            println("\nReached target at t = ", t, " seconds");
            println("MPC successfully handled tight constraints!");
            break;
        }

        t += dt;
    }

    println("");
}

// ============================================================================
// Main Program
// ============================================================================
int main() {
    println("");
    println("╔════════════════════════════════════════╗");
    println("║  Model Predictive Control (MPC)       ║");
    println("║  Optimal Control with Constraints      ║");
    println("╚════════════════════════════════════════╝");
    println("");

    demonstrateMPCBasics();
    demonstrateTrajectoryTracking();
    compareMPCvsPID();
    demonstrateConstraintHandling();

    println("========================================");
    println("  Key Takeaways");
    println("========================================\n");

    println("What is MPC?");
    println("  • Model Predictive Control optimizes control actions");
    println("  • Predicts future behavior over a horizon");
    println("  • Handles constraints naturally");
    println("  • Receding horizon approach (solves repeatedly)\n");

    println("When to Use MPC:");
    println("  ✓ Constraints are critical (safety, actuator limits)");
    println("  ✓ Multi-step ahead planning is valuable");
    println("  ✓ Smooth trajectories are important");
    println("  ✓ System model is reasonably accurate");
    println("  ✓ Sufficient computation available\n");

    println("When to Use PID Instead:");
    println("  ✓ Simple regulation problems");
    println("  ✓ Very limited computation");
    println("  ✓ No hard constraints");
    println("  ✓ Fast response critical\n");

    println("MPC Advantages:");
    println("  1. Constraint Handling");
    println("     • Velocity, acceleration, position limits");
    println("     • Automatically respects all constraints\n");

    println("  2. Optimal Performance");
    println("     • Minimizes specified cost function");
    println("     • Balances multiple objectives (tracking, control effort)\n");

    println("  3. Predictive Planning");
    println("     • Looks ahead to avoid problems");
    println("     • Smoother motion profiles\n");

    println("  4. Multi-Variable Control");
    println("     • Naturally handles MIMO systems");
    println("     • Considers coupling between variables\n");

    println("MPC Disadvantages:");
    println("  • Requires system model");
    println("  • Computationally intensive");
    println("  • Tuning complexity (horizon, weights)");
    println("  • May need warm-starting for real-time\n");

    println("Implementation Tips:");
    println("  1. Start with simple models (linear)");
    println("  2. Choose horizon length carefully");
    println("     • Too short: Poor performance");
    println("     • Too long: Excessive computation");
    println("  3. Tune weights (Q, R matrices)");
    println("     • Q: State error penalty");
    println("     • R: Control effort penalty");
    println("  4. Use warm-starting (previous solution)");
    println("  5. Consider real-time constraints\n");

    println("Real-World Applications:");
    println("  • Autonomous vehicle trajectory planning");
    println("  • Robot arm motion control");
    println("  • Drone stabilization and navigation");
    println("  • Industrial process control");
    println("  • Chemical plant optimization");
    println("  • Power system management\n");

    println("Further Reading:");
    println("  • \"Model Predictive Control\" by Camacho & Bordons");
    println("  • \"Predictive Control\" by Maciejowski");
    println("  • IEEE TAC, Automatica journals\n");

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
