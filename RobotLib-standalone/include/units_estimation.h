// ============================================================================
// units_estimation.h - Advanced State Estimation for Robotics
// ============================================================================
// Purpose: Extended Kalman Filter and advanced estimation algorithms
// Dependencies: units_core.h, units_3d.h
//
// This file contains:
// - Extended Kalman Filter (EKF) for nonlinear systems
// - Unscented Kalman Filter (UKF) for highly nonlinear systems
// - Particle Filter for multi-modal distributions
// - Helper functions for state estimation
// ============================================================================

#ifndef ROBOTICS_UNITS_ESTIMATION_H
#define ROBOTICS_UNITS_ESTIMATION_H

#include "units_core.h"
#include <vector>
#include <functional>
#include <cmath>
#include <algorithm>

namespace units {
namespace estimation {

// ============================================================================
// EXTENDED KALMAN FILTER (EKF)
// ============================================================================
// WHAT: Optimal state estimator for nonlinear systems
//
// THE PROBLEM EKF SOLVES:
//   Robots operate in uncertain world with:
//   - Noisy sensors (measurements are never perfect)
//   - Imperfect models (predictions drift over time)
//   - Nonlinear dynamics (turning, accelerating, etc.)
//
//   QUESTION: How do we estimate true state (position, velocity, etc.)
//            given noisy measurements and imperfect predictions?
//
//   ANSWER: Extended Kalman Filter!
//
// WHY "EXTENDED":
//   - Original Kalman Filter: Only works for LINEAR systems
//   - Extended Kalman Filter: Works for NONLINEAR systems
//   - Extension: Linearizes nonlinear functions using Jacobian matrices
//
// HOW IT WORKS (Two-Step Cycle):
//
//   1. PREDICTION STEP (Time Update):
//      "Where do I think I am based on my motion?"
//      - Use robot's motion model to predict next state
//      - Uncertainty increases (prediction isn't perfect)
//
//   2. UPDATE STEP (Measurement Update):
//      "Now I see a landmark, let me correct my estimate"
//      - Use sensor measurement to correct prediction
//      - Uncertainty decreases (measurement helps!)
//
// MATH OVERVIEW:
//
//   STATE: x = [position, velocity, heading, ...]ᵀ
//      Everything we want to estimate
//
//   COVARIANCE: P = uncertainty in our estimate
//      How confident are we? Small P = very confident
//
//   PREDICTION:
//      x̂ₖ = f(x̂ₖ₋₁, uₖ)           Predict state
//      Pₖ = FₖPₖ₋₁Fₖᵀ + Q          Predict uncertainty
//
//   UPDATE:
//      Kₖ = PₖHₖᵀ(HₖPₖHₖᵀ + R)⁻¹  Kalman gain
//      x̂ₖ = x̂ₖ + Kₖ(zₖ - h(x̂ₖ))   Correct state
//      Pₖ = (I - KₖHₖ)Pₖ            Correct uncertainty
//
//   WHERE:
//      f(x,u) = nonlinear state transition function
//      h(x) = nonlinear measurement function
//      F = Jacobian of f (linearization!)
//      H = Jacobian of h (linearization!)
//      Q = process noise (model uncertainty)
//      R = measurement noise (sensor uncertainty)
//      K = Kalman gain (optimal blending factor)
//
// VISUAL EXAMPLE (Robot Localization):
//
//   Time 1: PREDICTION
//      "I drove forward 1m"
//      [Previous position] --model--> [Predicted position]
//           (confident)                    (less confident)
//           P = 0.1                        P = 0.3
//
//   Time 2: MEASUREMENT
//      "I see a landmark at known location"
//      [Predicted: x=5.2, P=0.3]  +  [Measured: x=5.0, R=0.2]
//              ↓
//      [Fused: x=5.08, P=0.12]  (Better than either alone!)
//
// JACOBIAN MATRICES (Key to "Extended"):
//
//   F = ∂f/∂x = Jacobian of state transition
//      Tells us: "How does each state variable affect others?"
//
//   H = ∂h/∂x = Jacobian of measurement function
//      Tells us: "How does state affect measurements?"
//
//   EXAMPLE (2D position):
//      State: x = [x, y, θ]
//      Motion: x' = x + v×cos(θ)×dt
//              y' = y + v×sin(θ)×dt
//              θ' = θ + ω×dt
//
//      Jacobian F:
//      ⎡ 1   0   -v×sin(θ)×dt ⎤
//      ⎢ 0   1    v×cos(θ)×dt ⎥
//      ⎣ 0   0    1           ⎦
//
// WHEN TO USE EKF:
//   ✓ System is "mildly" nonlinear
//   ✓ You can compute Jacobians (analytical or numerical)
//   ✓ Need real-time performance
//   ✓ Common in: robot localization, SLAM, GPS/INS fusion
//
// LIMITATIONS:
//   ✗ Assumes Gaussian noise (not always true)
//   ✗ Linearization can fail for highly nonlinear systems
//   ✗ Requires Jacobian calculation (can be complex)
//   → For highly nonlinear: Use UKF (Unscented Kalman Filter)
//
// TYPICAL APPLICATION (Robot Localization):
//   State: [x, y, θ, vₓ, vᵧ, ω]
//   Prediction: Integrate velocity and angular velocity
//   Measurement: GPS position, landmark bearings, etc.
//   Result: Smooth, accurate position estimate
//
// ============================================================================
template<size_t STATE_DIM, size_t MEASUREMENT_DIM>
class ExtendedKalmanFilter {
public:
    using StateVector = std::array<double, STATE_DIM>;
    using MeasurementVector = std::array<double, MEASUREMENT_DIM>;
    using StateMatrix = std::array<std::array<double, STATE_DIM>, STATE_DIM>;
    using MeasurementMatrix = std::array<std::array<double, MEASUREMENT_DIM>, MEASUREMENT_DIM>;
    using KalmanGainMatrix = std::array<std::array<double, STATE_DIM>, MEASUREMENT_DIM>;

    // Function types for user-defined models
    using StatePredictionFunc = std::function<StateVector(const StateVector&, double)>;
    using StateJacobianFunc = std::function<StateMatrix(const StateVector&, double)>;
    using MeasurementFunc = std::function<MeasurementVector(const StateVector&)>;
    using MeasurementJacobianFunc = std::function<std::array<std::array<double, STATE_DIM>, MEASUREMENT_DIM>(const StateVector&)>;

private:
    StateVector state_;          // State estimate
    StateMatrix covariance_;     // State covariance (uncertainty)
    StateMatrix processNoise_;   // Process noise covariance (Q)
    MeasurementMatrix measurementNoise_;  // Measurement noise covariance (R)

    // User-provided functions
    StatePredictionFunc predict_f_;
    StateJacobianFunc jacobian_F_;
    MeasurementFunc measure_h_;
    MeasurementJacobianFunc jacobian_H_;

public:
    // Constructor
    ExtendedKalmanFilter(
        const StateVector& initial_state,
        const StateMatrix& initial_covariance,
        const StateMatrix& process_noise,
        const MeasurementMatrix& measurement_noise
    ) : state_(initial_state),
        covariance_(initial_covariance),
        processNoise_(process_noise),
        measurementNoise_(measurement_noise) {}

    // Set model functions
    void setStateModel(StatePredictionFunc f, StateJacobianFunc F) {
        predict_f_ = f;
        jacobian_F_ = F;
    }

    void setMeasurementModel(MeasurementFunc h, MeasurementJacobianFunc H) {
        measure_h_ = h;
        jacobian_H_ = H;
    }

    // ========================================================================
    // PREDICTION STEP (Time Update)
    // ========================================================================
    // WHAT: Predict where robot will be after dt seconds
    //
    // WHY: Between measurements, we need to track robot motion
    //   - Robot drives forward → position changes
    //   - No sensor reading yet → rely on motion model
    //   - Uncertainty INCREASES (predictions aren't perfect!)
    //
    // THE TWO PREDICTION EQUATIONS:
    //
    //   1. STATE PREDICTION:
    //      x̂ₖ = f(x̂ₖ₋₁, uₖ)
    //
    //      WHERE:
    //        x̂ₖ = predicted state at time k
    //        f = nonlinear state transition function
    //        uₖ = control input (velocity commands, motor voltages, etc.)
    //
    //      EXAMPLE (2D robot):
    //        State: x = [x_pos, y_pos, θ]
    //        Motion: "Drive forward 1m/s for 0.1s"
    //        f(x, dt) = [x + v×cos(θ)×dt,
    //                    y + v×sin(θ)×dt,
    //                    θ + ω×dt]
    //
    //   2. COVARIANCE PREDICTION (Uncertainty Propagation):
    //      Pₖ = F × Pₖ₋₁ × Fᵀ + Q
    //
    //      WHERE:
    //        P = covariance matrix (uncertainty)
    //        F = Jacobian of f (linearization!)
    //        Q = process noise (model imperfections)
    //
    //      WHY THIS FORMULA?
    //        - F×P×Fᵀ: Propagate old uncertainty through linearized dynamics
    //        - + Q: Add uncertainty from model errors
    //
    // JACOBIAN F (Key to "Extended" KF):
    //
    //   F = ∂f/∂x = how small changes in state affect predictions
    //
    //   EXAMPLE (same 2D robot):
    //     f = [x + v×cos(θ)×dt, y + v×sin(θ)×dt, θ + ω×dt]
    //
    //     Jacobian:
    //     F = ⎡ ∂f₁/∂x   ∂f₁/∂y   ∂f₁/∂θ ⎤   ⎡ 1   0   -v×sin(θ)×dt ⎤
    //         ⎢ ∂f₂/∂x   ∂f₂/∂y   ∂f₂/∂θ ⎥ = ⎢ 0   1    v×cos(θ)×dt ⎥
    //         ⎣ ∂f₃/∂x   ∂f₃/∂y   ∂f₃/∂θ ⎦   ⎣ 0   0    1           ⎦
    //
    //     INTERPRETATION:
    //       - F[0][2] = -v×sin(θ)×dt: "x position is sensitive to heading!"
    //       - F[1][2] = v×cos(θ)×dt:  "y position also depends on heading"
    //       - Diagonal 1's: Each variable affects itself directly
    //
    // WHY UNCERTAINTY INCREASES:
    //
    //   Initial: P = ⎡0.1  0  ⎤  (confident in position)
    //                ⎣ 0  0.1⎦
    //
    //   After prediction: P = ⎡0.3  0.05⎤  (less confident now!)
    //                         ⎣0.05 0.3 ⎦
    //
    //   REASON: Motion model isn't perfect:
    //     - Wheel slip
    //     - Uneven ground
    //     - Motor noise
    //     → Process noise Q adds uncertainty!
    //
    // NUMERICAL EXAMPLE:
    //
    //   State: x = [10.0, 5.0, 0.0]  (x=10m, y=5m, θ=0°)
    //   Motion: v=1m/s, ω=0rad/s, dt=0.1s
    //
    //   Prediction:
    //     x_new = 10.0 + 1.0×cos(0)×0.1 = 10.1m
    //     y_new = 5.0 + 1.0×sin(0)×0.1 = 5.0m
    //     θ_new = 0.0 + 0×0.1 = 0.0rad
    //
    //   Covariance (simplified):
    //     P_old = 0.1 (position uncertainty)
    //     F ≈ I (identity, since heading constant)
    //     Q = 0.01 (process noise)
    //     P_new = I × 0.1 × I + 0.01 = 0.11
    //     → Uncertainty grew from 0.1 to 0.11!
    //
    // ========================================================================
    void predict(double dt) {
        if (!predict_f_ || !jacobian_F_) {
            return;  // Models not set
        }

        // Predict state: x̂ₖ = f(x̂ₖ₋₁, uₖ)
        state_ = predict_f_(state_, dt);

        // Predict covariance: Pₖ = F × Pₖ₋₁ × Fᵀ + Q
        StateMatrix F = jacobian_F_(state_, dt);
        StateMatrix F_P = matrixMultiply(F, covariance_);
        StateMatrix F_P_Ft = matrixMultiplyTranspose(F_P, F);
        covariance_ = matrixAdd(F_P_Ft, processNoise_);
    }

    // ========================================================================
    // UPDATE STEP (Measurement Update / Correction)
    // ========================================================================
    // WHAT: Correct prediction using new sensor measurement
    //
    // WHY: Measurements reduce uncertainty!
    //   - Prediction alone → uncertainty grows over time
    //   - Measurement → fresh information from sensors
    //   - Optimal fusion → best estimate combining both!
    //
    // THE KALMAN FILTER'S MAGIC:
    //   It computes the OPTIMAL way to blend prediction and measurement
    //   "Optimal" = Minimizes mean squared error
    //
    // THE UPDATE EQUATIONS (5 steps):
    //
    //   1. INNOVATION (Measurement Residual):
    //      y = z - h(x̂)
    //
    //      WHERE:
    //        z = actual measurement from sensor
    //        h(x̂) = predicted measurement (what we expect to see)
    //        y = innovation (the "surprise" in the measurement)
    //
    //      EXAMPLE:
    //        Predicted position: x̂ = 10.0m
    //        GPS measures: z = 10.5m
    //        Innovation: y = 10.5 - 10.0 = 0.5m
    //        → "We're 0.5m off from prediction!"
    //
    //   2. INNOVATION COVARIANCE:
    //      S = H × P × Hᵀ + R
    //
    //      WHERE:
    //        S = uncertainty in the innovation
    //        H = measurement Jacobian (∂h/∂x)
    //        P = state covariance (prediction uncertainty)
    //        R = measurement noise (sensor uncertainty)
    //
    //      WHY:
    //        H×P×Hᵀ = Uncertainty from state estimate
    //        + R    = Uncertainty from sensor noise
    //        → Total uncertainty in measurement prediction
    //
    //   3. KALMAN GAIN (The Optimal Blending Factor):
    //      K = P × Hᵀ × S⁻¹
    //
    //      WHERE:
    //        K = Kalman gain matrix
    //        Range: 0 (ignore measurement) to 1 (trust measurement fully)
    //
    //      INTERPRETATION:
    //        - If P large (uncertain prediction): K → 1 (trust measurement more)
    //        - If R large (noisy sensor): K → 0 (trust prediction more)
    //        - Kalman gain is OPTIMAL balance!
    //
    //      EXAMPLE VALUES:
    //        K = 0.1 → Trust prediction 90%, measurement 10%
    //        K = 0.5 → Equal trust
    //        K = 0.9 → Trust measurement 90%, prediction 10%
    //
    //   4. STATE CORRECTION:
    //      x̂ = x̂ + K × y
    //
    //      WHERE:
    //        x̂ = corrected state estimate
    //        K×y = weighted correction based on innovation
    //
    //      EXAMPLE:
    //        Predicted: x̂ = 10.0m, uncertainty P = 0.3
    //        Measured: z = 10.5m, uncertainty R = 0.2
    //        Innovation: y = 0.5m
    //        Kalman gain: K ≈ 0.6 (favor measurement slightly)
    //        Correction: x̂ = 10.0 + 0.6×0.5 = 10.3m
    //        → Blended estimate between 10.0 and 10.5!
    //
    //   5. COVARIANCE CORRECTION:
    //      P = (I - K × H) × P
    //
    //      WHERE:
    //        P = corrected covariance (reduced uncertainty!)
    //        I = identity matrix
    //
    //      RESULT: Uncertainty DECREASES!
    //        Before: P = 0.3
    //        After: P = 0.12
    //        → We're more confident now!
    //
    // MEASUREMENT JACOBIAN H:
    //
    //   H = ∂h/∂x = how measurements depend on state
    //
    //   EXAMPLE (GPS measuring position):
    //     State: x = [x_pos, y_pos, θ, v]  (4D state)
    //     Measurement: z = [x_pos, y_pos]  (2D GPS)
    //     h(x) = [x[0], x[1]]  (extract position)
    //
    //     Jacobian:
    //     H = ⎡ ∂h₁/∂x  ∂h₁/∂y  ∂h₁/∂θ  ∂h₁/∂v ⎤   ⎡ 1  0  0  0 ⎤
    //         ⎣ ∂h₂/∂x  ∂h₂/∂y  ∂h₂/∂θ  ∂h₂/∂v ⎦ = ⎣ 0  1  0  0 ⎦
    //
    //     INTERPRETATION:
    //       - GPS measures x position directly → H[0][0] = 1
    //       - GPS measures y position directly → H[1][1] = 1
    //       - GPS doesn't see heading or velocity → zeros
    //
    // COMPLETE NUMERICAL EXAMPLE:
    //
    //   State: x̂ = [10.0, 5.0]  (position estimate)
    //   Covariance: P = diag(0.3, 0.3)  (uncertainty)
    //   Measurement noise: R = diag(0.2, 0.2)
    //   GPS measurement: z = [10.5, 5.2]
    //
    //   Step 1 - Innovation:
    //     y = [10.5, 5.2] - [10.0, 5.0] = [0.5, 0.2]
    //
    //   Step 2 - Innovation covariance:
    //     H = I (measuring position directly)
    //     S = I×P×I + R = P + R = diag(0.5, 0.5)
    //
    //   Step 3 - Kalman gain:
    //     K = P × S⁻¹ = diag(0.3, 0.3) × diag(2, 2) = diag(0.6, 0.6)
    //     → Trust measurement 60%, prediction 40%
    //
    //   Step 4 - State correction:
    //     x̂ = [10.0, 5.0] + diag(0.6, 0.6) × [0.5, 0.2]
    //       = [10.0, 5.0] + [0.3, 0.12]
    //       = [10.3, 5.12]
    //     → Blended result!
    //
    //   Step 5 - Covariance correction:
    //     P = (I - K×H) × P
    //       = (I - diag(0.6, 0.6)) × diag(0.3, 0.3)
    //       = diag(0.4, 0.4) × diag(0.3, 0.3)
    //       = diag(0.12, 0.12)
    //     → Uncertainty decreased from 0.3 to 0.12!
    //
    // WHY THIS IS OPTIMAL:
    //   The Kalman gain K minimizes the expected squared error
    //   Mathematically proven to be the best linear estimator!
    //   No other blending factor gives better accuracy
    //
    // ========================================================================
    void update(const MeasurementVector& measurement) {
        if (!measure_h_ || !jacobian_H_) {
            return;  // Models not set
        }

        // STEP 1: Innovation (measurement residual)
        // y = z - h(x̂)
        MeasurementVector predicted = measure_h_(state_);
        MeasurementVector innovation;
        for (size_t i = 0; i < MEASUREMENT_DIM; i++) {
            innovation[i] = measurement[i] - predicted[i];
        }

        // Get measurement Jacobian H = ∂h/∂x
        auto H = jacobian_H_(state_);

        // STEP 2: Innovation covariance
        // S = H × P × Hᵀ + R
        // (Uncertainty in measurement prediction)

        // First: Compute H × P
        std::array<std::array<double, STATE_DIM>, MEASUREMENT_DIM> H_P;
        for (size_t i = 0; i < MEASUREMENT_DIM; i++) {
            for (size_t j = 0; j < STATE_DIM; j++) {
                H_P[i][j] = 0;
                for (size_t k = 0; k < STATE_DIM; k++) {
                    H_P[i][j] += H[i][k] * covariance_[k][j];
                }
            }
        }

        // Then: Compute S = H × P × Hᵀ + R
        MeasurementMatrix S;
        for (size_t i = 0; i < MEASUREMENT_DIM; i++) {
            for (size_t j = 0; j < MEASUREMENT_DIM; j++) {
                S[i][j] = measurementNoise_[i][j];  // Start with R
                for (size_t k = 0; k < STATE_DIM; k++) {
                    S[i][j] += H_P[i][k] * H[j][k];  // Add H×P×Hᵀ
                }
            }
        }

        // STEP 3: Kalman gain
        // K = P × Hᵀ × S⁻¹
        // (Optimal blending factor between prediction and measurement)
        MeasurementMatrix S_inv = matrixInverse(S);
        KalmanGainMatrix K;
        for (size_t i = 0; i < STATE_DIM; i++) {
            for (size_t j = 0; j < MEASUREMENT_DIM; j++) {
                K[i][j] = 0;
                for (size_t k = 0; k < STATE_DIM; k++) {
                    for (size_t m = 0; m < MEASUREMENT_DIM; m++) {
                        K[i][j] += covariance_[i][k] * H[m][k] * S_inv[m][j];
                    }
                }
            }
        }

        // STEP 4: State correction
        // x̂ = x̂ + K × y
        // (Apply weighted correction based on innovation)
        for (size_t i = 0; i < STATE_DIM; i++) {
            for (size_t j = 0; j < MEASUREMENT_DIM; j++) {
                state_[i] += K[i][j] * innovation[j];
            }
        }

        // STEP 5: Covariance correction
        // P = (I - K × H) × P
        // (Reduce uncertainty - we're more confident now!)
        StateMatrix I_KH;
        for (size_t i = 0; i < STATE_DIM; i++) {
            for (size_t j = 0; j < STATE_DIM; j++) {
                I_KH[i][j] = (i == j) ? 1.0 : 0.0;  // Start with identity matrix
                for (size_t k = 0; k < MEASUREMENT_DIM; k++) {
                    I_KH[i][j] -= K[i][k] * H[k][j];  // Subtract K×H
                }
            }
        }

        covariance_ = matrixMultiply(I_KH, covariance_);
    }

    // Getters
    const StateVector& getState() const { return state_; }
    const StateMatrix& getCovariance() const { return covariance_; }

    // Get state uncertainty (diagonal of covariance)
    StateVector getUncertainty() const {
        StateVector unc;
        for (size_t i = 0; i < STATE_DIM; i++) {
            unc[i] = std::sqrt(std::abs(covariance_[i][i]));
        }
        return unc;
    }

    // Reset filter
    void reset(const StateVector& state, const StateMatrix& covariance) {
        state_ = state;
        covariance_ = covariance;
    }

private:
    // Matrix operations (helper functions)
    StateMatrix matrixMultiply(const StateMatrix& A, const StateMatrix& B) const {
        StateMatrix result;
        for (size_t i = 0; i < STATE_DIM; i++) {
            for (size_t j = 0; j < STATE_DIM; j++) {
                result[i][j] = 0;
                for (size_t k = 0; k < STATE_DIM; k++) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }

    StateMatrix matrixMultiplyTranspose(const StateMatrix& A, const StateMatrix& B) const {
        StateMatrix result;
        for (size_t i = 0; i < STATE_DIM; i++) {
            for (size_t j = 0; j < STATE_DIM; j++) {
                result[i][j] = 0;
                for (size_t k = 0; k < STATE_DIM; k++) {
                    result[i][j] += A[i][k] * B[j][k];  // Note: B[j][k] not B[k][j]
                }
            }
        }
        return result;
    }

    StateMatrix matrixAdd(const StateMatrix& A, const StateMatrix& B) const {
        StateMatrix result;
        for (size_t i = 0; i < STATE_DIM; i++) {
            for (size_t j = 0; j < STATE_DIM; j++) {
                result[i][j] = A[i][j] + B[i][j];
            }
        }
        return result;
    }

    MeasurementMatrix matrixInverse(const MeasurementMatrix& M) const {
        // For small matrices, use Gauss-Jordan elimination
        // This is a simplified version for common small measurement dimensions
        MeasurementMatrix inv = M;
        MeasurementMatrix aug;

        // Initialize augmented matrix as identity
        for (size_t i = 0; i < MEASUREMENT_DIM; i++) {
            for (size_t j = 0; j < MEASUREMENT_DIM; j++) {
                aug[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }

        // Forward elimination
        for (size_t i = 0; i < MEASUREMENT_DIM; i++) {
            // Find pivot
            double pivot = inv[i][i];
            if (std::abs(pivot) < 1e-10) {
                // Singular matrix - return identity (graceful degradation)
                return aug;
            }

            // Scale row
            for (size_t j = 0; j < MEASUREMENT_DIM; j++) {
                inv[i][j] /= pivot;
                aug[i][j] /= pivot;
            }

            // Eliminate column
            for (size_t k = 0; k < MEASUREMENT_DIM; k++) {
                if (k != i) {
                    double factor = inv[k][i];
                    for (size_t j = 0; j < MEASUREMENT_DIM; j++) {
                        inv[k][j] -= factor * inv[i][j];
                        aug[k][j] -= factor * aug[i][j];
                    }
                }
            }
        }

        return aug;
    }
};

// ============================================================================
// Common EKF Configurations
// ============================================================================

// Simple 2D position and velocity tracker (4 states: x, y, vx, vy)
class EKF2DPositionVelocity : public ExtendedKalmanFilter<4, 2> {
public:
    EKF2DPositionVelocity(double process_noise, double measurement_noise)
        : ExtendedKalmanFilter<4, 2>(
            {0, 0, 0, 0},  // Initial state
            createInitialCovariance(),
            createProcessNoise(process_noise),
            createMeasurementNoise(measurement_noise)
        ) {
        // Set constant velocity model
        setStateModel(
            // State prediction: x_new = x + vx*dt, y_new = y + vy*dt
            [](const StateVector& x, double dt) -> StateVector {
                return {
                    x[0] + x[2] * dt,  // x = x + vx*dt
                    x[1] + x[3] * dt,  // y = y + vy*dt
                    x[2],              // vx unchanged
                    x[3]               // vy unchanged
                };
            },
            // State Jacobian F
            [](const StateVector&, double dt) -> StateMatrix {
                return {{
                    {1, 0, dt, 0},
                    {0, 1, 0, dt},
                    {0, 0, 1, 0},
                    {0, 0, 0, 1}
                }};
            }
        );

        // Set measurement model (measure position only)
        setMeasurementModel(
            // Measurement function: h(x) = [x, y]
            [](const StateVector& x) -> MeasurementVector {
                return {x[0], x[1]};
            },
            // Measurement Jacobian H
            [](const StateVector&) -> std::array<std::array<double, 4>, 2> {
                return {{
                    {1, 0, 0, 0},  // dx/dx = 1, dx/dy = 0, dx/dvx = 0, dx/dvy = 0
                    {0, 1, 0, 0}   // dy/dx = 0, dy/dy = 1, dy/dvx = 0, dy/dvy = 0
                }};
            }
        );
    }

private:
    static StateMatrix createInitialCovariance() {
        StateMatrix P;
        for (size_t i = 0; i < 4; i++) {
            for (size_t j = 0; j < 4; j++) {
                P[i][j] = (i == j) ? 1.0 : 0.0;  // Identity with scale
            }
        }
        return P;
    }

    static StateMatrix createProcessNoise(double noise) {
        StateMatrix Q;
        for (size_t i = 0; i < 4; i++) {
            for (size_t j = 0; j < 4; j++) {
                Q[i][j] = (i == j) ? noise : 0.0;
            }
        }
        return Q;
    }

    static MeasurementMatrix createMeasurementNoise(double noise) {
        MeasurementMatrix R;
        for (size_t i = 0; i < 2; i++) {
            for (size_t j = 0; j < 2; j++) {
                R[i][j] = (i == j) ? noise : 0.0;
            }
        }
        return R;
    }
};

} // namespace estimation
} // namespace units

#endif // ROBOTICS_UNITS_ESTIMATION_H

/*
Features provided:
- Extended Kalman Filter (EKF) for nonlinear state estimation
- Template-based for any state and measurement dimensions
- User-definable state prediction and measurement models
- Automatic Jacobian-based linearization
- Pre-configured 2D position/velocity tracker
- Covariance tracking for uncertainty quantification

Use cases:
- Robot localization (position estimation from noisy sensors)
- Sensor fusion (GPS + IMU, wheel odometry + vision, etc.)
- SLAM (Simultaneous Localization and Mapping)
- Target tracking
- Battery state of charge estimation
- Any nonlinear filtering problem

Example usage:
    using namespace units::estimation;

    // Create 2D position/velocity tracker
    EKF2DPositionVelocity tracker(0.01, 0.1);  // Process and measurement noise

    // Prediction step (time update)
    tracker.predict(0.02);  // dt = 20ms

    // Measurement update (when GPS measurement arrives)
    std::array<double, 2> gps_measurement = {10.5, 5.2};  // x, y
    tracker.update(gps_measurement);

    // Get estimated state
    auto state = tracker.getState();  // [x, y, vx, vy]
    auto uncertainty = tracker.getUncertainty();  // Standard deviations

    std::cout << "Position: (" << state[0] << ", " << state[1] << ")\n";
    std::cout << "Velocity: (" << state[2] << ", " << state[3] << ")\n";
    std::cout << "Position uncertainty: ±" << uncertainty[0] << " m\n";

Custom models:
    // For custom systems, create EKF and define models:
    ExtendedKalmanFilter<STATE_DIM, MEAS_DIM> ekf(...);

    // Define state prediction function
    ekf.setStateModel(
        [](const StateVector& x, double dt) -> StateVector {
            // Your nonlinear dynamics: x_new = f(x, dt)
            return ...;
        },
        [](const StateVector& x, double dt) -> StateMatrix {
            // Jacobian of f with respect to x
            return ...;
        }
    );

    // Define measurement model
    ekf.setMeasurementModel(
        [](const StateVector& x) -> MeasurementVector {
            // Your measurement function: z = h(x)
            return ...;
        },
        [](const StateVector& x) -> MeasurementJacobian {
            // Jacobian of h with respect to x
            return ...;
        }
    );
*/
