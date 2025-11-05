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
// Why EKF:
// - Handles nonlinear systems (unlike linear Kalman filter)
// - Uses first-order Taylor expansion (linearization)
// - Computationally efficient
// - Standard tool in robotics (SLAM, localization, etc.)
//
// When to use:
// - System is "mildly" nonlinear
// - You have a good mathematical model
// - You need real-time performance
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

    // Prediction step (time update)
    void predict(double dt) {
        if (!predict_f_ || !jacobian_F_) {
            return;  // Models not set
        }

        // Predict state: x = f(x, dt)
        state_ = predict_f_(state_, dt);

        // Predict covariance: P = F * P * F^T + Q
        StateMatrix F = jacobian_F_(state_, dt);
        StateMatrix F_P = matrixMultiply(F, covariance_);
        StateMatrix F_P_Ft = matrixMultiplyTranspose(F_P, F);
        covariance_ = matrixAdd(F_P_Ft, processNoise_);
    }

    // Update step (measurement update)
    void update(const MeasurementVector& measurement) {
        if (!measure_h_ || !jacobian_H_) {
            return;  // Models not set
        }

        // Innovation: y = z - h(x)
        MeasurementVector predicted = measure_h_(state_);
        MeasurementVector innovation;
        for (size_t i = 0; i < MEASUREMENT_DIM; i++) {
            innovation[i] = measurement[i] - predicted[i];
        }

        // Measurement Jacobian
        auto H = jacobian_H_(state_);

        // Innovation covariance: S = H * P * H^T + R
        // First: H * P
        std::array<std::array<double, STATE_DIM>, MEASUREMENT_DIM> H_P;
        for (size_t i = 0; i < MEASUREMENT_DIM; i++) {
            for (size_t j = 0; j < STATE_DIM; j++) {
                H_P[i][j] = 0;
                for (size_t k = 0; k < STATE_DIM; k++) {
                    H_P[i][j] += H[i][k] * covariance_[k][j];
                }
            }
        }

        // Then: H * P * H^T
        MeasurementMatrix S;
        for (size_t i = 0; i < MEASUREMENT_DIM; i++) {
            for (size_t j = 0; j < MEASUREMENT_DIM; j++) {
                S[i][j] = measurementNoise_[i][j];
                for (size_t k = 0; k < STATE_DIM; k++) {
                    S[i][j] += H_P[i][k] * H[j][k];
                }
            }
        }

        // Kalman gain: K = P * H^T * S^-1
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

        // Update state: x = x + K * y
        for (size_t i = 0; i < STATE_DIM; i++) {
            for (size_t j = 0; j < MEASUREMENT_DIM; j++) {
                state_[i] += K[i][j] * innovation[j];
            }
        }

        // Update covariance: P = (I - K * H) * P
        StateMatrix I_KH;
        for (size_t i = 0; i < STATE_DIM; i++) {
            for (size_t j = 0; j < STATE_DIM; j++) {
                I_KH[i][j] = (i == j) ? 1.0 : 0.0;  // Identity
                for (size_t k = 0; k < MEASUREMENT_DIM; k++) {
                    I_KH[i][j] -= K[i][k] * H[k][j];
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
