// ============================================================================
// RobotLib Fluent API - Easy-to-use chaining interface
// ============================================================================
// High-level fluent API that wraps RobotLib's type-safe units system.
// Every configuration method returns *this for chaining.
//
// Example usage:
//   Arm leftArm = Arm()
//       .withPID(1.0, 0.1, 0.05)
//       .withLimits(deg(-90), deg(90))
//       .withFeedforward(0.5, 0.002);
//
//   leftArm.moveTo(deg(90))
//          .setSpeed(0.8)
//          .waitUntilAtTarget();
//
// Part of RobotLib v2.2
// C++11 compatible, header-only, chainable API
// ============================================================================

#ifndef ROBOTLIB_API_H
#define ROBOTLIB_API_H

#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"
#include <cmath>

namespace robotlib {

using namespace units;
using namespace units::robotics;

// ============================================================================
// Motor/Arm Controller - Fluent API
// ============================================================================
class Arm {
private:
    // Configuration
    Radians min_angle_;
    Radians max_angle_;
    double speed_;
    bool has_limits_;

    // PID configuration
    PIDController pid_;
    double kP_, kI_, kD_;
    bool pid_enabled_;

    // Feedforward configuration
    double kS_;  // Static friction
    double kV_;  // Velocity feedforward
    bool ff_enabled_;

    // Command state (what we commanded)
    Radians target_position_;
    double duty_cycle_;

    // Measured state (sensor feedback)
    Radians current_position_;      // Actual measured position
    RadiansPerSecond current_velocity_;  // Actual measured velocity

public:
    // Constructor
    Arm()
        : min_angle_(Radians::fromRadians(-M_PI))
        , max_angle_(Radians::fromRadians(M_PI))
        , speed_(1.0)
        , has_limits_(false)
        , pid_(PIDController::Gains(1.0, 0.0, 0.0))
        , kP_(1.0), kI_(0.0), kD_(0.0)
        , pid_enabled_(false)
        , kS_(0.0), kV_(0.0)
        , ff_enabled_(false)
        , target_position_(Radians::fromRadians(0.0))
        , duty_cycle_(0.0)
        , current_position_(Radians::fromRadians(0.0))
        , current_velocity_(radps(0.0))
    {}

    // ========================================================================
    // Configuration Methods (return *this for chaining)
    // ========================================================================

    Arm& withPID(double kP, double kI, double kD) {
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;

        PIDController::Gains gains(kP, kI, kD);
        gains.outputMin = -1.0;
        gains.outputMax = 1.0;
        pid_ = PIDController(gains);
        pid_enabled_ = true;

        return *this;
    }

    Arm& withExponentialPID(double kP, double kI, double kD, double exponent = 2.0) {
        (void)exponent;  // Reserved for future exponential control implementation
        withPID(kP, kI, kD);
        return *this;
    }

    Arm& withFeedforward(double kS, double kV) {
        kS_ = kS;
        kV_ = kV;
        ff_enabled_ = true;
        return *this;
    }

    template<typename AngleRatio>
    Arm& withLimits(const Angle<AngleRatio>& min_angle, const Angle<AngleRatio>& max_angle) {
        min_angle_ = Radians::fromRadians(min_angle.toRadians());
        max_angle_ = Radians::fromRadians(max_angle.toRadians());
        has_limits_ = true;
        return *this;
    }

    Arm& withoutLimits() {
        has_limits_ = false;
        return *this;
    }

    Arm& enablePID(bool enable = true) {
        pid_enabled_ = enable;
        return *this;
    }

    Arm& disablePID() {
        pid_enabled_ = false;
        return *this;
    }

    // ========================================================================
    // Control Methods (return *this for chaining where sensible)
    // ========================================================================

    template<typename AngleRatio>
    Arm& moveTo(const Angle<AngleRatio>& target) {
        target_position_ = Radians::fromRadians(target.toRadians());

        // Clamp to limits if enabled
        if (has_limits_) {
            if (target_position_.toRadians() < min_angle_.toRadians()) {
                target_position_ = min_angle_;
            }
            if (target_position_.toRadians() > max_angle_.toRadians()) {
                target_position_ = max_angle_;
            }
        }

        return *this;
    }

    template<typename AngleRatio>
    Arm& moveBy(const Angle<AngleRatio>& delta) {
        return moveTo(Radians::fromRadians(
            current_position_.toRadians() + delta.toRadians()
        ));
    }

    Arm& setSpeed(double speed) {
        speed_ = speed;
        if (speed_ < 0.0) speed_ = 0.0;
        if (speed_ > 1.0) speed_ = 1.0;
        return *this;
    }

    Arm& setDutyCycle(double duty) {
        duty_cycle_ = duty;
        if (duty_cycle_ < -1.0) duty_cycle_ = -1.0;
        if (duty_cycle_ > 1.0) duty_cycle_ = 1.0;
        return *this;
    }

    Arm& stop() {
        duty_cycle_ = 0.0;
        target_position_ = current_position_;
        return *this;
    }

    Arm& brake() {
        return stop();
    }

    template<typename AngleRatio>
    Arm& resetPosition(const Angle<AngleRatio>& position) {
        current_position_ = Radians::fromRadians(position.toRadians());
        target_position_ = Radians::fromRadians(position.toRadians());
        return *this;
    }

    // ========================================================================
    // UPDATE - Main control loop (with position sensor only)
    // ========================================================================
    // WHAT: Called every loop iteration to update arm control
    //
    // INPUTS:
    //   dt: Time since last update (seconds)
    //   measured_position: Current angle from encoder/potentiometer
    //
    // ALGORITHM:
    //   1. Calculate velocity from position change (numerical derivative)
    //   2. Calculate PID control output
    //   3. Add feedforward compensation
    //   4. Clamp to safe limits
    //
    // ========================================================================
    template<typename AngleRatio>
    Arm& update(double dt, const Angle<AngleRatio>& measured_position) {
        // ====================================================================
        // STEP 1: CALCULATE VELOCITY (Numerical Differentiation)
        // ====================================================================
        // FORMULA: velocity = Δposition / Δtime
        //
        // MATH:
        //   ω = (θ_new - θ_old) / dt
        //
        // EXAMPLE:
        //   Old position: 30° = 0.524 rad
        //   New position: 35° = 0.611 rad
        //   Time step: dt = 0.02 seconds (50Hz control loop)
        //   Velocity = (0.611 - 0.524) / 0.02 = 4.35 rad/s
        //
        // WHY: We need velocity for derivative term and feedforward
        //
        double prev_pos = current_position_.toRadians();
        current_position_ = Radians::fromRadians(measured_position.toRadians());
        double new_pos = current_position_.toRadians();
        current_velocity_ = RadiansPerSecond::fromRadiansPerSecond((new_pos - prev_pos) / dt);

        if (pid_enabled_) {
            // ================================================================
            // STEP 2: PID CONTROL
            // ================================================================
            // ERROR = Target - Current (how far off we are)
            //
            // EXAMPLE:
            //   Target: 90°
            //   Current: 75°
            //   Error: 90° - 75° = 15° (need to move 15° more)
            //
            // SIGN:
            //   Positive error: arm is below target (move up)
            //   Negative error: arm is above target (move down)
            //
            double error = target_position_.toRadians() - current_position_.toRadians();

            // PID calculates control output [-1, 1]
            // Then scale by speed limit (user-defined 0-1)
            //
            // EXAMPLE:
            //   PID output: 0.8 (80% power)
            //   Speed limit: 0.5 (user set max 50% speed)
            //   Final: 0.8 × 0.5 = 0.4 (40% duty cycle)
            //
            duty_cycle_ = pid_.calculate(error, dt) * speed_;

            // ================================================================
            // STEP 3: FEEDFORWARD COMPENSATION
            // ================================================================
            // WHY: PID reacts to error AFTER it happens
            //      Feedforward predicts what control is needed
            //
            // TWO TERMS:
            //
            // 1. STATIC FRICTION (kS):
            //    Constant force needed to overcome friction
            //    SIGN: Direction depends on which way we're moving
            //    EXAMPLE: kS=0.05 means always need 5% just to start moving
            //
            // 2. VELOCITY FEEDFORWARD (kV):
            //    More power needed for faster motion
            //    FORMULA: kV × desired_velocity
            //    EXAMPLE: kV=0.001, velocity=10 rad/s → 0.01 (1% more power)
            //
            // TOTAL FEEDFORWARD = kS×sign(error) + kV×velocity
            //
            if (ff_enabled_) {
                // Estimate desired velocity from error
                // (Simplified: error/dt gives crude velocity estimate)
                double velocity = error / dt;

                // Static friction: overcome stiction
                // Sign: +1 if moving forward, -1 if backward
                double friction_comp = kS_ * (error > 0 ? 1.0 : -1.0);

                // Velocity feedforward: more power for faster motion
                double velocity_comp = kV_ * velocity;

                duty_cycle_ += friction_comp + velocity_comp;
            }
        }

        // ====================================================================
        // STEP 4: SATURATION (Safety Limits)
        // ====================================================================
        // PROBLEM: Can't command more than 100% power or less than -100%
        // SOLUTION: Clamp to [-1.0, 1.0]
        //
        // WHY: Protects motor from over-voltage and ensures safe operation
        //
        if (duty_cycle_ < -1.0) duty_cycle_ = -1.0;
        if (duty_cycle_ > 1.0) duty_cycle_ = 1.0;

        return *this;
    }

    // Update with measured velocity (if available from sensor)
    template<typename AngleRatio, typename AngularVelocityRatio>
    Arm& update(double dt, const Angle<AngleRatio>& measured_position,
                const AngularVelocity<AngularVelocityRatio>& measured_velocity) {
        current_position_ = Radians::fromRadians(measured_position.toRadians());
        current_velocity_ = RadiansPerSecond::fromRadiansPerSecond(measured_velocity.toRadiansPerSecond());

        if (pid_enabled_) {
            double error = target_position_.toRadians() - current_position_.toRadians();
            duty_cycle_ = pid_.calculate(error, dt) * speed_;

            // Add feedforward if enabled
            if (ff_enabled_) {
                double velocity = error / dt;  // Simplified
                duty_cycle_ += kS_ * (error > 0 ? 1.0 : -1.0) + kV_ * velocity;
            }
        }

        // Clamp output
        if (duty_cycle_ < -1.0) duty_cycle_ = -1.0;
        if (duty_cycle_ > 1.0) duty_cycle_ = 1.0;

        return *this;
    }

    // ========================================================================
    // Query Methods (return values, end chain)
    // ========================================================================

    // Configuration
    Radians getMinAngle() const { return min_angle_; }
    Radians getMaxAngle() const { return max_angle_; }
    double getSpeed() const { return speed_; }
    bool hasLimits() const { return has_limits_; }

    // PID configuration
    double getKP() const { return kP_; }
    double getKI() const { return kI_; }
    double getKD() const { return kD_; }
    bool isPIDEnabled() const { return pid_enabled_; }

    // Feedforward configuration
    double getKS() const { return kS_; }
    double getKV() const { return kV_; }
    bool isFeedforwardEnabled() const { return ff_enabled_; }

    // Command state (what we commanded)
    Radians getTarget() const { return target_position_; }
    double getDutyCycle() const { return duty_cycle_; }

    // Measured state (sensor feedback)
    Radians getPosition() const { return current_position_; }
    RadiansPerSecond getVelocity() const { return current_velocity_; }

    // Calculated status
    double getError() const {
        return target_position_.toRadians() - current_position_.toRadians();
    }

    bool isAtTarget(double tolerance_deg = 2.0) const {
        double tolerance_rad = tolerance_deg * M_PI / 180.0;
        return std::abs(getError()) < tolerance_rad;
    }

    bool isMoving(double threshold = 0.01) const {
        return std::abs(duty_cycle_) > threshold;
    }
};

// ============================================================================
// Differential Drive - Fluent API
// ============================================================================
class DifferentialDrive {
private:
    // Configuration (robot hardware parameters)
    Meters wheelbase_;
    Meters wheel_diameter_;
    MetersPerSecond max_speed_;

    // Command state (what we commanded the robot to do)
    double left_duty_;
    double right_duty_;
    MetersPerSecond linear_velocity_;
    RadiansPerSecond angular_velocity_;

    // Odometry state (where the robot thinks it is)
    double x_position_;      // meters
    double y_position_;      // meters
    double theta_;           // radians (heading)

    // Measured state (sensor feedback)
    Meters left_distance_;   // Total distance traveled by left wheel
    Meters right_distance_;  // Total distance traveled by right wheel
    MetersPerSecond left_velocity_;   // Measured left wheel velocity
    MetersPerSecond right_velocity_;  // Measured right wheel velocity

public:
    DifferentialDrive()
        : wheelbase_(m(0.5))
        , wheel_diameter_(m(0.1))
        , max_speed_(mps(1.0))
        , left_duty_(0.0)
        , right_duty_(0.0)
        , linear_velocity_(mps(0.0))
        , angular_velocity_(radps(0.0))
        , x_position_(0.0)
        , y_position_(0.0)
        , theta_(0.0)
        , left_distance_(m(0.0))
        , right_distance_(m(0.0))
        , left_velocity_(mps(0.0))
        , right_velocity_(mps(0.0))
    {}

    // ========================================================================
    // Configuration
    // ========================================================================

    DifferentialDrive& withWheelbase(const Meters& wheelbase) {
        wheelbase_ = wheelbase;
        return *this;
    }

    DifferentialDrive& withWheelDiameter(const Meters& diameter) {
        wheel_diameter_ = diameter;
        return *this;
    }

    DifferentialDrive& withMaxSpeed(const MetersPerSecond& max_speed) {
        max_speed_ = max_speed;
        return *this;
    }

    // ========================================================================
    // Control
    // ========================================================================

    // ========================================================================
    // DRIVE - Differential Drive Kinematics
    // ========================================================================
    // WHAT: Convert desired robot motion to individual wheel speeds
    //
    // INPUTS:
    //   linear: Forward/backward speed (m/s)
    //   angular: Rotation speed (rad/s)
    //
    // OUTPUTS:
    //   left_duty_, right_duty_: Motor power for each wheel
    //
    // DIFFERENTIAL DRIVE MATH:
    //   Robot has two wheels separated by wheelbase L
    //   To move forward: both wheels same speed
    //   To turn: wheels at different speeds
    //
    // FORMULAS:
    //   v_left = v - (ω × L/2)
    //   v_right = v + (ω × L/2)
    //
    //   where:
    //   v = linear velocity (m/s)
    //   ω = angular velocity (rad/s)
    //   L = wheelbase (distance between wheels)
    //
    // ========================================================================
    DifferentialDrive& drive(const MetersPerSecond& linear, const RadiansPerSecond& angular) {
        linear_velocity_ = linear;
        angular_velocity_ = angular;

        // ====================================================================
        // DIFFERENTIAL DRIVE KINEMATICS
        // ====================================================================
        // CONCEPT: To turn, one wheel goes faster than the other
        //
        // VISUAL (top view of robot):
        //       Forward (v)
        //           ↑
        //      [L]  ●  [R]    L = Left wheel
        //       |←─→|         R = Right wheel
        //         L           ← = Wheelbase
        //
        // STRAIGHT MOTION (ω=0):
        //   Both wheels same speed: v_L = v_R = v
        //   Robot moves straight forward
        //
        // PURE ROTATION (v=0):
        //   Wheels rotate opposite: v_L = -ω×L/2, v_R = +ω×L/2
        //   Robot spins in place
        //
        // COMBINED (v≠0, ω≠0):
        //   Robot follows a curved path (arc)
        //
        // ====================================================================

        double v = linear.toMetersPerSecond();
        double w = angular.toRadiansPerSecond();
        double L = wheelbase_.toMeters();

        // ====================================================================
        // WHEEL VELOCITY CALCULATION
        // ====================================================================
        // DERIVATION:
        //   When robot turns, it follows circular arc with radius R
        //   Left wheel follows smaller circle: R - L/2
        //   Right wheel follows larger circle: R + L/2
        //
        // MATH:
        //   v_left = v - (ω × L/2)
        //   v_right = v + (ω × L/2)
        //
        // WHY SUBTRACT/ADD L/2?
        //   ω × (L/2) = speed difference needed for that wheel
        //   Left wheel is closer to turn center → slower
        //   Right wheel is farther from turn center → faster
        //
        // EXAMPLES:
        //
        // 1. STRAIGHT FORWARD (v=1 m/s, ω=0, L=0.5m):
        //    v_left = 1 - (0 × 0.25) = 1 m/s
        //    v_right = 1 + (0 × 0.25) = 1 m/s
        //    Both wheels same → straight
        //
        // 2. TURN LEFT (v=1 m/s, ω=2 rad/s, L=0.5m):
        //    v_left = 1 - (2 × 0.25) = 1 - 0.5 = 0.5 m/s
        //    v_right = 1 + (2 × 0.25) = 1 + 0.5 = 1.5 m/s
        //    Left slower, right faster → curves left
        //
        // 3. SPIN IN PLACE (v=0, ω=3 rad/s, L=0.5m):
        //    v_left = 0 - (3 × 0.25) = -0.75 m/s (backward!)
        //    v_right = 0 + (3 × 0.25) = 0.75 m/s (forward)
        //    Wheels oppose → spins clockwise
        //
        double left_vel = v - (w * L / 2.0);
        double right_vel = v + (w * L / 2.0);

        // ====================================================================
        // CONVERT TO DUTY CYCLE (Motor Power)
        // ====================================================================
        // DUTY CYCLE: Fraction of max speed (-1.0 to 1.0)
        //   -1.0 = full power backward
        //    0.0 = stopped
        //   +1.0 = full power forward
        //
        // MATH:
        //   duty = velocity / max_velocity
        //
        // EXAMPLE:
        //   left_vel = 0.5 m/s
        //   max_speed = 2.0 m/s
        //   left_duty = 0.5 / 2.0 = 0.25 (25% power)
        //
        left_duty_ = left_vel / max_speed_.toMetersPerSecond();
        right_duty_ = right_vel / max_speed_.toMetersPerSecond();

        // ====================================================================
        // SATURATION (Safety Limits)
        // ====================================================================
        // PROBLEM: Calculated duty might exceed ±1.0
        //   Happens when turning very fast or max_speed too low
        //
        // SOLUTION: Clamp to safe range
        //   This might reduce turn rate slightly but prevents motor damage
        //
        if (left_duty_ < -1.0) left_duty_ = -1.0;
        if (left_duty_ > 1.0) left_duty_ = 1.0;
        if (right_duty_ < -1.0) right_duty_ = -1.0;
        if (right_duty_ > 1.0) right_duty_ = 1.0;

        return *this;
    }

    DifferentialDrive& arcade(double forward, double turn) {
        // Arcade drive: forward/backward + turn
        return drive(
            MetersPerSecond::fromMetersPerSecond(forward * max_speed_.toMetersPerSecond()),
            RadiansPerSecond::fromRadiansPerSecond(turn * 2.0)  // Scale turning
        );
    }

    DifferentialDrive& tank(double left, double right) {
        left_duty_ = left;
        right_duty_ = right;

        // Clamp
        if (left_duty_ < -1.0) left_duty_ = -1.0;
        if (left_duty_ > 1.0) left_duty_ = 1.0;
        if (right_duty_ < -1.0) right_duty_ = -1.0;
        if (right_duty_ > 1.0) right_duty_ = 1.0;

        return *this;
    }

    DifferentialDrive& stop() {
        left_duty_ = 0.0;
        right_duty_ = 0.0;
        linear_velocity_ = mps(0.0);
        angular_velocity_ = radps(0.0);
        return *this;
    }

    DifferentialDrive& brake() {
        return stop();
    }

    // ========================================================================
    // State Update (sensor feedback and odometry)
    // ========================================================================

    DifferentialDrive& updateEncoders(const Meters& left_dist, const Meters& right_dist) {
        left_distance_ = left_dist;
        right_distance_ = right_dist;
        return *this;
    }

    DifferentialDrive& updateVelocities(const MetersPerSecond& left_vel, const MetersPerSecond& right_vel) {
        left_velocity_ = left_vel;
        right_velocity_ = right_vel;
        return *this;
    }

    // ========================================================================
    // UPDATE ODOMETRY - Track robot position from wheel encoders
    // ========================================================================
    // WHAT: Estimate robot's global position from wheel movement
    //
    // INPUTS:
    //   dt: Time since last update (seconds)
    //   (Uses left_velocity_ and right_velocity_ from sensors)
    //
    // OUTPUTS:
    //   Updates: x_position_, y_position_, theta_ (robot pose)
    //
    // WHY: GPS doesn't work indoors, odometry is the fundamental way
    //      mobile robots know where they are
    //
    // ACCURACY: Drifts over time due to wheel slip, but essential for
    //           short-term navigation and combining with other sensors
    //
    // ========================================================================
    DifferentialDrive& updateOdometry(double dt) {
        // ====================================================================
        // STEP 1: CALCULATE WHEEL DISPLACEMENT
        // ====================================================================
        // FORMULA: distance = velocity × time
        //
        // MATH:
        //   Δd_left = v_left × dt
        //   Δd_right = v_right × dt
        //
        // EXAMPLE:
        //   Left wheel velocity: 0.5 m/s
        //   Right wheel velocity: 0.7 m/s
        //   Time step: dt = 0.02 s (50Hz update rate)
        //
        //   left_delta = 0.5 × 0.02 = 0.01 m (1 cm)
        //   right_delta = 0.7 × 0.02 = 0.014 m (1.4 cm)
        //
        double left_vel = left_velocity_.toMetersPerSecond();
        double right_vel = right_velocity_.toMetersPerSecond();

        double left_delta = left_vel * dt;
        double right_delta = right_vel * dt;

        // ====================================================================
        // STEP 2: CALCULATE ROBOT CENTER DISPLACEMENT AND ROTATION
        // ====================================================================
        //
        // LINEAR DISTANCE (robot center):
        //   FORMULA: distance = (left_delta + right_delta) / 2
        //
        //   WHY: Robot center is halfway between wheels
        //        Average of two wheels gives center movement
        //
        //   EXAMPLE:
        //     left_delta = 0.01 m
        //     right_delta = 0.014 m
        //     distance = (0.01 + 0.014) / 2 = 0.012 m (robot moved 1.2 cm)
        //
        // ROTATION (heading change):
        //   FORMULA: dθ = (right_delta - left_delta) / wheelbase
        //
        //   DERIVATION:
        //     When wheels travel different distances, robot rotates
        //     Arc length difference = right_delta - left_delta
        //     This arc is subtended by wheelbase radius
        //     Therefore: dθ = arc_difference / wheelbase
        //
        //   WHY THIS WORKS:
        //     Imagine right wheel goes 1m, left wheel goes 0m
        //     Robot pivots around left wheel
        //     Right wheel traces arc of length 1m at radius = wheelbase
        //     Angle = arc_length / radius = 1m / wheelbase
        //
        //   EXAMPLE:
        //     left_delta = 0.01 m
        //     right_delta = 0.014 m
        //     wheelbase = 0.5 m
        //     dθ = (0.014 - 0.01) / 0.5 = 0.004 / 0.5 = 0.008 rad
        //     = 0.008 × 180/π = 0.46° (small turn to the left)
        //
        //   SIGN CONVENTION:
        //     Positive dθ: counterclockwise rotation (right faster)
        //     Negative dθ: clockwise rotation (left faster)
        //     Zero dθ: straight motion (wheels same speed)
        //
        double distance = (left_delta + right_delta) / 2.0;
        double dtheta = (right_delta - left_delta) / wheelbase_.toMeters();

        // ====================================================================
        // STEP 3: UPDATE GLOBAL POSITION
        // ====================================================================
        //
        // COORDINATE SYSTEM:
        //   X-axis: points "forward" in world frame
        //   Y-axis: points "left" in world frame
        //   θ (theta): robot heading angle from X-axis
        //
        // VISUAL (top view):
        //        Y (left)
        //        ↑
        //        |
        //        |   θ
        //        |  ╱
        //        | ╱ robot
        //        |╱
        //   ─────●────────→ X (forward)
        //      (0,0)
        //
        // ROTATION UPDATE:
        //   θ_new = θ_old + dθ
        //   Simply accumulate rotation
        //
        // POSITION UPDATE (using heading):
        //   Robot moved 'distance' meters in direction θ
        //   Break into X and Y components using trig:
        //
        //   dx = distance × cos(θ)
        //   dy = distance × sin(θ)
        //
        //   x_new = x_old + dx
        //   y_new = y_old + dy
        //
        // WHY THIS ORDER:
        //   Update theta FIRST, then use it for position
        //   Ensures position uses the average heading during motion
        //
        // EXAMPLE:
        //   Current: (x=1.0, y=2.0, θ=90° = π/2)
        //   Distance moved: 0.1 m
        //   Heading change: dθ = 0.1 rad
        //
        //   New heading: θ = π/2 + 0.1 = 1.67 rad ≈ 96°
        //   dx = 0.1 × cos(1.67) = 0.1 × (-0.08) = -0.008 m
        //   dy = 0.1 × sin(1.67) = 0.1 × (0.996) = 0.0996 m
        //   New position: (0.992, 2.0996, 96°)
        //
        // NOTE ON ACCURACY:
        //   This is a simplified model (assumes small angle changes)
        //   For large dt or fast turns, use more sophisticated integration
        //   But this works well for typical control loop rates (>20Hz)
        //
        theta_ += dtheta;
        x_position_ += distance * std::cos(theta_);
        y_position_ += distance * std::sin(theta_);

        return *this;
    }

    DifferentialDrive& resetOdometry(double x = 0.0, double y = 0.0, double theta = 0.0) {
        x_position_ = x;
        y_position_ = y;
        theta_ = theta;
        left_distance_ = m(0.0);
        right_distance_ = m(0.0);
        return *this;
    }

    // ========================================================================
    // Query
    // ========================================================================

    // Configuration (robot hardware parameters)
    Meters getWheelbase() const { return wheelbase_; }
    Meters getWheelDiameter() const { return wheel_diameter_; }
    MetersPerSecond getMaxSpeed() const { return max_speed_; }

    // Command state (what we commanded)
    double getLeftDuty() const { return left_duty_; }
    double getRightDuty() const { return right_duty_; }
    MetersPerSecond getLinearVelocity() const { return linear_velocity_; }
    RadiansPerSecond getAngularVelocity() const { return angular_velocity_; }

    // Odometry state (where the robot thinks it is)
    double getX() const { return x_position_; }
    double getY() const { return y_position_; }
    double getTheta() const { return theta_; }
    double getThetaDegrees() const { return theta_ * 180.0 / M_PI; }

    // Measured state (sensor feedback)
    Meters getLeftDistance() const { return left_distance_; }
    Meters getRightDistance() const { return right_distance_; }
    MetersPerSecond getLeftVelocity() const { return left_velocity_; }
    MetersPerSecond getRightVelocity() const { return right_velocity_; }

    // Status
    bool isStopped(double threshold = 0.01) const {
        return std::abs(left_duty_) < threshold && std::abs(right_duty_) < threshold;
    }
};

// ============================================================================
// Sensor with Filtering - Fluent API
// ============================================================================
class Sensor {
private:
    double raw_value_;
    double filtered_value_;
    MovingAverageFilter<5> ma_filter_;
    LowPassFilter lpf_;
    bool use_ma_;
    bool use_lpf_;
    double lpf_alpha_;

public:
    Sensor()
        : raw_value_(0.0)
        , filtered_value_(0.0)
        , lpf_(0.5)
        , use_ma_(false)
        , use_lpf_(false)
        , lpf_alpha_(0.5)
    {}

    // ========================================================================
    // Configuration
    // ========================================================================

    Sensor& withMovingAverage(bool enable = true) {
        use_ma_ = enable;
        return *this;
    }

    Sensor& withLowPassFilter(double alpha) {
        lpf_alpha_ = alpha;
        lpf_ = LowPassFilter(alpha);
        use_lpf_ = true;
        return *this;
    }

    Sensor& withoutFiltering() {
        use_ma_ = false;
        use_lpf_ = false;
        return *this;
    }

    // ========================================================================
    // Control/Update
    // ========================================================================

    Sensor& read(double value) {
        raw_value_ = value;
        filtered_value_ = raw_value_;

        if (use_ma_) {
            filtered_value_ = ma_filter_.update(filtered_value_);
        }

        if (use_lpf_) {
            filtered_value_ = lpf_.update(filtered_value_);
        }

        return *this;
    }

    Sensor& reset() {
        raw_value_ = 0.0;
        filtered_value_ = 0.0;
        return *this;
    }

    // ========================================================================
    // Query
    // ========================================================================

    // Current values
    double getValue() const { return filtered_value_; }
    double getRawValue() const { return raw_value_; }

    // Configuration
    bool isMovingAverageEnabled() const { return use_ma_; }
    bool isLowPassEnabled() const { return use_lpf_; }
    double getLowPassAlpha() const { return lpf_alpha_; }
};

// ============================================================================
// PID Controller Wrapper - Fluent API
// ============================================================================
class PID {
private:
    PIDController controller_;
    double kP_, kI_, kD_;
    double output_;
    double error_;

public:
    PID()
        : controller_(PIDController::Gains(1.0, 0.0, 0.0))
        , kP_(1.0), kI_(0.0), kD_(0.0)
        , output_(0.0)
        , error_(0.0)
    {}

    // ========================================================================
    // Configuration
    // ========================================================================

    PID& withGains(double kP, double kI, double kD) {
        kP_ = kP;
        kI_ = kI;
        kD_ = kD;

        PIDController::Gains gains(kP, kI, kD);
        controller_ = PIDController(gains);

        return *this;
    }

    PID& withOutputLimits(double min, double max) {
        PIDController::Gains gains(kP_, kI_, kD_);
        gains.outputMin = min;
        gains.outputMax = max;
        controller_ = PIDController(gains);
        return *this;
    }

    PID& withIntegralLimit(double max) {
        PIDController::Gains gains(kP_, kI_, kD_);
        gains.iMax = max;
        controller_ = PIDController(gains);
        return *this;
    }

    // ========================================================================
    // Control
    // ========================================================================

    PID& calculate(double error, double dt) {
        error_ = error;
        output_ = controller_.calculate(error, dt);
        return *this;
    }

    PID& reset() {
        controller_.reset();
        output_ = 0.0;
        error_ = 0.0;
        return *this;
    }

    // ========================================================================
    // Query
    // ========================================================================

    // Current values
    double getOutput() const { return output_; }
    double getError() const { return error_; }

    // Configuration
    double getKP() const { return kP_; }
    double getKI() const { return kI_; }
    double getKD() const { return kD_; }
};

} // namespace robotlib

#endif // ROBOTLIB_API_H
