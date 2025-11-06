// ============================================================================
// units_3d.h - 3D Transformations, Quaternions, and Spatial Math
// ============================================================================
// Purpose: 3D geometry, quaternions, and SE(3) transformations for robotics
// Dependencies: units_core.h, units_physics.h
//
// This file contains:
// - Quaternion class for 3D rotations
// - Vec3D for 3D vectors
// - Rotation3D using quaternions
// - Pose3D (position + orientation in 3D)
// - SE(3) transformation operations
// ============================================================================

#ifndef ROBOTICS_UNITS_3D_H
#define ROBOTICS_UNITS_3D_H

#include "units_core.h"
#include "units_physics.h"
#include <cmath>
#include <array>

namespace units {
namespace spatial {

// ============================================================================
// 3D VECTOR
// ============================================================================
// Why: Fundamental building block for 3D geometry
// - Position vectors
// - Direction vectors
// - Velocity/acceleration in 3D
// ============================================================================
class Vec3D {
public:
    double x, y, z;

    // Constructors
    constexpr Vec3D() : x(0), y(0), z(0) {}
    constexpr Vec3D(double x, double y, double z) : x(x), y(y), z(z) {}

    // Magnitude and normalization
    UNITS_CONSTEXPR14 double magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    constexpr double magnitudeSquared() const {
        return x * x + y * y + z * z;
    }

    UNITS_CONSTEXPR14 Vec3D normalized() const {
        double mag = magnitude();
        return numerical::isZero(mag) ? Vec3D() : Vec3D(x / mag, y / mag, z / mag);
    }

    // ========================================================================
    // DOT PRODUCT (3D)
    // ========================================================================
    // MATH: a · b = ax×bx + ay×by + az×bz
    //
    // GEOMETRIC MEANING:
    //   a · b = |a| × |b| × cos(θ)
    //
    // USE CASES:
    //   - Find angle between vectors: θ = acos((a·b)/(|a||b|))
    //   - Check if perpendicular: if a·b = 0
    //   - Project one vector onto another
    //   - Calculate work: W = Force · displacement
    //
    // EXAMPLE:
    //   a = (1, 2, 3)
    //   b = (4, 5, 6)
    //   a·b = 1×4 + 2×5 + 3×6 = 4 + 10 + 18 = 32
    //
    constexpr double dot(const Vec3D& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // ========================================================================
    // CROSS PRODUCT (3D) - Returns perpendicular vector
    // ========================================================================
    // WHAT: Creates vector perpendicular to both input vectors
    //
    // MATH:
    //   a × b = (ay×bz - az×by, az×bx - ax×bz, ax×by - ay×bx)
    //
    // MNEMONIC (using determinant):
    //   | i    j    k  |
    //   | ax   ay   az |
    //   | bx   by   bz |
    //
    // PROPERTIES:
    //   - Result perpendicular to both a and b
    //   - Magnitude = |a| × |b| × sin(θ)
    //   - Direction follows right-hand rule
    //   - Anti-commutative: a×b = -(b×a)
    //   - a×a = 0 (parallel vectors)
    //
    // GEOMETRIC MEANING:
    //   |a × b| = area of parallelogram formed by a and b
    //
    // RIGHT-HAND RULE:
    //   Point fingers along a, curl toward b
    //   Thumb points in direction of a×b
    //
    // EXAMPLES:
    //   1. Standard basis:
    //      (1,0,0) × (0,1,0) = (0,0,1)  ✓ (x × y = z)
    //      (0,1,0) × (0,0,1) = (1,0,0)  ✓ (y × z = x)
    //      (0,0,1) × (1,0,0) = (0,1,0)  ✓ (z × x = y)
    //
    //   2. Find normal to plane:
    //      v1 = (1,0,0), v2 = (0,1,0)
    //      normal = v1 × v2 = (0,0,1) (points up from XY plane)
    //
    //   3. Calculate torque:
    //      τ = r × F (position × force)
    //
    // USE CASES:
    //   - Find normal vector to a surface
    //   - Calculate angular momentum: L = r × p
    //   - Calculate torque: τ = r × F
    //   - Determine if vectors are parallel (cross = 0)
    //   - Find rotation axis between two orientations
    //
    constexpr Vec3D cross(const Vec3D& other) const {
        return Vec3D(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Arithmetic operators
    constexpr Vec3D operator+(const Vec3D& other) const {
        return Vec3D(x + other.x, y + other.y, z + other.z);
    }

    constexpr Vec3D operator-(const Vec3D& other) const {
        return Vec3D(x - other.x, y - other.y, z - other.z);
    }

    constexpr Vec3D operator*(double scalar) const {
        return Vec3D(x * scalar, y * scalar, z * scalar);
    }

    constexpr Vec3D operator/(double scalar) const {
        return Vec3D(x / scalar, y / scalar, z / scalar);
    }

    constexpr Vec3D operator-() const {
        return Vec3D(-x, -y, -z);
    }

    // Distance to another vector
    UNITS_CONSTEXPR14 double distanceTo(const Vec3D& other) const {
        return (*this - other).magnitude();
    }

    // Linear interpolation
    constexpr Vec3D lerp(const Vec3D& target, double t) const {
        return Vec3D(
            x + (target.x - x) * t,
            y + (target.y - y) * t,
            z + (target.z - z) * t
        );
    }

    // Component-wise operations
    constexpr Vec3D componentMultiply(const Vec3D& other) const {
        return Vec3D(x * other.x, y * other.y, z * other.z);
    }

    // Projection onto another vector
    UNITS_CONSTEXPR14 Vec3D project(const Vec3D& onto) const {
        double d = onto.dot(onto);
        return numerical::isZero(d) ? Vec3D() : onto * (this->dot(onto) / d);
    }
};

// Scalar multiplication (scalar * vector)
inline constexpr Vec3D operator*(double scalar, const Vec3D& v) {
    return v * scalar;
}

// ============================================================================
// QUATERNION
// ============================================================================
// Why quaternions over Euler angles or rotation matrices:
// - No gimbal lock
// - Compact (4 numbers vs 9 for matrix)
// - Efficient interpolation (SLERP)
// - Stable numerical properties
// - No singularities
// ============================================================================
class Quaternion {
public:
    double w, x, y, z;  // w is scalar part, (x,y,z) is vector part

    // Constructors
    constexpr Quaternion() : w(1), x(0), y(0), z(0) {}  // Identity
    constexpr Quaternion(double w, double x, double y, double z)
        : w(w), x(x), y(y), z(z) {}

    // ========================================================================
    // CREATE QUATERNION FROM AXIS-ANGLE
    // ========================================================================
    // WHAT: Create rotation quaternion from rotation axis and angle
    //
    // INPUTS:
    //   axis: Unit vector defining rotation axis (MUST be normalized!)
    //   angleRad: Rotation angle in radians (right-hand rule)
    //
    // FORMULA:
    //   q = (cos(θ/2), axis.x×sin(θ/2), axis.y×sin(θ/2), axis.z×sin(θ/2))
    //   q = (w, x, y, z)
    //
    // WHY HALF ANGLE (θ/2)?
    //   Quaternion rotation formula: v' = q × v × q*
    //   This applies the rotation TWICE (once for q, once for q*)
    //   To get rotation of θ, we need q to represent θ/2
    //   Mathematical property of quaternion algebra!
    //
    // DERIVATION:
    //   From Rodrigues' rotation formula and quaternion algebra:
    //   q = cos(θ/2) + sin(θ/2) × (axis.x×i + axis.y×j + axis.z×k)
    //   where i, j, k are quaternion basis elements
    //
    // COMPONENTS:
    //   w (scalar): cos(θ/2)
    //     - w=1 (θ=0°): no rotation
    //     - w=0 (θ=180°): half rotation
    //     - w=-1 (θ=360°): full rotation (same as no rotation!)
    //
    //   (x,y,z) (vector): axis × sin(θ/2)
    //     - Encodes both axis direction and rotation amount
    //     - Length = sin(θ/2)
    //
    // EXAMPLES:
    //
    // 1. 90° rotation around Z-axis:
    //    axis = (0, 0, 1)
    //    angle = π/2 (90°)
    //    halfAngle = π/4 (45°)
    //    w = cos(π/4) = 0.707
    //    x = 0 × sin(π/4) = 0
    //    y = 0 × sin(π/4) = 0
    //    z = 1 × sin(π/4) = 0.707
    //    q = (0.707, 0, 0, 0.707)
    //
    // 2. 180° rotation around X-axis:
    //    axis = (1, 0, 0)
    //    angle = π (180°)
    //    halfAngle = π/2 (90°)
    //    w = cos(π/2) = 0
    //    x = 1 × sin(π/2) = 1
    //    y = 0 × sin(π/2) = 0
    //    z = 0 × sin(π/2) = 0
    //    q = (0, 1, 0, 0)
    //
    // 3. Identity (no rotation):
    //    angle = 0
    //    halfAngle = 0
    //    w = cos(0) = 1
    //    x, y, z = anything × sin(0) = 0
    //    q = (1, 0, 0, 0)  ← Identity quaternion
    //
    // UNIT QUATERNION:
    //   w² + x² + y² + z² = cos²(θ/2) + sin²(θ/2) = 1
    //   All rotation quaternions have magnitude 1 (unit quaternions)
    //
    static UNITS_CONSTEXPR14 Quaternion fromAxisAngle(const Vec3D& axis, double angleRad) {
        double halfAngle = angleRad * 0.5;
        double s = std::sin(halfAngle);
        double c = std::cos(halfAngle);
        return Quaternion(c, axis.x * s, axis.y * s, axis.z * s);
    }

    // ========================================================================
    // CREATE QUATERNION FROM EULER ANGLES
    // ========================================================================
    // WHAT: Convert Euler angles (roll, pitch, yaw) to quaternion
    //
    // EULER ANGLES:
    //   Roll (φ): Rotation around X-axis (bank, tilt side-to-side)
    //   Pitch (θ): Rotation around Y-axis (nose up/down)
    //   Yaw (ψ): Rotation around Z-axis (compass heading)
    //
    // VISUAL (aircraft):
    //          Pitch (Y-axis)
    //             ↑
    //             |
    //        _____|_____
    //       /     |     \
    //      |------●------|  → Roll (X-axis)
    //       \_____Yaw____/
    //             (Z-axis down)
    //
    // CONVENTION: ZYX (Tait-Bryan angles)
    //   Apply rotations in order: Yaw → Pitch → Roll
    //   These are "intrinsic" rotations (each in the rotated frame)
    //
    // WHY THIS IS COMPLEX:
    //   Need to compose three rotations: q = q_yaw × q_pitch × q_roll
    //   Math expands into complex trig expressions
    //
    // FORMULA DERIVATION:
    //   Start with three axis-angle quaternions:
    //   q_roll  = (cos(φ/2), sin(φ/2), 0, 0)  [X-axis]
    //   q_pitch = (cos(θ/2), 0, sin(θ/2), 0)  [Y-axis]
    //   q_yaw   = (cos(ψ/2), 0, 0, sin(ψ/2))  [Z-axis]
    //
    //   Multiply: q = q_yaw × q_pitch × q_roll
    //   (Using quaternion multiplication rules)
    //
    //   Result (after expansion and simplification):
    //   w = cr×cp×cy + sr×sp×sy
    //   x = sr×cp×cy - cr×sp×sy
    //   y = cr×sp×cy + sr×cp×sy
    //   z = cr×cp×sy - sr×sp×cy
    //
    //   Where: cr=cos(roll/2), sr=sin(roll/2), etc.
    //
    // MNEMONIC (for signs):
    //   w: all same products (+ +)
    //   x: roll differs (- opposite)
    //   y: pitch differs (+ +)
    //   z: yaw differs (-)
    //
    // EXAMPLE (90° yaw, 0° pitch, 0° roll):
    //   roll=0, pitch=0, yaw=π/2
    //   cr=cos(0)=1, sr=sin(0)=0
    //   cp=cos(0)=1, sp=sin(0)=0
    //   cy=cos(π/4)=0.707, sy=sin(π/4)=0.707
    //
    //   w = 1×1×0.707 + 0×0×0.707 = 0.707
    //   x = 0×1×0.707 - 1×0×0.707 = 0
    //   y = 1×0×0.707 + 0×1×0.707 = 0
    //   z = 1×1×0.707 - 0×0×0.707 = 0.707
    //   q = (0.707, 0, 0, 0.707)  ← 90° Z-rotation
    //
    // GIMBAL LOCK WARNING:
    //   Euler angles suffer from gimbal lock at pitch = ±90°
    //   Quaternions don't have this problem!
    //   That's why we prefer quaternions for 3D rotations
    //
    static UNITS_CONSTEXPR14 Quaternion fromEuler(double roll, double pitch, double yaw) {
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);

        return Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }

    // Convert to axis-angle
    UNITS_CONSTEXPR14 void toAxisAngle(Vec3D& axis, double& angleRad) const {
        double qw_clamped = numerical::clamp(w, -1.0, 1.0);
        angleRad = 2.0 * std::acos(qw_clamped);

        double s = std::sqrt(1.0 - qw_clamped * qw_clamped);
        if (s < 0.001) {
            axis = Vec3D(1, 0, 0);  // Arbitrary axis for small angles
        } else {
            axis = Vec3D(x / s, y / s, z / s);
        }
    }

    // Convert to Euler angles (roll, pitch, yaw)
    UNITS_CONSTEXPR14 void toEuler(double& roll, double& pitch, double& yaw) const {
        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(constants::PI / 2, sinp); // Use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    // Quaternion operations
    constexpr double norm() const {
        return w * w + x * x + y * y + z * z;
    }

    UNITS_CONSTEXPR14 double magnitude() const {
        return std::sqrt(norm());
    }

    UNITS_CONSTEXPR14 Quaternion normalized() const {
        double mag = magnitude();
        return numerical::isZero(mag) ? Quaternion() :
            Quaternion(w / mag, x / mag, y / mag, z / mag);
    }

    // Conjugate (inverse for unit quaternions)
    constexpr Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    // Inverse
    UNITS_CONSTEXPR14 Quaternion inverse() const {
        double n = norm();
        return numerical::isZero(n) ? Quaternion() :
            Quaternion(w / n, -x / n, -y / n, -z / n);
    }

    // Quaternion multiplication (composition of rotations)
    constexpr Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        );
    }

    // Rotate a vector
    UNITS_CONSTEXPR14 Vec3D rotate(const Vec3D& v) const {
        // Using formula: v' = q * v * q^-1
        // Optimized version without creating intermediate quaternions
        Vec3D qv(x, y, z);
        Vec3D uv = qv.cross(v);
        Vec3D uuv = qv.cross(uv);
        uv = uv * (2.0 * w);
        uuv = uuv * 2.0;
        return v + uv + uuv;
    }

    // Spherical linear interpolation (SLERP)
    // Smooth interpolation between rotations
    UNITS_CONSTEXPR14 Quaternion slerp(const Quaternion& target, double t) const {
        Quaternion q1 = normalized();
        Quaternion q2 = target.normalized();

        double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

        // If negative dot, negate one quaternion to take shorter path
        if (dot < 0.0) {
            q2 = Quaternion(-q2.w, -q2.x, -q2.y, -q2.z);
            dot = -dot;
        }

        // If quaternions are very close, use linear interpolation
        if (dot > 0.9995) {
            return Quaternion(
                q1.w + t * (q2.w - q1.w),
                q1.x + t * (q2.x - q1.x),
                q1.y + t * (q2.y - q1.y),
                q1.z + t * (q2.z - q1.z)
            ).normalized();
        }

        // Calculate angle between quaternions
        double theta_0 = std::acos(dot);
        double theta = theta_0 * t;
        double sin_theta = std::sin(theta);
        double sin_theta_0 = std::sin(theta_0);

        double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        double s1 = sin_theta / sin_theta_0;

        return Quaternion(
            s0 * q1.w + s1 * q2.w,
            s0 * q1.x + s1 * q2.x,
            s0 * q1.y + s1 * q2.y,
            s0 * q1.z + s1 * q2.z
        );
    }

    // Get rotation angle (always positive)
    UNITS_CONSTEXPR14 double getAngle() const {
        return 2.0 * std::acos(numerical::clamp(w, -1.0, 1.0));
    }

    // Get rotation axis (for non-identity rotations)
    UNITS_CONSTEXPR14 Vec3D getAxis() const {
        double s = std::sqrt(1.0 - w * w);
        if (s < 0.001) {
            return Vec3D(1, 0, 0);  // Arbitrary for small rotations
        }
        return Vec3D(x / s, y / s, z / s);
    }
};

// ============================================================================
// POSE3D - Position and Orientation in 3D (SE(3))
// ============================================================================
// Represents a rigid body transformation in 3D space
// Combines translation (Vec3D) and rotation (Quaternion)
// ============================================================================
class Pose3D {
public:
    Vec3D position;
    Quaternion orientation;

    // Constructors
    Pose3D() : position(), orientation() {}
    Pose3D(const Vec3D& pos, const Quaternion& orient)
        : position(pos), orientation(orient.normalized()) {}

    // Transform a point from local to global frame
    UNITS_CONSTEXPR14 Vec3D transformPoint(const Vec3D& localPoint) const {
        return position + orientation.rotate(localPoint);
    }

    // Transform a point from global to local frame
    UNITS_CONSTEXPR14 Vec3D inverseTransformPoint(const Vec3D& globalPoint) const {
        return orientation.conjugate().rotate(globalPoint - position);
    }

    // Transform a direction (rotation only, no translation)
    UNITS_CONSTEXPR14 Vec3D transformDirection(const Vec3D& localDir) const {
        return orientation.rotate(localDir);
    }

    // Compose transformations (apply this, then other)
    UNITS_CONSTEXPR14 Pose3D operator*(const Pose3D& other) const {
        return Pose3D(
            position + orientation.rotate(other.position),
            orientation * other.orientation
        );
    }

    // Inverse transformation
    UNITS_CONSTEXPR14 Pose3D inverse() const {
        Quaternion inv_orient = orientation.conjugate();
        return Pose3D(
            inv_orient.rotate(-position),
            inv_orient
        );
    }

    // Relative pose from this to target
    UNITS_CONSTEXPR14 Pose3D relativeTo(const Pose3D& target) const {
        return inverse() * target;
    }

    // Distance between poses (translation only)
    UNITS_CONSTEXPR14 double distanceTo(const Pose3D& other) const {
        return position.distanceTo(other.position);
    }

    // Interpolation (SLERP for rotation, linear for translation)
    UNITS_CONSTEXPR14 Pose3D lerp(const Pose3D& target, double t) const {
        return Pose3D(
            position.lerp(target.position, t),
            orientation.slerp(target.orientation, t)
        );
    }
};

// ============================================================================
// Utility Functions
// ============================================================================

// Create rotation quaternion around X axis
inline UNITS_CONSTEXPR14 Quaternion rotationX(double angleRad) {
    return Quaternion::fromAxisAngle(Vec3D(1, 0, 0), angleRad);
}

// Create rotation quaternion around Y axis
inline UNITS_CONSTEXPR14 Quaternion rotationY(double angleRad) {
    return Quaternion::fromAxisAngle(Vec3D(0, 1, 0), angleRad);
}

// Create rotation quaternion around Z axis
inline UNITS_CONSTEXPR14 Quaternion rotationZ(double angleRad) {
    return Quaternion::fromAxisAngle(Vec3D(0, 0, 1), angleRad);
}

// Angle between two vectors
inline UNITS_CONSTEXPR14 double angleBetween(const Vec3D& a, const Vec3D& b) {
    double dot = a.dot(b);
    double mags = a.magnitude() * b.magnitude();
    if (numerical::isZero(mags)) return 0.0;
    return std::acos(numerical::clamp(dot / mags, -1.0, 1.0));
}

// Angle between two quaternions
inline UNITS_CONSTEXPR14 double angleBetween(const Quaternion& a, const Quaternion& b) {
    double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    return 2.0 * std::acos(numerical::clamp(std::abs(dot), 0.0, 1.0));
}

} // namespace spatial
} // namespace units

#endif // ROBOTICS_UNITS_3D_H

/*
Features provided:
- Vec3D: Full 3D vector math (dot, cross, magnitude, etc.)
- Quaternion: Robust 3D rotations without gimbal lock
- Pose3D: SE(3) transformations (position + orientation)
- Conversions: Euler angles, axis-angle, quaternions
- Interpolation: SLERP for smooth rotation interpolation
- All operations are constexpr where possible for compile-time computation

Use cases:
- Drone state estimation and control
- Robot arm forward/inverse kinematics in 3D
- Camera pose tracking
- IMU orientation filtering
- Multi-DOF robot transformations
- 3D path planning
- Computer vision applications

Example usage:
    using namespace units::spatial;

    // Create a pose
    Vec3D pos(1, 2, 3);
    Quaternion rot = Quaternion::fromEuler(0, 0, PI/2);  // 90° yaw
    Pose3D robot_pose(pos, rot);

    // Transform a point
    Vec3D local_point(1, 0, 0);
    Vec3D global_point = robot_pose.transformPoint(local_point);

    // Compose transformations
    Pose3D camera_to_robot(...);
    Pose3D robot_to_world(...);
    Pose3D camera_to_world = robot_to_world * camera_to_robot;

    // Smooth interpolation
    Pose3D start(...), end(...);
    Pose3D midpoint = start.lerp(end, 0.5);  // SLERP + linear
*/
