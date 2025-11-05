// ============================================================================
// RobotLib - ROS2 Interoperability
// ============================================================================
// Helper functions and structures for converting between RobotLib units and
// ROS2 message types. This header provides the conversion logic WITHOUT
// requiring ROS2 to be installed.
//
// When using with actual ROS2:
// 1. Include this header after ROS2 headers
// 2. Use the conversion functions to go to/from ROS2 messages
// 3. All conversions preserve units correctly
//
// Supported ROS2 message types:
// - geometry_msgs/msg/Point, Vector3, Pose, Twist, Quaternion
// - sensor_msgs/msg/Imu
// - nav_msgs/msg/Odometry
// - std_msgs/msg/Float64
//
// Part of RobotLib v2.2
// C++11 compatible, header-only, zero-overhead design
// ============================================================================

#ifndef ROBOTLIB_UNITS_ROS2_INTEROP_H
#define ROBOTLIB_UNITS_ROS2_INTEROP_H

#include "units_core.h"
#include "units_physics.h"
#include "units_robotics.h"
#include "units_3d.h"

// Optional: If ROS2 is available, we can provide direct conversions
#ifdef ROS2_AVAILABLE
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#endif

namespace units {
namespace ros2 {

// ============================================================================
// Simple Data Structures (ROS2-compatible layout)
// ============================================================================
// These mirror ROS2 message structures but don't require ROS2 to be installed

struct Point {
    double x;
    double y;
    double z;
};

struct Vector3 {
    double x;
    double y;
    double z;
};

struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

struct Pose {
    Point position;
    Quaternion orientation;
};

struct Twist {
    Vector3 linear;
    Vector3 angular;
};

struct Imu {
    Quaternion orientation;
    Vector3 angular_velocity;  // rad/s
    Vector3 linear_acceleration;  // m/s²
};

struct Odometry {
    Pose pose;
    Twist twist;
};

// ============================================================================
// Conversion Functions: RobotLib → ROS2 Structures
// ============================================================================

// Convert Vec3D to ROS2 Point
inline Point toPoint(const spatial::Vec3D& v) {
    return Point{v.x, v.y, v.z};
}

// Convert Vec3D to ROS2 Vector3
inline Vector3 toVector3(const spatial::Vec3D& v) {
    return Vector3{v.x, v.y, v.z};
}

// Convert Quaternion to ROS2 Quaternion
inline Quaternion toQuaternion(const spatial::Quaternion& q) {
    return Quaternion{q.x, q.y, q.z, q.w};
}

// Convert Pose3D to ROS2 Pose
inline Pose toPose(const spatial::Pose3D& pose) {
    return Pose{
        toPoint(pose.position),
        toQuaternion(pose.orientation)
    };
}

// Convert Vec2D to ROS2 Point (z=0)
inline Point toPoint2D(const robotics::Vec2D& v) {
    return Point{v.x, v.y, 0.0};
}

// Convert Pose2D to ROS2 Pose
inline Pose toPose2D(const robotics::Pose2D& pose) {
    // Create quaternion from heading angle (rotation around Z-axis)
    auto q = spatial::Quaternion::fromEuler(0.0, 0.0, pose.theta.toRadians());
    return Pose{
        Point{pose.position.x, pose.position.y, 0.0},
        toQuaternion(q)
    };
}

// Convert velocity to ROS2 Vector3 (m/s)
inline Vector3 toVector3Velocity(const MetersPerSecond& vx,
                                 const MetersPerSecond& vy,
                                 const MetersPerSecond& vz) {
    return Vector3{
        vx.toMetersPerSecond(),
        vy.toMetersPerSecond(),
        vz.toMetersPerSecond()
    };
}

// Convert angular velocity to ROS2 Vector3 (rad/s)
inline Vector3 toVector3Angular(const RadiansPerSecond& wx,
                                const RadiansPerSecond& wy,
                                const RadiansPerSecond& wz) {
    return Vector3{
        wx.toRadiansPerSecond(),
        wy.toRadiansPerSecond(),
        wz.toRadiansPerSecond()
    };
}

// Convert acceleration to ROS2 Vector3 (m/s²)
inline Vector3 toVector3Accel(const MetersPerSecondSquared& ax,
                              const MetersPerSecondSquared& ay,
                              const MetersPerSecondSquared& az) {
    return Vector3{
        ax.toMetersPerSecondSquared(),
        ay.toMetersPerSecondSquared(),
        az.toMetersPerSecondSquared()
    };
}

// ============================================================================
// Conversion Functions: ROS2 Structures → RobotLib
// ============================================================================

// Convert ROS2 Point to Vec3D
inline spatial::Vec3D fromPoint(const Point& p) {
    return spatial::Vec3D(p.x, p.y, p.z);
}

// Convert ROS2 Vector3 to Vec3D
inline spatial::Vec3D fromVector3(const Vector3& v) {
    return spatial::Vec3D(v.x, v.y, v.z);
}

// Convert ROS2 Quaternion to RobotLib Quaternion
inline spatial::Quaternion fromQuaternion(const Quaternion& q) {
    return spatial::Quaternion(q.w, q.x, q.y, q.z);
}

// Convert ROS2 Pose to Pose3D
inline spatial::Pose3D fromPose(const Pose& pose) {
    return spatial::Pose3D(
        fromPoint(pose.position),
        fromQuaternion(pose.orientation)
    );
}

// Convert ROS2 Point to Vec2D (ignoring z)
inline robotics::Vec2D fromPoint2D(const Point& p) {
    return robotics::Vec2D(p.x, p.y);
}

// Convert ROS2 Pose to Pose2D (using yaw from quaternion)
inline robotics::Pose2D fromPose2D(const Pose& pose) {
    auto q = fromQuaternion(pose.orientation);
    double roll, pitch, yaw;
    q.toEuler(roll, pitch, yaw);

    return robotics::Pose2D(
        pose.position.x,
        pose.position.y,
        Radians::fromRadians(yaw)
    );
}

// Convert ROS2 Vector3 to velocity
inline MetersPerSecond fromVector3VelocityX(const Vector3& v) {
    return MetersPerSecond::fromMetersPerSecond(v.x);
}

inline MetersPerSecond fromVector3VelocityY(const Vector3& v) {
    return MetersPerSecond::fromMetersPerSecond(v.y);
}

inline MetersPerSecond fromVector3VelocityZ(const Vector3& v) {
    return MetersPerSecond::fromMetersPerSecond(v.z);
}

// Convert ROS2 Vector3 to angular velocity
inline RadiansPerSecond fromVector3AngularX(const Vector3& v) {
    return RadiansPerSecond::fromRadiansPerSecond(v.x);
}

inline RadiansPerSecond fromVector3AngularY(const Vector3& v) {
    return RadiansPerSecond::fromRadiansPerSecond(v.y);
}

inline RadiansPerSecond fromVector3AngularZ(const Vector3& v) {
    return RadiansPerSecond::fromRadiansPerSecond(v.z);
}

// ============================================================================
// Convenience Functions for Common Patterns
// ============================================================================

// Create ROS2 Twist from 2D robot velocity
inline Twist toTwist2D(const MetersPerSecond& linear_x,
                       const MetersPerSecond& linear_y,
                       const RadiansPerSecond& angular_z) {
    return Twist{
        Vector3{linear_x.toMetersPerSecond(), linear_y.toMetersPerSecond(), 0.0},
        Vector3{0.0, 0.0, angular_z.toRadiansPerSecond()}
    };
}

// Create ROS2 Odometry from Pose2D and velocities
inline Odometry toOdometry2D(const robotics::Pose2D& pose,
                             const MetersPerSecond& linear_x,
                             const MetersPerSecond& linear_y,
                             const RadiansPerSecond& angular_z) {
    return Odometry{
        toPose2D(pose),
        toTwist2D(linear_x, linear_y, angular_z)
    };
}

// Extract linear velocity from ROS2 Twist (2D case, X component)
inline MetersPerSecond getLinearVelocity2D(const Twist& twist) {
    return MetersPerSecond::fromMetersPerSecond(twist.linear.x);
}

// Extract angular velocity from ROS2 Twist (2D case, Z component)
inline RadiansPerSecond getAngularVelocity2D(const Twist& twist) {
    return RadiansPerSecond::fromRadiansPerSecond(twist.angular.z);
}

// ============================================================================
// Direct ROS2 Message Conversions (if ROS2 is available)
// ============================================================================

#ifdef ROS2_AVAILABLE

// Direct conversions to actual ROS2 messages
inline geometry_msgs::msg::Point toRosPoint(const spatial::Vec3D& v) {
    geometry_msgs::msg::Point msg;
    msg.x = v.x;
    msg.y = v.y;
    msg.z = v.z;
    return msg;
}

inline geometry_msgs::msg::Vector3 toRosVector3(const spatial::Vec3D& v) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x;
    msg.y = v.y;
    msg.z = v.z;
    return msg;
}

inline geometry_msgs::msg::Quaternion toRosQuaternion(const spatial::Quaternion& q) {
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x;
    msg.y = q.y;
    msg.z = q.z;
    msg.w = q.w;
    return msg;
}

inline geometry_msgs::msg::Pose toRosPose(const spatial::Pose3D& pose) {
    geometry_msgs::msg::Pose msg;
    msg.position = toRosPoint(pose.position);
    msg.orientation = toRosQuaternion(pose.orientation);
    return msg;
}

inline geometry_msgs::msg::Pose toRosPose2D(const robotics::Pose2D& pose) {
    geometry_msgs::msg::Pose msg;
    msg.position.x = pose.position.x;
    msg.position.y = pose.position.y;
    msg.position.z = 0.0;

    auto q = spatial::Quaternion::fromEuler(0.0, 0.0, pose.theta.toRadians());
    msg.orientation = toRosQuaternion(q);
    return msg;
}

inline geometry_msgs::msg::Twist toRosTwist2D(const MetersPerSecond& linear_x,
                                               const MetersPerSecond& linear_y,
                                               const RadiansPerSecond& angular_z) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x.toMetersPerSecond();
    msg.linear.y = linear_y.toMetersPerSecond();
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = angular_z.toRadiansPerSecond();
    return msg;
}

// Direct conversions from actual ROS2 messages
inline spatial::Vec3D fromRosPoint(const geometry_msgs::msg::Point& msg) {
    return spatial::Vec3D(msg.x, msg.y, msg.z);
}

inline spatial::Quaternion fromRosQuaternion(const geometry_msgs::msg::Quaternion& msg) {
    return spatial::Quaternion(msg.w, msg.x, msg.y, msg.z);
}

inline spatial::Pose3D fromRosPose(const geometry_msgs::msg::Pose& msg) {
    return spatial::Pose3D(
        fromRosPoint(msg.position),
        fromRosQuaternion(msg.orientation)
    );
}

inline robotics::Pose2D fromRosPose2D(const geometry_msgs::msg::Pose& msg) {
    auto q = fromRosQuaternion(msg.orientation);
    double roll, pitch, yaw;
    q.toEuler(roll, pitch, yaw);

    return robotics::Pose2D(
        msg.position.x,
        msg.position.y,
        Radians::fromRadians(yaw)
    );
}

#endif // ROS2_AVAILABLE

// ============================================================================
// Usage Examples and Documentation
// ============================================================================

/*
USAGE WITHOUT ROS2:
-------------------
#include "units_ros2_interop.h"

// Create RobotLib data
Pose2D robot_pose(m(1.0), m(2.0), deg(45));

// Convert to ROS2-compatible structure
auto ros_pose = units::ros2::toPose2D(robot_pose);

// Now ros_pose.position.x, ros_pose.orientation.w, etc. are accessible


USAGE WITH ROS2:
----------------
#define ROS2_AVAILABLE
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "units_ros2_interop.h"

// Create RobotLib data
Pose2D robot_pose(m(1.0), m(2.0), deg(45));

// Convert directly to ROS2 message
geometry_msgs::msg::Pose ros_msg = units::ros2::toRosPose2D(robot_pose);

// Publish to ROS2 topic
publisher->publish(ros_msg);

// Receive from ROS2 and convert back
void callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    Pose2D robot_pose = units::ros2::fromRosPose2D(*msg);
    // Use robot_pose with type-safe units!
}


COMMON PATTERNS:
----------------

1. Publishing Odometry:
   auto odom = units::ros2::toOdometry2D(pose, vx, vy, omega);

2. Reading IMU Data:
   RadiansPerSecond gyro_z = units::ros2::fromVector3AngularZ(imu_msg.angular_velocity);

3. Sending Velocity Commands:
   auto twist = units::ros2::toTwist2D(vx, vy, omega);
*/

} // namespace ros2
} // namespace units

#endif // ROBOTLIB_UNITS_ROS2_INTEROP_H
