

#ifndef TRANSFORM_HELPERS_HPP
#define TRANSFORM_HELPERS_HPP

#include <urdf/model.h>           // To define the URDF model
// #include <urdf_parser/urdf_parser.h>    // To parse the URDF file
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <cmath>

using namespace Eigen;

//
// Cross Product
//
inline Vector3d cross(const Vector3d& a, const Vector3d& b) {
    return a.cross(b);
}

inline Matrix3d crossmat(const Vector3d& e) {
    Matrix3d mat;
    mat <<    0, -e(2),  e(1),
           e(2),     0, -e(0),
          -e(1),  e(0),     0;
    return mat;
}

//
// 3x1 Position Vector
//
inline Vector3d pzero() {
    return Vector3d::Zero();
}

inline Vector3d pxyz(double x, double y, double z) {
    return Vector3d(x, y, z);
}

inline Vector3d pe(const Vector3d& e, double d) {
    return e * d;
}

//
// 3x1 Axis (Unit Vector)
//
inline Vector3d ex() {
    return Vector3d(1.0, 0.0, 0.0);
}

inline Vector3d ey() {
    return Vector3d(0.0, 1.0, 0.0);
}

inline Vector3d ez() {
    return Vector3d(0.0, 0.0, 1.0);
}

inline Vector3d exyz(double x, double y, double z) {
    return Vector3d(x, y, z).normalized();
}

//
// 3x3 Rotation Matrix
//
inline Matrix3d Reye() {
    return Matrix3d::Identity();
}

inline Matrix3d Rotx(double alpha) {
    Matrix3d R;
    R << 1, 0, 0,
         0, cos(alpha), -sin(alpha),
         0, sin(alpha), cos(alpha);
    return R;
}

inline Matrix3d Roty(double alpha) {
    Matrix3d R;
    R << cos(alpha), 0, sin(alpha),
         0, 1, 0,
         -sin(alpha), 0, cos(alpha);
    return R;
}

inline Matrix3d Rotz(double alpha) {
    Matrix3d R;
    R << cos(alpha), -sin(alpha), 0,
         sin(alpha), cos(alpha), 0,
         0, 0, 1;
    return R;
}

inline Matrix3d Rote(const Vector3d& e, double alpha) {
    Matrix3d ex = crossmat(e);
    return Matrix3d::Identity() + sin(alpha) * ex + (1.0 - cos(alpha)) * ex * ex;
}

//
// 4x4 Transform Matrix
//
inline Matrix4d T_from_Rp(const Matrix3d& R, const Vector3d& p) {
    Matrix4d T = Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T.block<3,1>(0,3) = p;
    return T;
}

inline Vector3d p_from_T(const Matrix4d& T) {
    return T.block<3,1>(0,3);
}

inline Matrix3d R_from_T(const Matrix4d& T) {
    return T.block<3,3>(0,0);
}

//
// Quaternion Conversion
//
inline Matrix3d R_from_quat(const Quaterniond& quat) {
    return quat.toRotationMatrix();
}

inline Quaterniond quat_from_R(const Matrix3d& R) {
    return Quaterniond(R);
}

//
// Axis-Angle Conversion
//
inline std::pair<Vector3d, double> axisangle_from_R(const Matrix3d& R) {
    AngleAxisd angleAxis(R);
    return std::make_pair(angleAxis.axis(), angleAxis.angle());
}

//
// Roll/Pitch/Yaw
//
inline Matrix3d R_from_RPY(double roll, double pitch, double yaw) {
    return Rotz(yaw) * Roty(pitch) * Rotx(roll);
}

//
// URDF Elements
//
inline Vector3d p_from_URDF_xyz(const std::vector<double>& xyz) {
    return Vector3d(xyz[0], xyz[1], xyz[2]);
}

inline Matrix3d R_from_URDF_rpy(const std::vector<double>& rpy) {
    return R_from_RPY(rpy[0], rpy[1], rpy[2]);
}

inline Matrix4d T_from_URDF_origin(const urdf::Pose& origin) {
    return T_from_Rp(R_from_URDF_rpy({origin.rotation.x, origin.rotation.y, origin.rotation.z}),
                     p_from_URDF_xyz({origin.position.x, origin.position.y, origin.position.z}));
}

inline Vector3d e_from_URDF_axis(const urdf::Vector3& axis) {
    return Vector3d(axis.x, axis.y, axis.z);
}

//
// From ROS Messages
//
inline Vector3d p_from_Point(const geometry_msgs::msg::Point& point) {
    return pxyz(point.x, point.y, point.z);
}

inline Vector3d p_from_Vector3(const geometry_msgs::msg::Vector3& vector3) {
    return pxyz(vector3.x, vector3.y, vector3.z);
}

inline Quaterniond quat_from_Quaternion(const geometry_msgs::msg::Quaternion& quaternion) {
    return Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

inline Matrix3d R_from_Quaternion(const geometry_msgs::msg::Quaternion& quaternion) {
    return R_from_quat(quat_from_Quaternion(quaternion));
}

inline Matrix4d T_from_Pose(const geometry_msgs::msg::Pose& pose) {
    return T_from_Rp(R_from_Quaternion(pose.orientation), p_from_Point(pose.position));
}

inline Matrix4d T_from_Transform(const geometry_msgs::msg::Transform& transform) {
    return T_from_Rp(R_from_Quaternion(transform.rotation), p_from_Vector3(transform.translation));
}

//
// To ROS Messages
//
inline geometry_msgs::msg::Point Point_from_p(const Vector3d& p) {
    geometry_msgs::msg::Point point;
    point.x = p(0);
    point.y = p(1);
    point.z = p(2);
    return point;
}

inline geometry_msgs::msg::Vector3 Vector3_from_p(const Vector3d& p) {
    geometry_msgs::msg::Vector3 vector;
    vector.x = p(0);
    vector.y = p(1);
    vector.z = p(2);
    return vector;
}

inline geometry_msgs::msg::Quaternion Quaternion_from_quat(const Quaterniond& quat) {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.x = quat.x();
    quaternion.y = quat.y();
    quaternion.z = quat.z();
    quaternion.w = quat.w();
    return quaternion;
}

inline geometry_msgs::msg::Quaternion Quaternion_from_R(const Matrix3d& R) {
    return Quaternion_from_quat(quat_from_R(R));
}

inline geometry_msgs::msg::Pose Pose_from_T(const Matrix4d& T) {
    geometry_msgs::msg::Pose pose;
    pose.position = Point_from_p(p_from_T(T));
    pose.orientation = Quaternion_from_R(R_from_T(T));
    return pose;
}

inline geometry_msgs::msg::Transform Transform_from_T(const Matrix4d& T) {
    geometry_msgs::msg::Transform transform;
    transform.translation = Vector3_from_p(p_from_T(T));
    transform.rotation = Quaternion_from_R(R_from_T(T));
    return transform;
}

#endif // TRANSFORM_HELPERS_HPP
