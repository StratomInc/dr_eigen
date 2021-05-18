#pragma once
#include "eigen.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace dr {

/// Convert a ROS Point to an Eigen vector.
inline Eigen::Vector3d toEigen(geometry_msgs::msg::Point const& vector)
{
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

/// Convert a ROS Quaternion to an Eigen quaternion.
inline Eigen::Quaterniond toEigen(geometry_msgs::msg::Quaternion& quaternion)
{
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

/// Convert a ROS Pose to an Eigen isometry.
inline Eigen::Isometry3d toEigen(geometry_msgs::msg::Pose& pose)
{
  return translate(toEigen(pose.position)) * toEigen(pose.orientation);
}

/// Convert an Eigen vector to a ROS Point.
inline geometry_msgs::msg::Point toRosPoint(Eigen::Vector3d const& vector)
{
  geometry_msgs::msg::Point result;
  result.x = vector.x();
  result.y = vector.y();
  result.z = vector.z();
  return result;
}

/// Convert an Eigen quaternion to a ROS Quaternion.
inline geometry_msgs::msg::Quaternion toRosQuaternion(Eigen::Quaterniond const& quaternion)
{
  geometry_msgs::msg::Quaternion result;
  result.w = quaternion.w();
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  return result;
}

/// Convert an Eigen angle axis to a ROS Quaternion.
inline geometry_msgs::msg::Quaternion toRosQuaternion(Eigen::AngleAxisd const& angle_axis)
{
  return toRosQuaternion(Eigen::Quaterniond(angle_axis));
}

/// Convert an Eigen isometry to a ROS Pose.
inline geometry_msgs::msg::Pose toRosPose(Eigen::Isometry3d const& pose)
{
  geometry_msgs::msg::Pose result;
  result.position = toRosPoint(pose.translation());
  result.orientation = toRosQuaternion(Eigen::Quaterniond(pose.rotation()));
  return result;
}


}
