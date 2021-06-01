#pragma once

namespace dr
{
/// Create a translation from a vector.
inline Eigen::Translation3d translate(Eigen::Vector3d translation)
{
  return Eigen::Translation3d{ translation };
}

}  // namespace dr
