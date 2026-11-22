#ifndef GECKO_RVIZ_PANEL__DRIVE_MAPPING_HPP_
#define GECKO_RVIZ_PANEL__DRIVE_MAPPING_HPP_

#include <cmath>

namespace gecko_rviz_panel
{

constexpr int DRIVE_STOP = 0;
constexpr int DRIVE_FORWARD = 1;
constexpr int DRIVE_BACK = 2;
constexpr int DRIVE_RIGHT = 3;
constexpr int DRIVE_LEFT = 4;

constexpr double DEAD_ZONE = 0.2;

// x grows to the right and y grows upwards, both in [-1, 1].
inline int driveFromVector(double x, double y)
{
  if (std::hypot(x, y) < DEAD_ZONE) {
    return DRIVE_STOP;
  }
  if (std::abs(x) > std::abs(y)) {
    return x > 0.0 ? DRIVE_RIGHT : DRIVE_LEFT;
  }
  return y > 0.0 ? DRIVE_FORWARD : DRIVE_BACK;
}

}  // namespace gecko_rviz_panel

#endif  // GECKO_RVIZ_PANEL__DRIVE_MAPPING_HPP_
