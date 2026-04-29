#pragma once

#include <cmath>
#include "od_fusion/base/obstacle_constant.h"
#include "od_fusion/utils/f_define.h"

namespace perception {
namespace fusion {

inline void LocalToGlobal(const GlobalPose& pose, float local_x, float local_y,
                          float* global_x, float* global_y) {
  float cos_heading = std::cos(pose.heading);
  float sin_heading = std::sin(pose.heading);
  *global_x = pose.x + cos_heading * local_x - sin_heading * local_y;
  *global_y = pose.y + sin_heading * local_x + cos_heading * local_y;
}

inline void GlobalToLocal(const GlobalPose& pose, float global_x, float global_y,
                          float* local_x, float* local_y) {
  float dx = global_x - pose.x;
  float dy = global_y - pose.y;
  float cos_heading = std::cos(pose.heading);
  float sin_heading = std::sin(pose.heading);
  *local_x = cos_heading * dx + sin_heading * dy;
  *local_y = -sin_heading * dx + cos_heading * dy;
}

inline void CartesianToIso8855(float x, float y, float* iso_x, float* iso_y) {
  *iso_x = y;
  *iso_y = -x;
}

inline void Iso8855ToCartesian(float iso_x, float iso_y, float* x, float* y) {
  *x = -iso_y;
  *y = iso_x;
}

inline float NormalizeAngle2Pi(float angle) {
  while (angle < 0.0f) {
    angle += 2.0f * kPi;
  }
  while (angle >= 2.0f * kPi) {
    angle -= 2.0f * kPi;
  }
  return angle;
}

inline float YawCs1ToIso8855(float yaw_cs1) {
  float b = -yaw_cs1;
  return NormalizeAngle2Pi(b);
}

inline float YawIso8855ToCs3(float yaw_iso8855) {
  return -yaw_iso8855;
}



inline void LocalToGlobalVel(const GlobalPose& pose, float local_vx, float local_vy,
                             float* global_vx, float* global_vy) {
  float cos_heading = std::cos(pose.heading);
  float sin_heading = std::sin(pose.heading);
  *global_vx = cos_heading * local_vx - sin_heading * local_vy;
  *global_vy = sin_heading * local_vx + cos_heading * local_vy;
}

inline void GlobalToLocalVel(const GlobalPose& pose, float global_vx, float global_vy,
                             float* local_vx, float* local_vy) {
  float cos_heading = std::cos(pose.heading);
  float sin_heading = std::sin(pose.heading);
  *local_vx = cos_heading * global_vx + sin_heading * global_vy;
  *local_vy = -sin_heading * global_vx + cos_heading * global_vy;
}

}  // namespace fusion
}  // namespace perception
