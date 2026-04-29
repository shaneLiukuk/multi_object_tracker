#pragma once

#include <Eigen/Dense>
#include <vector>
#include "od_fusion/base/obstacle_constant.h"
#include "od_fusion/utils/f_define.h"

namespace perception {
namespace fusion {

class CvKalmanFilter {
 public:
  CvKalmanFilter();

  void Init(float x, float y, float vx, float vy, ObjectType type);

  void Predict(float time_diff);

  void Update(float x, float y, float vx, float vy, ObjectType type);

  void GetEstimate(float* x, float* y, float* vx, float* vy) const;

 private:
  Eigen::Matrix4f TransitionMatrix(float dt) const;

  Eigen::Matrix4f ProcessNoiseCovariance(float dt) const;

  void Decorrelation();

  void CorrectionBreakdown();

  Eigen::Vector4f x_est_;
  Eigen::Matrix4f p_est_;
};

}  // namespace fusion
}  // namespace perception
