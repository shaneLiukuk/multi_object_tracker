#include "od_fusion/lib/cv_kalman_filter.h"

namespace perception {
namespace fusion {


bool IsValidType(ObjectType type) {
  int type_val = static_cast<int>(type);
  bool valid = (type_val > 0 && type_val < 8) ||
               (type_val > 14 && type_val < 18) ||
               (type_val > 19 && type_val < 22);
  return valid;
}


CvKalmanFilter::CvKalmanFilter() : x_est_(4, 1), p_est_(4, 4) {
  x_est_.setZero();
  p_est_.setZero();
}

Eigen::Matrix4f CvKalmanFilter::TransitionMatrix(float dt) const {
  Eigen::Matrix4f A;
  A << 1.0f, 0.0f, dt,  0.0f,
       0.0f, 1.0f, 0.0f, dt,
       0.0f, 0.0f, 1.0f, 0.0f,
       0.0f, 0.0f, 0.0f, 1.0f;
  return A;
}

Eigen::Matrix4f CvKalmanFilter::ProcessNoiseCovariance(float dt) const {
  Eigen::Matrix4f Q;
  Q.setZero();
  Q.diagonal() << kProcessNoisePos, kProcessNoisePos, kProcessNoiseVel, kProcessNoiseVel;
  return Q * dt;
}

void CvKalmanFilter::Init(float x, float y, float vx, float vy, ObjectType type) {
  Eigen::Matrix4f Q = ProcessNoiseCovariance(1.0f);
  x_est_ << x, y, vx, vy;
  p_est_ = Q;
}

void CvKalmanFilter::Predict(float time_diff) {
  float dt = time_diff;
  if (dt <= 0.0f) {
    dt = kTimeDiffMin;
  }
  if (dt > kTimeDiffMax) {
    dt = kTimeDiffMax;
  }

  Eigen::Matrix4f A = TransitionMatrix(dt);
  Eigen::Matrix4f Q = ProcessNoiseCovariance(dt);

  x_est_ = A * x_est_;
  p_est_ = A * p_est_ * A.transpose() + Q;

  Decorrelation();
}

void CvKalmanFilter::Update(float x, float y, float vx, float vy, ObjectType type) {
  if (!IsValidType(type)) {
    Eigen::Matrix4f Q = ProcessNoiseCovariance(1.0f);
    x_est_ << x, y, vx, vy;
    p_est_ = Q;
    return;
  }

  Eigen::Matrix4f H = Eigen::Matrix4f::Identity();

  Eigen::Vector4f z;
  z << x, y, vx, vy;

  Eigen::Matrix4f R;
  R.setZero();
  R.diagonal() << kMeasurementNoiseRx, kMeasurementNoiseRy,
                   kMeasurementNoiseRvx, kMeasurementNoiseRvy;

  Eigen::Matrix4f S = H * p_est_ * H.transpose() + R;
  Eigen::Matrix4f K = p_est_ * H.transpose() * S.inverse();

  Eigen::Vector4f x_prd = x_est_;
  x_est_ = x_prd + K * (z - H * x_prd);
  p_est_ = (Eigen::Matrix4f::Identity() - K * H) * p_est_;

  CorrectionBreakdown();
}

void CvKalmanFilter::GetEstimate(float* x, float* y, float* vx, float* vy) const {
  *x = x_est_(0);
  *y = x_est_(1);
  *vx = x_est_(2);
  *vy = x_est_(3);
}

void CvKalmanFilter::Decorrelation() {
  p_est_.block<1, 2>(0, 2).setZero();
  p_est_.block<2, 1>(2, 0).setZero();
}

void CvKalmanFilter::CorrectionBreakdown() {
  static constexpr float kGainMaskPosX = 1.0f;
  static constexpr float kGainMaskPosY = 1.0f;
  static constexpr float kGainMaskVelX = 0.8f;
  static constexpr float kGainMaskVelY = 0.8f;
  static constexpr float kGainThreshold = 0.5f;
  static constexpr float kValueMaskVelX = 0.0f;
  static constexpr float kValueMaskVelY = 0.0f;
  static constexpr float kValueThreshold = 0.05f;

  Eigen::Vector4f gain_mask;
  gain_mask << kGainMaskPosX, kGainMaskPosY, kGainMaskVelX, kGainMaskVelY;

  Eigen::Vector4f value_mask;
  value_mask << kValueMaskVelX, kValueMaskVelY, 1.0f, 1.0f;

  Eigen::Vector4f x_prev = x_est_;

  Eigen::Vector4f state_gain = x_est_ - x_prev;
  Eigen::Vector4f break_diff = state_gain.cwiseProduct(gain_mask);

  x_est_ = x_est_ - break_diff;

  float norm_diff = break_diff.norm();
  if (norm_diff > kGainThreshold) {
    break_diff = break_diff / norm_diff * kGainThreshold;
  }
  x_est_ = x_est_ + break_diff;

  float vel_norm = (x_est_.cwiseProduct(value_mask)).norm();
  if (vel_norm < kValueThreshold) {
    x_est_ = x_est_.cwiseProduct(Eigen::Vector4f(1.0f, 1.0f, 0.0f, 0.0f));
  }
}

}  // namespace fusion
}  // namespace perception
