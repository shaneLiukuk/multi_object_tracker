#pragma once

#include <vector>
#include <cstdint>
#include <limits>
#include <Eigen/Dense>
#include "od_fusion/base/obstacle_constant.h"
#include "od_fusion/lib/cv_kalman_filter.h"

namespace perception {
namespace fusion {

class Track {
 public:
  Track();

  void Reset();

  void Initialize(const FusedObject& obs);

  void Predict(float dt);

  void Update(const FusedObject& obs);

  void UpdateWithoutSensorObject(SensorType sensor_type, uint64_t meas_time);

  void GetEstimate(float* x, float* y, float* vx, float* vy) const;

  // Static ID generation
  static int32_t GenerateNewTrackId();
  static void SetMaxInvisiblePeriod(SensorType sensor_type, float period);
  static void ResetTrackIdCounter();

  // Track ID
  int32_t GetId() const { return track_id_; }
  void SetId(int32_t id) { track_id_ = id; }

  // State
  TrackState GetState() const { return status_.state; }
  bool IsUsed() const { return status_.used > 0; }
  void SetUsed(bool used) { status_.used = used ? 1 : 0; }
  bool IsAlive() const { return is_alive_; }
  void SetAlive(bool alive) { is_alive_ = alive; }
  bool IsPredicted() const { return is_predicted_; }
  void SetPredicted(bool predicted) { is_predicted_ = predicted; }
  int32_t GetPredictionTime() const { return prediction_time_; }
  void IncPredictionTime() { prediction_time_++; }
  void ResetPredictionTime() { prediction_time_ = 0; is_predicted_ = false; }

  // Visibility per sensor type
  bool IsVisible(SensorType sensor_type) const;
  bool IsSvsVisible() const { return IsVisible(SensorType::kSvs); }
  bool IsBevVisible() const { return IsVisible(SensorType::kBev); }
  bool IsRadarVisible() const { return IsVisible(SensorType::kRadar); }

  // Tracking period
  double GetTrackingPeriod() const { return tracking_period_; }
  void AddTrackingPeriod(double period) { tracking_period_ += period; }

  // Invisibility period per sensor
  double GetInvisibilityPeriod(SensorType sensor_type) const;
  void SetInvisibilityPeriod(SensorType sensor_type, double period);

  // Counter management
  void UpdateCounter(int32_t* cnt, int32_t* lst, bool valid);
  void UpdateSvsCounter(bool valid);
  void UpdateBevCounter(bool valid);
  void UpdateRadarCounter(bool valid);
  void UpdateFusedCounter();

  void SetFusedType(ObjectType type) { status_.fused_type = type; }
  ObjectType GetFusedType() const { return status_.fused_type; }

  void SetLastTrackingTime(uint64_t time) { status_.last_tracking_time = time; }
  uint64_t GetLastTrackingTime() const { return status_.last_tracking_time; }

  // For measurement update without associated sensor
  void UpdateSensorInvisibilityPeriod(SensorType sensor_type, uint64_t meas_time);

  // History buffer access
  int32_t GetPreviousIndex(int32_t offset) const;
  FusedObject& GetTrackObject();
  const FusedObject& GetTrackObject() const;
  FusedObject& GetHistoryObject(int32_t depth_idx);
  const FusedObject& GetHistoryObject(int32_t depth_idx) const;
  void SetHistoryObject(int32_t depth_idx, const FusedObject& obj);
  uint8_t& GetHistoryFlag(int32_t depth_idx);
  uint8_t GetHistoryFlag(int32_t depth_idx) const;
  void SetHistoryFlag(int32_t depth_idx, uint8_t flag);

  void SetCursorNext();
  int32_t GetNextCursor() const;

  // Estimated and output
  FusedObject& GetEstimated() { return estimated_; }
  const FusedObject& GetEstimated() const { return estimated_; }
  void SetEstimated(const FusedObject& est) { estimated_ = est; }

  FusedObject& GetOutput() { return output_; }
  const FusedObject& GetOutput() const { return output_; }
  void SetOutput(const FusedObject& out) { output_ = out; }

  void SetSvsMatchId(uint8_t id) { estimated_.svs_match_id = id; }
  void SetBevMatchId(uint8_t id) { estimated_.bev_match_id = id; }
  void SetRadarMatchId(uint8_t id) { estimated_.radar_match_id = id; }

  void SetMeasFlag(uint8_t flag) { meas_flag_ = flag; }
  uint8_t GetMeasFlag() const { return meas_flag_; }

  void SetFlag(uint8_t flag) { status_.flag = flag; }
  uint8_t GetFlag() const { return status_.flag; }

  void CheckStability();

  const TrackStatus& GetStatus() const { return status_; }
  TrackStatus& GetStatus() { return status_; }

  CvKalmanFilter& GetKalmanFilter() { return kalman_filter_; }
  const CvKalmanFilter& GetKalmanFilter() const { return kalman_filter_; }

 private:
  int32_t track_id_;
  uint8_t cursor_;
  TrackStatus status_;
  CvKalmanFilter kalman_filter_;
  std::vector<FusedObject> matrix_;
  std::vector<uint8_t> lst_flg_;
  FusedObject estimated_;
  FusedObject output_;
  uint8_t meas_flag_;

  // New state variables
  bool is_alive_;
  bool is_predicted_;
  int32_t prediction_time_;
  double tracking_period_;

  // Per-sensor invisibility tracking
  double invisibility_period_svs_;
  double invisibility_period_bev_;
  double invisibility_period_radar_;
  uint64_t last_obs_time_svs_;
  uint64_t last_obs_time_bev_;
  uint64_t last_obs_time_radar_;

  // Static configurable parameters
  static int32_t s_track_idx_;
  static float s_max_invisible_period_svs_;
  static float s_max_invisible_period_bev_;
  static float s_max_invisible_period_radar_;
};

}  // namespace fusion
}  // namespace perception
