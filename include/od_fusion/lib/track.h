#pragma once

#include <vector>
#include <cstdint>
#include <limits>
#include <deque>
#include <iostream>
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

  void UpdateWithoutSensorObject(SensorType sensor_type, double meas_time);

  void GetEstimate(float* x, float* y, float* vx, float* vy) const;

  // Static ID generation
  static int32_t GenerateNewTrackId();
  static void SetMaxInvisiblePeriod(SensorType sensor_type, float period);
  static void ResetTrackIdCounter();

  // ========== Track ID ==========
  int32_t GetId() const { return track_id_; }
  void SetId(int32_t id) { track_id_ = id; }

  // ========== State ==========
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

  // ========== Visibility per sensor type ==========
  bool IsVisible(SensorType sensor_type) const;
  bool IsSvsVisible() const { return IsVisible(SensorType::kSvs); }
  bool IsBevVisible() const { return IsVisible(SensorType::kBev); }
  bool IsRadarVisible() const { return IsVisible(SensorType::kRadar); }

  // ========== Tracking period ==========
  double GetTrackingPeriod() const { return tracking_period_; }

  // ========== Invisibility period per sensor ==========
  double GetInvisibilityPeriod(SensorType sensor_type) const;
  void SetInvisibilityPeriod(SensorType sensor_type, double period);

  // ========== Counter management ==========
  void UpdateCounter(int32_t* cnt, int32_t* lst, bool valid);
  void UpdateSvsCounter(bool valid);
  void UpdateBevCounter(bool valid);
  void UpdateRadarCounter(bool valid);
  void UpdateFusedCounter();

  void SetFusedType(ObjectType type) { status_.fused_type = type; }
  ObjectType GetFusedType() const { return status_.fused_type; }

  void SetLastTrackingTime(double time) { status_.last_tracking_time = time; }
  double GetLastTrackingTime() const { return status_.last_tracking_time; }

  // ========== Sensor History Management (new design) ==========
  // Add observation to sensor-specific history
  void AddToSensorHistory(SensorType sensor_type, const FusedObject& obs);

  // Get sensor-specific history
  const std::deque<FusedObject>& GetSensorHistory(SensorType sensor_type) const;
  std::deque<FusedObject>& GetSensorHistory(SensorType sensor_type);

  // Get previous observation from sensor history
  FusedObject GetPreviousSensorObject(SensorType sensor_type, int32_t offset) const;

  // Prune old observations from sensor history (remove observations older than threshold)
  void PruneSensorHistory(SensorType sensor_type, double current_time);

  // ========== Fused Track Object (replaces estimated_) ==========
  FusedObject& GetFusedObject() { return fused_object_; }
  const FusedObject& GetFusedObject() const { return fused_object_; }
  void SetFusedObject(const FusedObject& obj) { fused_object_ = obj; }

  // ========== Output ==========
  FusedObject& GetOutput() { return output_; }
  const FusedObject& GetOutput() const { return output_; }
  void SetOutput(const FusedObject& out) { output_ = out; }

  // ========== Flags ==========
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
  TrackStatus status_;
  CvKalmanFilter kalman_filter_;

  // Current fused track object (updated at each measurement)
  FusedObject fused_object_;

  // Output object (for publishing)
  FusedObject output_;

  uint8_t meas_flag_;

  // ========== State variables ==========
  bool is_alive_;
  bool is_predicted_;
  int32_t prediction_time_;
  double tracking_period_;

  // Per-sensor invisibility tracking
  double invisibility_period_svs_;
  double invisibility_period_bev_;
  double invisibility_period_radar_;
  double last_obs_time_svs_;
  double last_obs_time_bev_;
  double last_obs_time_radar_;

  // ========== Sensor History (new design - separate per sensor) ==========
  static constexpr size_t kMaxSensorHistorySize = 30;
  std::deque<FusedObject> sensor_history_svs_;
  std::deque<FusedObject> sensor_history_bev_;
  std::deque<FusedObject> sensor_history_radar_;

  // Static configurable parameters
  static int32_t s_track_idx_;
  static float s_max_invisible_period_svs_;
  static float s_max_invisible_period_bev_;
  static float s_max_invisible_period_radar_;
};

}  // namespace fusion
}  // namespace perception
