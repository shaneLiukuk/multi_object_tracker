#include "od_fusion/lib/track.h"

#include <algorithm>
#include <cmath>

namespace perception {
namespace fusion {

// Static member initialization
int32_t Track::s_track_idx_ = 1;
float Track::s_max_invisible_period_svs_ = 0.5f;   // 500ms
float Track::s_max_invisible_period_bev_ = 0.5f;   // 500ms
float Track::s_max_invisible_period_radar_ = 0.75f; // 750ms

namespace {
constexpr int32_t kThresStable = 3;
constexpr int32_t kThresUnstable = 2;
constexpr int32_t kThresIdAssign = 20;
constexpr float kVelocityThreshold = 1e-4f;
}  // namespace

Track::Track()
    : track_id_(0),
      meas_flag_(0),
      is_alive_(true),
      is_predicted_(false),
      prediction_time_(0),
      tracking_period_(0.0),
      invisibility_period_svs_(0.0),
      invisibility_period_bev_(0.0),
      invisibility_period_radar_(0.0),
      last_obs_time_svs_(0),
      last_obs_time_bev_(0),
      last_obs_time_radar_(0) {
}

int32_t Track::GenerateNewTrackId() {
  int32_t new_id = s_track_idx_;
  if (s_track_idx_ >= std::numeric_limits<int32_t>::max()) {
    s_track_idx_ = 1;
  } else {
    s_track_idx_++;
  }
  return new_id;
}

void Track::SetMaxInvisiblePeriod(SensorType sensor_type, float period) {
  switch (sensor_type) {
    case SensorType::kSvs:
      s_max_invisible_period_svs_ = period;
      break;
    case SensorType::kBev:
      s_max_invisible_period_bev_ = period;
      break;
    case SensorType::kRadar:
      s_max_invisible_period_radar_ = period;
      break;
    default:
      break;
  }
}

void Track::ResetTrackIdCounter() {
  s_track_idx_ = 1;
}

void Track::Reset() {
  status_.cnt = 0;
  status_.lst = 0;
  status_.cnt_svs = 0;
  status_.cnt_bev = 0;
  status_.cnt_radar = 0;
  status_.cnt_fused = 0;
  status_.lst_svs = 0;
  status_.lst_bev = 0;
  status_.lst_radar = 0;
  status_.used = 0;
  status_.flag = 0;
  status_.fused_type = ObjectType::kUnknown;
  status_.state = TrackState::kUntracking;

  fused_object_ = FusedObject();
  output_ = FusedObject();
  meas_flag_ = 0;

  // Clear sensor histories
  sensor_history_svs_.clear();
  sensor_history_bev_.clear();
  sensor_history_radar_.clear();

  // Reset state variables
  is_alive_ = true;
  is_predicted_ = false;
  prediction_time_ = 0;
  tracking_period_ = 0.0;
  invisibility_period_svs_ = 0.0;
  invisibility_period_bev_ = 0.0;
  invisibility_period_radar_ = 0.0;
  last_obs_time_svs_ = 0;
  last_obs_time_bev_ = 0;
  last_obs_time_radar_ = 0;
}

void Track::Initialize(const FusedObject& obs) {
  fused_object_ = obs;

  status_.used = 1;
  status_.flag = 1;
  status_.cnt = 1;
  status_.lst = 0;

  // Initialize sensor-specific counters based on which sensor provided measurement
  if (obs.svs_match_id > 0) {
    status_.cnt_svs = 1;
    status_.lst_svs = 0;
    last_obs_time_svs_ = obs.object.timestamp;
    AddToSensorHistory(SensorType::kSvs, obs);
  }
  if (obs.bev_match_id > 0) {
    status_.cnt_bev = 1;
    status_.lst_bev = 0;
    last_obs_time_bev_ = obs.object.timestamp;
    AddToSensorHistory(SensorType::kBev, obs);
  }
  if (obs.radar_match_id > 0) {
    status_.cnt_radar = 1;
    status_.lst_radar = 0;
    last_obs_time_radar_ = obs.object.timestamp;
    AddToSensorHistory(SensorType::kRadar, obs);
  }

  // Generate new track ID
  track_id_ = GenerateNewTrackId();

  // Initialize Kalman filter
  kalman_filter_.Init(
      obs.object.x, obs.object.y, obs.object.vx, obs.object.vy,
      obs.object.type);

  status_.last_tracking_time = obs.object.timestamp;

  // Reset state
  is_alive_ = true;
  is_predicted_ = false;
  prediction_time_ = 0;
  tracking_period_ = 0.0;
}

void Track::Predict(float dt) {
  kalman_filter_.Predict(dt);
}

void Track::Update(const FusedObject& obs) {
  kalman_filter_.Update(
      obs.object.x, obs.object.y,
      obs.object.vx, obs.object.vy,
      obs.object.type);

  // Update tracking period
  if (obs.object.timestamp > status_.last_tracking_time) {
    double time_diff = static_cast<double>(obs.object.timestamp - status_.last_tracking_time) / 1000.0;
    tracking_period_ += time_diff;
  }

  // Add observation to appropriate sensor history
  if (obs.svs_match_id > 0) {
    AddToSensorHistory(SensorType::kSvs, obs);
  }
  if (obs.bev_match_id > 0) {
    AddToSensorHistory(SensorType::kBev, obs);
  }
  if (obs.radar_match_id > 0) {
    AddToSensorHistory(SensorType::kRadar, obs);
  }
}

void Track::UpdateWithoutSensorObject(SensorType sensor_type, uint64_t meas_time) {
  // Update invisibility period for the sensor that didn't provide measurement
  double time_diff = 0.0;
  if (meas_time > status_.last_tracking_time) {
    time_diff = static_cast<double>(meas_time - status_.last_tracking_time) / 1000.0;
  }

  switch (sensor_type) {
    case SensorType::kSvs:
      invisibility_period_svs_ = time_diff;
      if (time_diff > s_max_invisible_period_svs_) {
        status_.lst_svs++;
        status_.cnt_svs = std::max(0, status_.cnt_svs - 1);
      }
      break;
    case SensorType::kBev:
      invisibility_period_bev_ = time_diff;
      if (time_diff > s_max_invisible_period_bev_) {
        status_.lst_bev++;
        status_.cnt_bev = std::max(0, status_.cnt_bev - 1);
      }
      break;
    case SensorType::kRadar:
      invisibility_period_radar_ = time_diff;
      if (time_diff > s_max_invisible_period_radar_) {
        status_.lst_radar++;
        status_.cnt_radar = std::max(0, status_.cnt_radar - 1);
      }
      break;
    default:
      break;
  }

  // Update prediction state
  SetPredicted(true);
  IncPredictionTime();

  // Prune old observations from this sensor's history
  PruneSensorHistory(sensor_type, meas_time);
}

double Track::GetInvisibilityPeriod(SensorType sensor_type) const {
  switch (sensor_type) {
    case SensorType::kSvs:
      return invisibility_period_svs_;
    case SensorType::kBev:
      return invisibility_period_bev_;
    case SensorType::kRadar:
      return invisibility_period_radar_;
    default:
      return 0.0;
  }
}

void Track::SetInvisibilityPeriod(SensorType sensor_type, double period) {
  switch (sensor_type) {
    case SensorType::kSvs:
      invisibility_period_svs_ = period;
      break;
    case SensorType::kBev:
      invisibility_period_bev_ = period;
      break;
    case SensorType::kRadar:
      invisibility_period_radar_ = period;
      break;
    default:
      break;
  }
}

bool Track::IsVisible(SensorType sensor_type) const {
  switch (sensor_type) {
    case SensorType::kSvs:
      return status_.cnt_svs > 0 && invisibility_period_svs_ < s_max_invisible_period_svs_;
    case SensorType::kBev:
      return status_.cnt_bev > 0 && invisibility_period_bev_ < s_max_invisible_period_bev_;
    case SensorType::kRadar:
      return status_.cnt_radar > 0 && invisibility_period_radar_ < s_max_invisible_period_radar_;
    default:
      return false;
  }
}

void Track::GetEstimate(float* x, float* y, float* vx, float* vy) const {
  kalman_filter_.GetEstimate(x, y, vx, vy);
}

void Track::UpdateCounter(int32_t* cnt, int32_t* lst, bool valid) {
  if (cnt == nullptr || lst == nullptr) {
    return;
  }
  if (valid) {
    *cnt = *cnt + 1;
    *lst = std::max(0, *lst - 1);
  } else {
    *cnt = std::max(0, *cnt - 1);
    *lst = *lst + 1;
  }
}

void Track::UpdateSvsCounter(bool valid) {
  UpdateCounter(&status_.cnt_svs, &status_.lst_svs, valid);
}

void Track::UpdateBevCounter(bool valid) {
  UpdateCounter(&status_.cnt_bev, &status_.lst_bev, valid);
}

void Track::UpdateRadarCounter(bool valid) {
  UpdateCounter(&status_.cnt_radar, &status_.lst_radar, valid);
}

void Track::UpdateFusedCounter() {
  if (status_.cnt_fused >= kThresIdAssign && fused_object_.svs_match_id > 0) {
    status_.fused_type = fused_object_.object.type;
  }
}

// ========== Sensor History Implementation ==========

void Track::AddToSensorHistory(SensorType sensor_type, const FusedObject& obs) {
  std::deque<FusedObject>* history = nullptr;

  switch (sensor_type) {
    case SensorType::kSvs:
      history = &sensor_history_svs_;
      break;
    case SensorType::kBev:
      history = &sensor_history_bev_;
      break;
    case SensorType::kRadar:
      history = &sensor_history_radar_;
      break;
    default:
      return;
  }

  history->push_back(obs);

  // Maintain max size of 30
  while (history->size() > kMaxSensorHistorySize) {
    history->pop_front();
  }
}

const std::deque<FusedObject>& Track::GetSensorHistory(SensorType sensor_type) const {
  switch (sensor_type) {
    case SensorType::kSvs:
      return sensor_history_svs_;
    case SensorType::kBev:
      return sensor_history_bev_;
    case SensorType::kRadar:
      return sensor_history_radar_;
    default:
      static std::deque<FusedObject> empty;
      return empty;
  }
}

std::deque<FusedObject>& Track::GetSensorHistory(SensorType sensor_type) {
  switch (sensor_type) {
    case SensorType::kSvs:
      return sensor_history_svs_;
    case SensorType::kBev:
      return sensor_history_bev_;
    case SensorType::kRadar:
      return sensor_history_radar_;
    default:
      static std::deque<FusedObject> empty;
      return empty;
  }
}

FusedObject Track::GetPreviousSensorObject(SensorType sensor_type, int32_t offset) const {
  const std::deque<FusedObject>& history = GetSensorHistory(sensor_type);

  if (history.empty()) {
    return FusedObject();
  }

  // offset=0 means latest, offset=1 means previous, etc.
  if (offset < 0 || static_cast<size_t>(offset) >= history.size()) {
    return history.back();
  }

  // history[0] is oldest, history.back() is newest
  // offset=0 -> return newest (back)
  // offset=1 -> return second newest
  size_t idx = history.size() - 1 - static_cast<size_t>(offset);
  return history[idx];
}

void Track::PruneSensorHistory(SensorType sensor_type, uint64_t current_time) {
  // Get the max invisible period threshold for this sensor
  float max_period = 0.0f;
  switch (sensor_type) {
    case SensorType::kSvs:
      max_period = s_max_invisible_period_svs_;
      break;
    case SensorType::kBev:
      max_period = s_max_invisible_period_bev_;
      break;
    case SensorType::kRadar:
      max_period = s_max_invisible_period_radar_;
      break;
    default:
      return;
  }

  // Threshold = max_period * 2 (e.g., if max_period is 0.5s, threshold is 1.0s)
  double threshold = max_period * 2.0;

  std::deque<FusedObject>* history = nullptr;
  switch (sensor_type) {
    case SensorType::kSvs:
      history = &sensor_history_svs_;
      break;
    case SensorType::kBev:
      history = &sensor_history_bev_;
      break;
    case SensorType::kRadar:
      history = &sensor_history_radar_;
      break;
    default:
      return;
  }

  // Remove old observations (older than threshold)
  while (!history->empty()) {
    double time_diff = static_cast<double>(current_time - history->front().object.timestamp) / 1000.0;
    if (time_diff > threshold) {
      history->pop_front();
    } else {
      break;
    }
  }
}

void Track::CheckStability() {
  TrackState state = TrackState::kUntracking;
  if (status_.cnt > 0) {
    if (status_.cnt < kThresStable) {
      if (status_.cnt < 3) {
        state = TrackState::kInitial;
      } else {
        state = TrackState::kGrowing;
      }
    } else if (status_.lst >= kThresUnstable) {
      state = TrackState::kUnstable;
    } else {
      state = TrackState::kStable;
    }
  }
  status_.state = state;

  // Update alive status based on sensor visibility
  bool any_visible = IsSvsVisible() || IsBevVisible() || IsRadarVisible();
  if (!any_visible && status_.lst >= 10) {
    is_alive_ = false;
  }
}

}  // namespace fusion
}  // namespace perception
