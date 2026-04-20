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
constexpr float kPredictionThreshold = 4;  // Number of prediction cycles before marking as predicted
}  // namespace

Track::Track()
    : track_id_(0),
      cursor_(1),
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
  const int32_t matrix_size = kTrackDepth * kTrackWidth;
  matrix_.resize(matrix_size);
  lst_flg_.assign(matrix_size, 0);
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

  for (int d = 0; d < kTrackDepth; ++d) {
    matrix_[d * kTrackWidth] = FusedObject();
  }
  for (int d = 0; d < kTrackDepth; ++d) {
    lst_flg_[d * kTrackWidth] = 0;
  }

  output_ = FusedObject();
  estimated_ = FusedObject();
  meas_flag_ = 0;
  cursor_ = 1;

  // Reset new state variables
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
  GetTrackObject() = obs;

  status_.used = 1;
  status_.flag = 1;
  status_.cnt = 1;
  status_.lst = 0;

  // Initialize sensor-specific counters
  if (obs.svs_match_id > 0) {
    status_.cnt_svs = 1;
    status_.lst_svs = 0;
    last_obs_time_svs_ = obs.object.timestamp;
  }
  if (obs.bev_match_id > 0) {
    status_.cnt_bev = 1;
    status_.lst_bev = 0;
    last_obs_time_bev_ = obs.object.timestamp;
  }
  if (obs.radar_match_id > 0) {
    status_.cnt_radar = 1;
    status_.lst_radar = 0;
    last_obs_time_radar_ = obs.object.timestamp;
  }

  // Generate new track ID
  track_id_ = GenerateNewTrackId();
  std::cout << "Initialize::track_id:" << track_id_ << std::endl;

  // Initialize Kalman filter
  kalman_filter_.Init(
      obs.object.x, obs.object.y, obs.object.vx, obs.object.vy,
      obs.object.type);

  // Set estimated state
  estimated_.object.x = obs.object.x;
  estimated_.object.y = obs.object.y;
  estimated_.object.vx = obs.object.vx;
  estimated_.object.vy = obs.object.vy;
  estimated_.state = status_.state;

  status_.last_tracking_time = obs.object.timestamp;

  // Reset new state
  is_alive_ = true;
  is_predicted_ = false;
  prediction_time_ = 0;
  tracking_period_ = 0.0;
  invisibility_period_svs_ = 0.0;
  invisibility_period_bev_ = 0.0;
  invisibility_period_radar_ = 0.0;
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
}

void Track::UpdateWithoutSensorObject(SensorType sensor_type, uint64_t meas_time) {
  // Update invisibility period for the sensor that didn't provide measurement
  double time_diff = 0.0;
  // TODO(Shane Liu): Should use observation ,not last_tracking_time
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
}

void Track::UpdateSensorInvisibilityPeriod(SensorType sensor_type, uint64_t meas_time) {
  double period = 0.0;
  if (meas_time > status_.last_tracking_time) {
    period = static_cast<double>(meas_time - status_.last_tracking_time) / 1000.0;
  }

  switch (sensor_type) {
    case SensorType::kSvs:
      invisibility_period_svs_ = period;
      last_obs_time_svs_ = meas_time;
      break;
    case SensorType::kBev:
      invisibility_period_bev_ = period;
      last_obs_time_bev_ = meas_time;
      break;
    case SensorType::kRadar:
      invisibility_period_radar_ = period;
      last_obs_time_radar_ = meas_time;
      break;
    default:
      break;
  }
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
  if (status_.cnt_fused >= kThresIdAssign && estimated_.svs_match_id > 0) {
    status_.fused_type = estimated_.object.type;
  }
}

int32_t Track::GetPreviousIndex(int32_t offset) const {
  int32_t idx = static_cast<int32_t>(cursor_) - offset - 1;
  if (idx < 1) {
    idx += kTrackDepth;
  }
  return idx;
}

void Track::SetCursorNext() {
  cursor_ = GetNextCursor();
}

int32_t Track::GetNextCursor() const {
  if (cursor_ >= kTrackDepth) {
    return 1;
  }
  return cursor_ + 1;
}

FusedObject& Track::GetTrackObject() {
  return matrix_[(cursor_ - 1) * kTrackWidth];
}

const FusedObject& Track::GetTrackObject() const {
  return matrix_[(cursor_ - 1) * kTrackWidth];
}

FusedObject& Track::GetHistoryObject(int32_t depth_idx) {
  return matrix_[depth_idx * kTrackWidth];
}

const FusedObject& Track::GetHistoryObject(int32_t depth_idx) const {
  return matrix_[depth_idx * kTrackWidth];
}

void Track::SetHistoryObject(int32_t depth_idx, const FusedObject& obj) {
  matrix_[depth_idx * kTrackWidth] = obj;
}

uint8_t& Track::GetHistoryFlag(int32_t depth_idx) {
  return lst_flg_[depth_idx * kTrackWidth];
}

uint8_t Track::GetHistoryFlag(int32_t depth_idx) const {
  return lst_flg_[depth_idx * kTrackWidth];
}

void Track::SetHistoryFlag(int32_t depth_idx, uint8_t flag) {
  lst_flg_[depth_idx * kTrackWidth] = flag;
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
