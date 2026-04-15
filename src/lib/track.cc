#include "od_fusion/lib/track.h"

#include <algorithm>

namespace perception {
namespace fusion {

namespace {
constexpr int32_t kThresStable = 3;
constexpr int32_t kThresUnstable = 2;
constexpr int32_t kThresIdAssign = 20;
constexpr float kVelocityThreshold = 1e-4f;
}  // namespace

Track::Track()
    : track_id_(0),
      cursor_(1),
      meas_flag_(0) {
  const int32_t matrix_size = kTrackDepth * kTrackWidth;
  matrix_.resize(matrix_size);
  lst_flg_.assign(matrix_size, 0);
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
}

void Track::Initialize(const FusedObject& obs) {
  GetTrackObject() = obs;

  status_.used = 1;
  status_.flag = 1;
  status_.cnt = 1;
  status_.lst = 0;

  if (obs.svs_match_id > 0) {
    status_.cnt_svs = 1;
    status_.lst_svs = 0;
  }
  if (obs.bev_match_id > 0) {
    status_.cnt_bev = 1;
    status_.lst_bev = 0;
  }
  if (obs.radar_match_id > 0) {
    status_.cnt_radar = 1;
    status_.lst_radar = 0;
  }

  kalman_filter_.Init(
      obs.object.x, obs.object.y, obs.object.vx, obs.object.vy,
      obs.object.type);

  estimated_.object.x = obs.object.x;
  estimated_.object.y = obs.object.y;
  estimated_.object.vx = obs.object.vx;
  estimated_.object.vy = obs.object.vy;
  estimated_.state = status_.state;

  status_.last_tracking_time = obs.object.timestamp;
}

void Track::Predict(float dt) {
  kalman_filter_.Predict(dt);
}

void Track::Update(const FusedObject& obs) {
  kalman_filter_.Update(
      obs.object.x, obs.object.y,
      obs.object.vx, obs.object.vy,
      obs.object.type);
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
}

}  // namespace fusion
}  // namespace perception