#include "od_fusion/lib/tracker_processor.h"

#include <cmath>
#include <algorithm>
#include "od_fusion/lib/hungarian.h"

namespace perception {
namespace fusion {

namespace {

constexpr int32_t kThresLost = 10;
constexpr int32_t kThresStable = 3;
constexpr int32_t kThresUnstable = 2;
constexpr int32_t kThresOutput = 7;
constexpr int32_t kThresIdAssign = 20;
constexpr float kStaticCntLow = 20.0f;
constexpr float kStaticDisThres = 15.0f;
constexpr float kVelocityThreshold = 1e-4f;
constexpr float kTimeDiffMin = 0.05f;
constexpr float kTimeDiffMax = 0.5f;
constexpr float kDistGate = 20.0f;
constexpr float kPosWeight = 1.0f;
constexpr float kVelWeight = 0.1f;
constexpr float kCostThreshold = 10.0f;
constexpr float kIdMatchCost = 0.01f;

bool IsStaticObject(ObjectType type) {
  int val = static_cast<int>(type);
  return (val > 0 && val < 8) || (val > 14 && val < 18) || (val > 19 && val < 22);
}

}  // namespace

TrackerProcessor::TrackerProcessor()
    : track_cnt_(0), cursor_(1) {
  Init();
}

void TrackerProcessor::Init() {
  const int32_t matrix_size = kTrackDepth * kTrackWidth;
  matrix_.resize(matrix_size);
  lst_flg_.assign(matrix_size, 0);
  status_.resize(kTrackWidth);
  kalman_filters_.resize(kTrackWidth);
  estimated_.resize(kTrackWidth);
  output_.resize(kTrackWidth);
  meas_flags_.resize(kMaxObsFuse, 0);
  cursor_ = 1;
  track_cnt_ = 0;
}

void TrackerProcessor::Reset() {
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    ResetTrack(i);
  }
  track_cnt_ = 0;
  cursor_ = 1;
}

void TrackerProcessor::ResetTrack(int32_t slot_idx) {
  if (slot_idx < 0 || slot_idx >= kTrackWidth) {
    return;
  }
  status_[slot_idx].cnt = 0;
  status_[slot_idx].cnt_svs = 0;
  status_[slot_idx].cnt_bev = 0;
  status_[slot_idx].cnt_radar = 0;
  status_[slot_idx].cnt_fused = 0;
  status_[slot_idx].lst = 0;
  status_[slot_idx].lst_svs = 0;
  status_[slot_idx].lst_bev = 0;
  status_[slot_idx].lst_radar = 0;
  status_[slot_idx].used = 0;
  status_[slot_idx].flag = 0;
  status_[slot_idx].fused_type = ObjectType::kUnknown;
  status_[slot_idx].state = TrackState::kUntracking;

  for (int d = 0; d < kTrackDepth; ++d) {
    matrix_[d * kTrackWidth + slot_idx] = FusedObject();
  }
  for (int d = 0; d < kTrackDepth; ++d) {
    lst_flg_[d * kTrackWidth + slot_idx] = 0;
  }

  output_[slot_idx] = FusedObject();
  estimated_[slot_idx] = FusedObject();
}

int32_t TrackerProcessor::GetPreviousIndex(int32_t offset) const {
  int32_t idx = static_cast<int32_t>(cursor_) - offset - 1;
  if (idx < 1) {
    idx += kTrackDepth;
  }
  return idx;
}

int32_t TrackerProcessor::GetNextCursor() const {
  if (cursor_ >= kTrackDepth) {
    return 1;
  }
  return cursor_ + 1;
}

FusedObject& TrackerProcessor::GetTrackObject(int32_t track_idx) {
  return matrix_[(cursor_ - 1) * kTrackWidth + track_idx];
}

const FusedObject& TrackerProcessor::GetTrackObject(int32_t track_idx) const {
  return matrix_[(cursor_ - 1) * kTrackWidth + track_idx];
}

void TrackerProcessor::UpdateCounter(int32_t* cnt, int32_t* lst, bool valid) {
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

bool TrackerProcessor::IsTrackReplaceable(int32_t idx) const {
  if (idx < 0 || idx >= kTrackWidth) {
    return false;
  }
  ObjDetProp det_prop = output_[idx].obj_det_prop;
  return !(det_prop == ObjDetProp::kSoleRadar || det_prop == ObjDetProp::kFused);
}

void TrackerProcessor::Process(
    const std::vector<FusedObject>& observations,
    const GlobalPose& glb,
    uint64_t meas_time,
    SensorType sensor_type,
    std::vector<FusedObject>* output) {
  if (output == nullptr) {
    return;
  }
  output->clear();

  meas_flags_.assign(kMaxObsFuse, 0);

  Eigen::MatrixXi match_result;
  AssociateTracks(observations, meas_time, &match_result);

  UpdateWithAssociated(observations, match_result, glb);

  UpdateWithoutAssociated(meas_time);

  if (sensor_type == SensorType::kSvs) {
    std::vector<uint8_t> meas_valid(kMaxObsFuse, 0);
    for (size_t i = 0; i < observations.size() && i < kMaxObsFuse; ++i) {
      meas_valid[i] = (observations[i].object.flag > 0) ? 1 : 0;
    }
    CreateNewTracks(observations, meas_valid);
  }

  std::vector<bool> is_fill;
  UpdateMotSupplementState(meas_time, &is_fill);

  PostProcess(is_fill, output);
}

void TrackerProcessor::AssociateTracks(
    const std::vector<FusedObject>& observations,
    uint64_t meas_time,
    Eigen::MatrixXi* match_result) {
  const int32_t num_m = kMaxObsFuse;
  const int32_t num_t = kTrackWidth;

  if (match_result == nullptr) {
    return;
  }
  match_result->setZero(num_m, num_t);

  bool has_valid_obs = false;
  for (const auto& obs : observations) {
    if (obs.object.flag > 0) {
      has_valid_obs = true;
      break;
    }
  }
  if (!has_valid_obs) {
    return;
  }

  bool has_used_track = false;
  for (const auto& s : status_) {
    if (s.used > 0) {
      has_used_track = true;
      break;
    }
  }
  if (!has_used_track) {
    return;
  }

  int32_t last_idx = GetPreviousIndex(1);

  Eigen::MatrixXf cost_matrix = Eigen::MatrixXf::Ones(num_m, num_t) * 10000.0f;

  for (int32_t j = 0; j < num_t; ++j) {
    if (status_[j].used == 0) {
      continue;
    }

    float tx = estimated_[j].object.x;
    float ty = estimated_[j].object.y;
    float tvx = estimated_[j].object.vx;
    float tvy = estimated_[j].object.vy;

    for (int32_t i = 0; i < num_m && i < static_cast<int32_t>(observations.size()); ++i) {
      const auto& obs = observations[i];
      if (obs.object.flag == 0) {
        continue;
      }

      bool id_matched = false;
      if (last_idx >= 0 && last_idx < kTrackDepth) {
        FusedObject prev_obs = matrix_[last_idx * kTrackWidth + j];
        if (prev_obs.object.flag == 1) {
          if (obs.svs_match_id > 0 && obs.svs_match_id == prev_obs.svs_match_id) {
            id_matched = true;
          }
          if (obs.bev_match_id > 0 && obs.bev_match_id == prev_obs.bev_match_id) {
            id_matched = true;
          }
          if (obs.radar_match_id > 0 && obs.radar_match_id == prev_obs.radar_match_id) {
            id_matched = true;
          }
        }
      }

      if (id_matched) {
        cost_matrix(i, j) = kIdMatchCost;
        continue;
      }

      if (obs.object.type != matrix_[last_idx * kTrackWidth + j].object.type) {
        continue;
      }

      float dx = obs.object.x - tx;
      float dy = obs.object.y - ty;
      float dist_sq = dx * dx + dy * dy;

      if (dist_sq < kDistGate * kDistGate) {
        float dvx = obs.object.vx - tvx;
        float dvy = obs.object.vy - tvy;
        float vel_sq = dvx * dvx + dvy * dvy;
        cost_matrix(i, j) = std::sqrt(dist_sq * kPosWeight + vel_sq * kVelWeight);
      }
    }
  }

  Hungarian::Solve(cost_matrix, match_result);

  for (int32_t i = 0; i < num_m; ++i) {
    for (int32_t j = 0; j < num_t; ++j) {
      if ((*match_result)(i, j) > 0 && cost_matrix(i, j) < kCostThreshold) {
        match_result->operator()(i, j) = 1;
      } else {
        match_result->operator()(i, j) = 0;
      }
    }
  }
}

void TrackerProcessor::UpdateWithAssociated(
    const std::vector<FusedObject>& observations,
    const Eigen::MatrixXi& match_result,
    const GlobalPose& glb) {
  const int32_t num_m = kMaxObsFuse;
  const int32_t num_t = kTrackWidth;

  std::vector<int32_t> meas_assoc_flag(num_m, 0);
  std::vector<int32_t> trs_assoc_flag(num_t, 0);

  for (int32_t i = 0; i < num_m; ++i) {
    if (i >= static_cast<int32_t>(observations.size())) {
      break;
    }
    const auto& obs = observations[i];
    if (obs.object.flag == 0) {
      continue;
    }

    for (int32_t j = 0; j < num_t; ++j) {
      if (match_result(i, j) > 0) {
        UpdateTracksStatus(j, obs);
        meas_assoc_flag[i] = 1;
        trs_assoc_flag[j] = 1;
        break;
      }
    }
  }

  for (int32_t j = 0; j < num_t; ++j) {
    CheckStability(j);
  }

  for (int32_t i = 0; i < num_t; ++i) {
    if (status_[i].used && status_[i].flag) {
      FusedObject lastest_obs = GetTrackObject(i);

      uint64_t last_time = status_[i].last_tracking_time;
      int32_t time_diff = static_cast<int32_t>(lastest_obs.object.timestamp - last_time);
      float dt = static_cast<float>(time_diff) / 1000.0f;
      if (dt <= 0.0f) {
        dt = kTimeDiffMin;
      }
      if (dt > kTimeDiffMax) {
        dt = kTimeDiffMax;
      }

      kalman_filters_[i].Update(
          lastest_obs.object.x, lastest_obs.object.y,
          lastest_obs.object.vx, lastest_obs.object.vy,
          lastest_obs.object.type);

      float est_x = 0.0f, est_y = 0.0f, est_vx = 0.0f, est_vy = 0.0f;
      kalman_filters_[i].GetEstimate(&est_x, &est_y, &est_vx, &est_vy);

      if (std::abs(lastest_obs.object.vx) < kVelocityThreshold &&
          std::abs(lastest_obs.object.vy) < kVelocityThreshold) {
        est_vx = 0.0f;
        est_vy = 0.0f;
      }

      estimated_[i].object.x = est_x;
      estimated_[i].object.y = est_y;
      estimated_[i].object.vx = est_vx;
      estimated_[i].object.vy = est_vy;
      estimated_[i].state = status_[i].state;

      status_[i].last_tracking_time = lastest_obs.object.timestamp;
    }
  }
}

void TrackerProcessor::UpdateWithoutAssociated(uint64_t meas_time) {
  for (int32_t j = 0; j < kTrackWidth; ++j) {
    if (status_[j].used == 1 && status_[j].flag == 0) {
      FusedObject lastest_obs = GetTrackObject(j);

      int32_t prev_idx = GetPreviousIndex(1);
      if (prev_idx >= 0 && prev_idx < kTrackDepth) {
        lastest_obs = matrix_[prev_idx * kTrackWidth + j];
      }

      int32_t time_diff = static_cast<int32_t>(meas_time - status_[j].last_tracking_time);
      float dt = static_cast<float>(time_diff) / 1000.0f;
      if (dt <= 0.0f) {
        dt = kTimeDiffMin;
      }
      if (dt > kTimeDiffMax) {
        dt = kTimeDiffMax;
      }

      kalman_filters_[j].Predict(dt);

      float est_x = 0.0f, est_y = 0.0f, est_vx = 0.0f, est_vy = 0.0f;
      kalman_filters_[j].GetEstimate(&est_x, &est_y, &est_vx, &est_vy);

      if (std::abs(lastest_obs.object.vx) < kVelocityThreshold &&
          std::abs(lastest_obs.object.vy) < kVelocityThreshold) {
        est_vx = 0.0f;
        est_vy = 0.0f;
      }

      lastest_obs.object.x = est_x;
      lastest_obs.object.y = est_y;
      lastest_obs.object.vx = est_vx;
      lastest_obs.object.vy = est_vy;

      estimated_[j] = lastest_obs;
      GetTrackObject(j) = lastest_obs;

      status_[j].last_tracking_time = meas_time;
      meas_flags_[j] = 1;
      status_[j].flag = 1;
    }
  }
}

void TrackerProcessor::CheckStability(int32_t track_idx) {
  if (track_idx < 0 || track_idx >= kTrackWidth) {
    return;
  }

  TrackState state = TrackState::kUntracking;
  if (status_[track_idx].cnt > 0) {
    if (status_[track_idx].cnt < kThresStable) {
      if (status_[track_idx].cnt < 3) {
        state = TrackState::kInitial;
      } else {
        state = TrackState::kGrowing;
      }
    } else if (status_[track_idx].lst >= kThresUnstable) {
      state = TrackState::kUnstable;
    } else {
      state = TrackState::kStable;
    }
  }
  status_[track_idx].state = state;
}

void TrackerProcessor::UpdateTracksStatus(int32_t trs_id,
                                            const FusedObject& raw_meas) {
  if (trs_id < 0 || trs_id >= kTrackWidth) {
    return;
  }
  if (raw_meas.object.flag == 0) {
    return;
  }

  GetTrackObject(trs_id) = raw_meas;
  meas_flags_[trs_id] = 0;
  lst_flg_[(cursor_ - 1) * kTrackWidth + trs_id] = 0;

  if (raw_meas.svs_match_id > 0) {
    UpdateCounter(&status_[trs_id].cnt_svs, &status_[trs_id].lst_svs, true);
  } else {
    UpdateCounter(&status_[trs_id].cnt_svs, &status_[trs_id].lst_svs, false);
  }

  if (raw_meas.bev_match_id > 0) {
    UpdateCounter(&status_[trs_id].cnt_bev, &status_[trs_id].lst_bev, true);
  } else {
    UpdateCounter(&status_[trs_id].cnt_bev, &status_[trs_id].lst_bev, false);
  }

  if (raw_meas.radar_match_id > 0) {
    UpdateCounter(&status_[trs_id].cnt_radar, &status_[trs_id].lst_radar, true);
  } else {
    UpdateCounter(&status_[trs_id].cnt_radar, &status_[trs_id].lst_radar, false);
  }

  if (raw_meas.bev_match_id > 0 && raw_meas.svs_match_id > 0) {
    status_[trs_id].cnt_fused = std::max(0, status_[trs_id].cnt_fused) + 1;
  }

  if (status_[trs_id].cnt_fused >= kThresIdAssign && raw_meas.svs_match_id > 0) {
    status_[trs_id].fused_type = raw_meas.object.type;
  }

  UpdateCounter(&status_[trs_id].cnt, &status_[trs_id].lst, true);
  status_[trs_id].flag = 1;
}

void TrackerProcessor::CreateNewTracks(
    const std::vector<FusedObject>& observations,
    const std::vector<uint8_t>& meas_valid_flag) {
  for (size_t obs_idx = 0; obs_idx < kMaxObsFuse && obs_idx < observations.size(); ++obs_idx) {
    if (obs_idx < meas_valid_flag.size() && meas_valid_flag[obs_idx] == 0 &&
        observations[obs_idx].object.flag) {
      int32_t slot_idx = FindEmptyTrackSlot();
      if (slot_idx > 0 && slot_idx < kTrackWidth) {
        InitializeTrack(slot_idx, observations[obs_idx]);
      } else {
        int32_t replace_idx = FindReplaceableTrack(observations[obs_idx]);
        if (replace_idx != 255) {
          ResetTrack(replace_idx);
          InitializeTrack(replace_idx, observations[obs_idx]);
        }
      }
    }
  }
}

int32_t TrackerProcessor::FindEmptyTrackSlot() const {
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    if (status_[i].used == 0) {
      return i;
    }
  }
  return 255;
}

int32_t TrackerProcessor::FindReplaceableTrack(const FusedObject& obs) const {
  float max_dist_x = 0.0f;
  float max_dist_y = 2.5f;
  int32_t replace_idx = 255;

  for (int32_t idx = 0; idx < kTrackWidth; ++idx) {
    FusedObject track_pos;
    if (status_[idx].flag) {
      track_pos = matrix_[(cursor_ - 1) * kTrackWidth + idx];
    } else {
      int32_t prev_idx = GetPreviousIndex(1);
      if (prev_idx >= 0 && prev_idx < kTrackDepth) {
        track_pos = matrix_[prev_idx * kTrackWidth + idx];
      }
    }

    if (IsTrackReplaceable(idx) &&
        (std::abs(track_pos.object.x) > max_dist_x ||
         std::abs(track_pos.object.y) > max_dist_y)) {
      if (std::abs(track_pos.object.x) > max_dist_x) {
        max_dist_x = std::abs(track_pos.object.x);
      }
      if (std::abs(track_pos.object.y) > max_dist_y) {
        max_dist_y = std::abs(track_pos.object.y);
      }
      replace_idx = idx;
    }
  }

  bool should_replace =
      (std::abs(obs.object.x) < max_dist_x) || (std::abs(obs.object.y) < max_dist_y);
  if (!should_replace) {
    return 255;
  }
  return replace_idx;
}

void TrackerProcessor::InitializeTrack(int32_t slot_idx,
                                        const FusedObject& obs) {
  if (slot_idx < 0 || slot_idx >= kTrackWidth) {
    return;
  }

  GetTrackObject(slot_idx) = obs;

  status_[slot_idx].used = 1;
  status_[slot_idx].flag = 1;
  status_[slot_idx].cnt = 1;
  status_[slot_idx].lst = 0;

  if (obs.svs_match_id > 0) {
    status_[slot_idx].cnt_svs = 1;
    status_[slot_idx].lst_svs = 0;
  }
  if (obs.bev_match_id > 0) {
    status_[slot_idx].cnt_bev = 1;
    status_[slot_idx].lst_bev = 0;
  }
  if (obs.radar_match_id > 0) {
    status_[slot_idx].cnt_radar = 1;
    status_[slot_idx].lst_radar = 0;
  }

  kalman_filters_[slot_idx].Init(
      obs.object.x, obs.object.y, obs.object.vx, obs.object.vy,
      obs.object.type);

  estimated_[slot_idx].object.x = obs.object.x;
  estimated_[slot_idx].object.y = obs.object.y;
  estimated_[slot_idx].object.vx = obs.object.vx;
  estimated_[slot_idx].object.vy = obs.object.vy;
  estimated_[slot_idx].state = status_[slot_idx].state;

  status_[slot_idx].last_tracking_time = obs.object.timestamp;
}

void TrackerProcessor::RemoveLostTrack() {
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    bool dynamic_flag = false;
    float vx = estimated_[i].object.vx;
    float vy = estimated_[i].object.vy;

    if (vx < 1.0f && status_[i].lst >= kThresLost) {
      dynamic_flag = true;
    }

    bool lost_cnt_flag = (status_[i].cnt < status_[i].lst);

    if (status_[i].used == 1 && (dynamic_flag || lost_cnt_flag)) {
      ResetTrack(i);
    }
  }
}

void TrackerProcessor::UpdateMotSupplementState(uint64_t meas_time,
                                                  std::vector<bool>* is_fill) {
  if (is_fill == nullptr) {
    return;
  }
  is_fill->assign(kTrackWidth, false);

  for (int32_t i = 0; i < kTrackWidth; ++i) {
    output_[i] = FusedObject();

    if (status_[i].used) {
      FusedObject lastest_obs;
      if (status_[i].flag) {
        lastest_obs = GetTrackObject(i);
      } else {
        int32_t prev_idx = GetPreviousIndex(1);
        if (prev_idx >= 0 && prev_idx < kTrackDepth) {
          lastest_obs = matrix_[prev_idx * kTrackWidth + i];
        }
      }

      bool cnt_flag = (status_[i].cnt > kThresOutput);

      if (true) {
        output_[i].object.x = estimated_[i].object.x;
        output_[i].object.y = estimated_[i].object.y;

        if (status_[i].cnt > 50) {
          output_[i].object.vx = estimated_[i].object.vx;
        } else {
          output_[i].object.vx = 0.0f;
        }
        output_[i].object.vy = estimated_[i].object.vy;

        output_[i].state = status_[i].state;
        output_[i].object.type = lastest_obs.object.type;
        output_[i].object.yaw = lastest_obs.object.yaw;
        output_[i].object.length = lastest_obs.object.length;
        output_[i].object.width = lastest_obs.object.width;
        output_[i].object.height = lastest_obs.object.height;
        output_[i].obj_det_prop = lastest_obs.obj_det_prop;
        output_[i].svs_match_id = lastest_obs.svs_match_id;
        output_[i].bev_match_id = lastest_obs.bev_match_id;
        output_[i].radar_match_id = lastest_obs.radar_match_id;
        output_[i].object.motion_status = lastest_obs.object.motion_status;

        if (cnt_flag) {
          (*is_fill)[i] = true;
        }
      }
    }
    status_[i].flag = 0;
  }

  cursor_ = GetNextCursor();
  track_cnt_ = track_cnt_ + 1;
}

void TrackerProcessor::PostProcess(const std::vector<bool>& is_fill,
                                     std::vector<FusedObject>* output) {
  if (output == nullptr) {
    return;
  }
  output->clear();

  int32_t idx_f = 0;
  for (int32_t ii = 0; ii < kTrackWidth && ii < kMaxOutputTracks; ++ii) {
    if (is_fill[ii]) {
      if (idx_f < kMaxOutputTracks) {
        FusedObject obj;
        obj.object.id = static_cast<uint8_t>(ii);
        obj.svs_match_id = output_[ii].svs_match_id;
        obj.bev_match_id = output_[ii].bev_match_id;
        obj.object.x = output_[ii].object.x;
        obj.object.y = output_[ii].object.y;
        obj.object.vx = output_[ii].object.vx;
        obj.object.vy = output_[ii].object.vy;
        obj.object.yaw = output_[ii].object.yaw;
        obj.object.length = output_[ii].object.length;
        obj.object.height = output_[ii].object.height;
        obj.object.width = output_[ii].object.width;
        obj.object.type = output_[ii].object.type;
        obj.state = output_[ii].state;
        obj.object.ay = output_[ii].object.ay;
        obj.object.motion_status = output_[ii].object.motion_status;
        obj.obj_det_prop = output_[ii].obj_det_prop;
        obj.object.flag = 1;
        output->push_back(obj);
        idx_f++;
      }
    }
  }
}

void TrackerProcessor::GetResults(std::vector<FusedObject>* results) const {
  if (results == nullptr) {
    return;
  }
  *results = output_;
}

}  // namespace fusion
}  // namespace perception
