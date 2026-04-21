#include "od_fusion/lib/tracker_processor.h"

#include <cmath>
#include <algorithm>
#include "od_fusion/lib/hungarian.h"

namespace perception {
namespace fusion {

namespace {

constexpr int32_t kThresLost = 10;
constexpr int32_t kThresOutput = 7;
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

}  // namespace

TrackerProcessor::TrackerProcessor()
    : track_cnt_(0) {
  tracks_.resize(kTrackWidth);
  Init();
}

void TrackerProcessor::Init() {
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    tracks_[i].SetId(i);
    tracks_[i].Reset();
  }
  track_cnt_ = 0;
}

void TrackerProcessor::Reset() {
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    tracks_[i].Reset();
  }
  track_cnt_ = 0;
  Track::ResetTrackIdCounter();
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
  std::cout << "beigin AssociateTracks::sensor_type " << static_cast<int>(sensor_type) << std::endl;
  Eigen::MatrixXi match_result;
  AssociateTracks(observations, meas_time, sensor_type, &match_result);
  std::cout << "beigin UpdateAssignedTracks." << std::endl;
  // Collect meas_assoc_flag and update assigned tracks
  std::vector<int32_t> meas_assoc_flag(kMaxObsFuse, 0);
  UpdateAssignedTracks(observations, match_result, sensor_type, meas_time, meas_assoc_flag);
  std::cout << "beigin UpdateUnassignedTracks." << std::endl;
  // Update unassigned tracks (no measurement this frame)
  UpdateUnassignedTracks(sensor_type, meas_time);
  std::cout << "beigin CreateNewTracks." << std::endl;
  // Create new tracks from unmatched observations (only for SVS)
  if (sensor_type == SensorType::kSvs) {
    CreateNewTracks(observations, meas_assoc_flag);
  }
  std::cout << "beigin RemoveLostTrack." << std::endl;
  RemoveLostTrack();
  std::cout << "beigin UpdateMotSupplementState." << std::endl;
  std::vector<bool> is_fill;
  UpdateMotSupplementState(meas_time, &is_fill);
  std::cout << "beigin PostProcess." << std::endl;
  PostProcess(is_fill, output);
}

void TrackerProcessor::AssociateTracks(
    const std::vector<FusedObject>& observations,
    uint64_t meas_time,
    SensorType sensor_type,
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
  for (const auto& track : tracks_) {
    if (track.IsUsed()) {
      has_used_track = true;
      break;
    }
  }
  if (!has_used_track) {
    return;
  }

  Eigen::MatrixXf cost_matrix = Eigen::MatrixXf::Ones(num_m, num_t) * 10000.0f;

  for (int32_t j = 0; j < num_t; ++j) {
    if (!tracks_[j].IsUsed()) {
      continue;
    }

    float tx = tracks_[j].GetFusedObject().object.x;
    float ty = tracks_[j].GetFusedObject().object.y;
    float tvx = tracks_[j].GetFusedObject().object.vx;
    float tvy = tracks_[j].GetFusedObject().object.vy;

    // Get previous observation from SVS history for ID matching
    const FusedObject& prev_obs = tracks_[j].GetPreviousSensorObject(SensorType::kSvs, 1);

    for (int32_t i = 0; i < num_m && i < static_cast<int32_t>(observations.size()); ++i) {
      const auto& obs = observations[i];
      if (obs.object.flag == 0) {
        continue;
      }

      bool id_matched = false;
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

      if (id_matched) {
        cost_matrix(i, j) = kIdMatchCost;
        continue;
      }

      if (obs.object.type != prev_obs.object.type) {
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
  std::cout << "Beigin Hungarian::Solve \n";
  int num_m = cost_matrix.rows();  // 行数
  int num_t = cost_matrix.cols();  // 列数
  std::vector<std::vector<int>> costs(num_m, std::vector<int>(num_t));
  for (int i = 0; i < num_m; ++i) {
    for (int j = 0; j < num_t; ++j) {
      // float 转 int，直接赋值
      costs[i][j] = static_cast<int>(cost_matrix(i, j));
    }
  }
  if (cost_matrix.rows() > 0 && cost_matrix.cols() > 0) {
    HungarianOptimizer<int> optimizer_;
    optimizer_.costs(costs);
    std::vector<std::pair<size_t, size_t>> assignments;
    optimizer_.Minimize(&assignments);
  }
  Eigen::MatrixXi mat(assignments.size(), 2);
  for (int i = 0; i < mat.rows(); ++i) {
    mat(i, 0) = assignments[i].first;
    mat(i, 1) = assignments[i].second;
  }
  std::cout << "Beigin Assign match_result.\n";

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

void TrackerProcessor::UpdateAssignedTracks(
    const std::vector<FusedObject>& observations,
    const Eigen::MatrixXi& match_result,
    SensorType sensor_type,
    uint64_t meas_time,
    std::vector<int32_t>& meas_assoc_flag) {
  const int32_t num_m = kMaxObsFuse;
  const int32_t num_t = kTrackWidth;

  // Directly iterate through match_result to find all matched observation-track pairs
  // and process all logic at once
  for (int32_t j = 0; j < num_t; ++j) {
    for (int32_t i = 0; i < num_m && i < static_cast<int32_t>(observations.size()); ++i) {
      if (match_result(i, j) > 0) {
        const auto& obs = observations[i];
        if (obs.object.flag == 0) {
          continue;
        }

        Track& track = tracks_[j];
        meas_assoc_flag[i] = 1;

        // Update sensor-specific counters and visibility
        if (obs.svs_match_id > 0) {
          track.UpdateSvsCounter(true);
          track.SetInvisibilityPeriod(SensorType::kSvs, 0.0);
        } else {
          track.UpdateSvsCounter(false);
        }

        if (obs.bev_match_id > 0) {
          track.UpdateBevCounter(true);
          track.SetInvisibilityPeriod(SensorType::kBev, 0.0);
        } else {
          track.UpdateBevCounter(false);
        }

        if (obs.radar_match_id > 0) {
          track.UpdateRadarCounter(true);
          track.SetInvisibilityPeriod(SensorType::kRadar, 0.0);
        } else {
          track.UpdateRadarCounter(false);
        }

        if (obs.bev_match_id > 0 && obs.svs_match_id > 0) {
          track.GetStatus().cnt_fused = std::max(0, track.GetStatus().cnt_fused) + 1;
        }

        track.GetStatus().cnt++;
        track.GetStatus().lst = std::max(0, track.GetStatus().lst - 1);
        track.SetFlag(1);
        track.SetMeasFlag(0);

        // Reset prediction state
        track.ResetPredictionTime();
        track.SetPredicted(false);
        track.SetAlive(true);

        // Calculate time difference for Kalman filter
        uint64_t last_time = track.GetLastTrackingTime();
        int32_t time_diff = static_cast<int32_t>(obs.object.timestamp - last_time);
        float dt = static_cast<float>(time_diff) / 1000.0f;
        if (dt <= 0.0f) {
          dt = kTimeDiffMin;
        }
        if (dt > kTimeDiffMax) {
          dt = kTimeDiffMax;
        }

        // Kalman filter: Predict + Update
        track.Predict(dt);
        track.Update(obs);

        // Update fused object with filtered state
        float est_x = 0.0f, est_y = 0.0f, est_vx = 0.0f, est_vy = 0.0f;
        track.GetEstimate(&est_x, &est_y, &est_vx, &est_vy);

        if (std::abs(obs.object.vx) < kVelocityThreshold &&
            std::abs(obs.object.vy) < kVelocityThreshold) {
          est_vx = 0.0f;
          est_vy = 0.0f;
        }

        FusedObject estimated = track.GetFusedObject();
        estimated.object.x = est_x;
        estimated.object.y = est_y;
        estimated.object.vx = est_vx;
        estimated.object.vy = est_vy;
        estimated.state = track.GetState();
        track.SetFusedObject(estimated);

        track.SetLastTrackingTime(obs.object.timestamp);

        // Found match for this track, no need to check other observations
        break;
      }
    }
  }

  // Check stability for all tracks
  for (int32_t j = 0; j < num_t; ++j) {
    tracks_[j].CheckStability();
  }
}

void TrackerProcessor::UpdateUnassignedTracks(SensorType sensor_type, uint64_t meas_time) {
  for (int32_t j = 0; j < kTrackWidth; ++j) {
    Track& track = tracks_[j];
    // Skip tracks that were assigned (flag == 1) or not used
    if (!track.IsUsed() || track.GetFlag() == 1) {
      continue;
    }

    // Prune all sensor histories and update invisibility for each
    track.PruneSensorHistory(SensorType::kSvs, meas_time);
    track.PruneSensorHistory(SensorType::kBev, meas_time);
    track.PruneSensorHistory(SensorType::kRadar, meas_time);

    // Update invisibility period for the current sensor (which didn't provide measurement)
    track.UpdateWithoutSensorObject(sensor_type, meas_time);

    // Get previous observation from the appropriate sensor history for prediction
    FusedObject prev_obs = track.GetPreviousSensorObject(sensor_type, 1);

    // Calculate time difference
    int32_t time_diff = static_cast<int32_t>(meas_time - prev_obs.object.timestamp);
    float dt = static_cast<float>(time_diff) / 1000.0f;
    if (dt <= 0.0f) {
      dt = kTimeDiffMin;
    }
    if (dt > kTimeDiffMax) {
      dt = kTimeDiffMax;
    }

    // Kalman filter: Predict only (no measurement update)
    track.Predict(dt);

    // Update fused object with predicted state
    float est_x = 0.0f, est_y = 0.0f, est_vx = 0.0f, est_vy = 0.0f;
    track.GetEstimate(&est_x, &est_y, &est_vx, &est_vy);

    if (std::abs(prev_obs.object.vx) < kVelocityThreshold &&
        std::abs(prev_obs.object.vy) < kVelocityThreshold) {
      est_vx = 0.0f;
      est_vy = 0.0f;
    }

    prev_obs.object.x = est_x;
    prev_obs.object.y = est_y;
    prev_obs.object.vx = est_vx;
    prev_obs.object.vy = est_vy;

    track.SetFusedObject(prev_obs);

    track.SetLastTrackingTime(meas_time);
    track.SetMeasFlag(1);
    track.SetFlag(1);

    // Check if all sensor histories are empty - if so, mark track as not alive
    if (track.GetSensorHistory(SensorType::kSvs).empty() &&
        track.GetSensorHistory(SensorType::kBev).empty() &&
        track.GetSensorHistory(SensorType::kRadar).empty()) {
      track.SetAlive(false);
    }
  }
}

void TrackerProcessor::CreateNewTracks(
    const std::vector<FusedObject>& observations,
    const std::vector<int>& meas_valid_flag) {
  for (size_t obs_idx = 0; obs_idx < kMaxObsFuse && obs_idx < observations.size(); ++obs_idx) {
    if (obs_idx < meas_valid_flag.size() && meas_valid_flag[obs_idx] == 0 &&
        observations[obs_idx].object.flag) {
      int32_t slot_idx = FindEmptyTrackSlot();
      if (slot_idx >= 0 && slot_idx < kTrackWidth) {
        tracks_[slot_idx].Initialize(observations[obs_idx]);
      } else {
        int32_t replace_idx = FindReplaceableTrack(observations[obs_idx]);
        if (replace_idx != -1) {
          tracks_[replace_idx].Reset();
          tracks_[replace_idx].Initialize(observations[obs_idx]);
        }
      }
    }
  }
}

int32_t TrackerProcessor::FindEmptyTrackSlot() const {
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    if (!tracks_[i].IsUsed()) {
      return i;
    }
  }
  return -1;
}

int32_t TrackerProcessor::FindReplaceableTrack(const FusedObject& obs) const {
  float max_dist_x = 0.0f;
  float max_dist_y = 2.5f;
  int32_t replace_idx = -1;

  for (int32_t idx = 0; idx < kTrackWidth; ++idx) {
    const Track& track = tracks_[idx];
    FusedObject track_pos;
    if (track.GetFlag()) {
      track_pos = track.GetFusedObject();
    } else {
      track_pos = track.GetPreviousSensorObject(SensorType::kSvs, 1);
    }

    ObjDetProp det_prop = track.GetFusedObject().obj_det_prop;
    bool is_replaceable = !(det_prop == ObjDetProp::kSoleRadar ||
                            det_prop == ObjDetProp::kFused);

    if (is_replaceable &&
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
    return -1;
  }
  return replace_idx;
}

void TrackerProcessor::RemoveLostTrack() {
  int del_track_cnt = 0;
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    Track& track = tracks_[i];
    bool dynamic_flag = false;
    float vx = track.GetFusedObject().object.vx;
    float vy = track.GetFusedObject().object.vy;

    if (vx < 1.0f && track.GetStatus().lst >= kThresLost) {
      dynamic_flag = true;
    }

    bool lost_cnt_flag = (track.GetStatus().cnt < track.GetStatus().lst);

    // Check if track is alive based on sensor visibility
    bool any_visible = track.IsSvsVisible() || track.IsBevVisible() || track.IsRadarVisible();

    if (track.IsUsed() && (dynamic_flag || lost_cnt_flag || !any_visible)) {
      track.Reset();
      del_track_cnt++;
    }
  }
  std::cout << "RemoveLostTrack::Removed " << del_track_cnt << " lost tracks."<< std::endl;
}

void TrackerProcessor::UpdateMotSupplementState(uint64_t meas_time,
                                                std::vector<bool>* is_fill) {
  if (is_fill == nullptr) {
    return;
  }
  is_fill->assign(kTrackWidth, false);

  for (int32_t i = 0; i < kTrackWidth; ++i) {
    Track& track = tracks_[i];
    track.SetOutput(FusedObject());

    if (track.IsUsed()) {
      FusedObject lastest_obs = track.GetFusedObject();

      bool cnt_flag = (track.GetStatus().cnt > kThresOutput);

      // Only output tracks that are alive and have reached output threshold
      if (track.IsAlive() && cnt_flag) {
        FusedObject out;
        out.object.id = static_cast<uint8_t>(track.GetId());
        out.object.x = track.GetFusedObject().object.x;
        out.object.y = track.GetFusedObject().object.y;

        if (track.GetStatus().cnt > 50) {
          out.object.vx = track.GetFusedObject().object.vx;
        } else {
          out.object.vx = 0.0f;
        }
        out.object.vy = track.GetFusedObject().object.vy;

        out.state = track.GetState();
        out.object.type = lastest_obs.object.type;
        out.object.yaw = lastest_obs.object.yaw;
        out.object.length = lastest_obs.object.length;
        out.object.width = lastest_obs.object.width;
        out.object.height = lastest_obs.object.height;
        out.obj_det_prop = lastest_obs.obj_det_prop;
        out.svs_match_id = lastest_obs.svs_match_id;
        out.bev_match_id = lastest_obs.bev_match_id;
        out.radar_match_id = lastest_obs.radar_match_id;
        out.object.motion_status = lastest_obs.object.motion_status;

        (*is_fill)[i] = true;
        track.SetOutput(out);
      }
    }
    track.SetFlag(0);
  }

  track_cnt_++;
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
        const FusedObject& out = tracks_[ii].GetOutput();
        FusedObject obj;
        obj.object.id = static_cast<uint8_t>(tracks_[ii].GetId());
        obj.svs_match_id = out.svs_match_id;
        obj.bev_match_id = out.bev_match_id;
        obj.object.x = out.object.x;
        obj.object.y = out.object.y;
        obj.object.vx = out.object.vx;
        obj.object.vy = out.object.vy;
        obj.object.yaw = out.object.yaw;
        obj.object.length = out.object.length;
        obj.object.height = out.object.height;
        obj.object.width = out.object.width;
        obj.object.type = out.object.type;
        obj.state = out.state;
        obj.object.ay = out.object.ay;
        obj.object.motion_status = out.object.motion_status;
        obj.obj_det_prop = out.obj_det_prop;
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
  results->clear();
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    if (tracks_[i].IsUsed()) {
      results->push_back(tracks_[i].GetOutput());
    }
  }
}

}  // namespace fusion
}  // namespace perception
