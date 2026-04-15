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

bool IsStaticObject(ObjectType type) {
  int val = static_cast<int>(type);
  return (val > 0 && val < 8) || (val > 14 && val < 18) || (val > 19 && val < 22);
}

}  // namespace

TrackerProcessor::TrackerProcessor()
    : track_cnt_(0) {
  tracks_.resize(kTrackWidth);
  meas_flags_.resize(kMaxObsFuse, 0);
  Init();
}

void TrackerProcessor::Init() {
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    tracks_[i].SetId(i);
    tracks_[i].Reset();
  }
  meas_flags_.assign(kMaxObsFuse, 0);
  track_cnt_ = 0;
}

void TrackerProcessor::Reset() {
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    tracks_[i].Reset();
  }
  track_cnt_ = 0;
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

    float tx = tracks_[j].GetEstimated().object.x;
    float ty = tracks_[j].GetEstimated().object.y;
    float tvx = tracks_[j].GetEstimated().object.vx;
    float tvy = tracks_[j].GetEstimated().object.vy;

    int32_t prev_idx = tracks_[j].GetPreviousIndex(1);
    const FusedObject& prev_obs = tracks_[j].GetHistoryObject(prev_idx);

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
        Track& track = tracks_[j];
        track.GetTrackObject() = obs;
        track.SetMeasFlag(0);
        track.SetHistoryFlag(track.GetPreviousIndex(0), 0);

        if (obs.svs_match_id > 0) {
          track.UpdateSvsCounter(true);
        } else {
          track.UpdateSvsCounter(false);
        }

        if (obs.bev_match_id > 0) {
          track.UpdateBevCounter(true);
        } else {
          track.UpdateBevCounter(false);
        }

        if (obs.radar_match_id > 0) {
          track.UpdateRadarCounter(true);
        } else {
          track.UpdateRadarCounter(false);
        }

        if (obs.bev_match_id > 0 && obs.svs_match_id > 0) {
          track.GetStatus().cnt_fused = std::max(0, track.GetStatus().cnt_fused) + 1;
        }

        track.GetStatus().cnt++;
        track.GetStatus().lst = std::max(0, track.GetStatus().lst - 1);
        track.SetFlag(1);

        meas_assoc_flag[i] = 1;
        trs_assoc_flag[j] = 1;
        break;
      }
    }
  }

  for (int32_t j = 0; j < num_t; ++j) {
    tracks_[j].CheckStability();
  }

  for (int32_t i = 0; i < num_t; ++i) {
    Track& track = tracks_[i];
    if (track.IsUsed() && track.GetFlag()) {
      FusedObject latest_obs = track.GetTrackObject();

      uint64_t last_time = track.GetLastTrackingTime();
      int32_t time_diff = static_cast<int32_t>(latest_obs.object.timestamp - last_time);
      float dt = static_cast<float>(time_diff) / 1000.0f;
      if (dt <= 0.0f) {
        dt = kTimeDiffMin;
      }
      if (dt > kTimeDiffMax) {
        dt = kTimeDiffMax;
      }

      track.Update(latest_obs);

      float est_x = 0.0f, est_y = 0.0f, est_vx = 0.0f, est_vy = 0.0f;
      track.GetEstimate(&est_x, &est_y, &est_vx, &est_vy);

      if (std::abs(latest_obs.object.vx) < kVelocityThreshold &&
          std::abs(latest_obs.object.vy) < kVelocityThreshold) {
        est_vx = 0.0f;
        est_vy = 0.0f;
      }

      FusedObject estimated = track.GetEstimated();
      estimated.object.x = est_x;
      estimated.object.y = est_y;
      estimated.object.vx = est_vx;
      estimated.object.vy = est_vy;
      estimated.state = track.GetState();
      track.SetEstimated(estimated);

      track.SetLastTrackingTime(latest_obs.object.timestamp);
    }
  }
}

void TrackerProcessor::UpdateWithoutAssociated(uint64_t meas_time) {
  for (int32_t j = 0; j < kTrackWidth; ++j) {
    Track& track = tracks_[j];
    if (track.IsUsed() && track.GetFlag() == 0) {
      FusedObject latest_obs = track.GetTrackObject();

      int32_t prev_idx = track.GetPreviousIndex(1);
      latest_obs = track.GetHistoryObject(prev_idx);

      int32_t time_diff = static_cast<int32_t>(meas_time - track.GetLastTrackingTime());
      float dt = static_cast<float>(time_diff) / 1000.0f;
      if (dt <= 0.0f) {
        dt = kTimeDiffMin;
      }
      if (dt > kTimeDiffMax) {
        dt = kTimeDiffMax;
      }

      track.Predict(dt);

      float est_x = 0.0f, est_y = 0.0f, est_vx = 0.0f, est_vy = 0.0f;
      track.GetEstimate(&est_x, &est_y, &est_vx, &est_vy);

      if (std::abs(latest_obs.object.vx) < kVelocityThreshold &&
          std::abs(latest_obs.object.vy) < kVelocityThreshold) {
        est_vx = 0.0f;
        est_vy = 0.0f;
      }

      latest_obs.object.x = est_x;
      latest_obs.object.y = est_y;
      latest_obs.object.vx = est_vx;
      latest_obs.object.vy = est_vy;

      track.SetEstimated(latest_obs);
      track.GetTrackObject() = latest_obs;

      track.SetLastTrackingTime(meas_time);
      track.SetMeasFlag(1);
      track.SetFlag(1);
    }
  }
}

void TrackerProcessor::CreateNewTracks(
    const std::vector<FusedObject>& observations,
    const std::vector<uint8_t>& meas_valid_flag) {
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
      track_pos = track.GetTrackObject();
    } else {
      int32_t prev_idx = track.GetPreviousIndex(1);
      track_pos = track.GetHistoryObject(prev_idx);
    }

    ObjDetProp det_prop = track.GetEstimated().obj_det_prop;
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
  for (int32_t i = 0; i < kTrackWidth; ++i) {
    Track& track = tracks_[i];
    bool dynamic_flag = false;
    float vx = track.GetEstimated().object.vx;
    float vy = track.GetEstimated().object.vy;

    if (vx < 1.0f && track.GetStatus().lst >= kThresLost) {
      dynamic_flag = true;
    }

    bool lost_cnt_flag = (track.GetStatus().cnt < track.GetStatus().lst);

    if (track.IsUsed() && (dynamic_flag || lost_cnt_flag)) {
      track.Reset();
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
    Track& track = tracks_[i];
    track.SetOutput(FusedObject());

    if (track.IsUsed()) {
      FusedObject lastest_obs;
      if (track.GetFlag()) {
        lastest_obs = track.GetTrackObject();
      } else {
        int32_t prev_idx = track.GetPreviousIndex(1);
        lastest_obs = track.GetHistoryObject(prev_idx);
      }

      bool cnt_flag = (track.GetStatus().cnt > kThresOutput);

      FusedObject out;
      out.object.x = track.GetEstimated().object.x;
      out.object.y = track.GetEstimated().object.y;

      if (track.GetStatus().cnt > 50) {
        out.object.vx = track.GetEstimated().object.vx;
      } else {
        out.object.vx = 0.0f;
      }
      out.object.vy = track.GetEstimated().object.vy;

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

      if (cnt_flag) {
        (*is_fill)[i] = true;
      }

      track.SetOutput(out);
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
        obj.object.id = static_cast<uint8_t>(ii);
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
    results->push_back(tracks_[i].GetOutput());
  }
}

}  // namespace fusion
}  // namespace perception