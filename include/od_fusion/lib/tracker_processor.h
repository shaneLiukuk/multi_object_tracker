#pragma once

#include <vector>
#include <cstdint>
#include <Eigen/Dense>
#include "od_fusion/base/obstacle_constant.h"
#include "od_fusion/lib/cv_kalman_filter.h"

namespace perception {
namespace fusion {

class TrackerProcessor {
 public:
  TrackerProcessor();
  ~TrackerProcessor() = default;

  void Init();

  void Reset();

  void Process(const std::vector<FusedObject>& observations,
               const GlobalPose& glb,
               uint64_t meas_time,
               SensorType sensor_type,
               std::vector<FusedObject>* output);

  void GetResults(std::vector<FusedObject>* results) const;

 private:
  void AssociateTracks(const std::vector<FusedObject>& observations,
                       uint64_t meas_time,
                       Eigen::MatrixXi* match_result);

  void UpdateWithAssociated(const std::vector<FusedObject>& observations,
                            const Eigen::MatrixXi& match_result,
                            const GlobalPose& glb);

  void UpdateWithoutAssociated(uint64_t meas_time);

  void CreateNewTracks(const std::vector<FusedObject>& observations,
                       const std::vector<uint8_t>& meas_valid_flag);

  void CheckStability(int32_t track_idx);

  void UpdateTracksStatus(int32_t trs_id, const FusedObject& raw_meas);

  void UpdateCounter(int32_t* cnt, int32_t* lst, bool valid);

  int32_t FindEmptyTrackSlot() const;

  int32_t FindReplaceableTrack(const FusedObject& obs) const;

  void InitializeTrack(int32_t slot_idx, const FusedObject& obs);

  void ResetTrack(int32_t slot_idx);

  bool IsTrackReplaceable(int32_t idx) const;

  int32_t GetPreviousIndex(int32_t offset) const;

  int32_t GetNextCursor() const;

  FusedObject& GetTrackObject(int32_t track_idx);

  const FusedObject& GetTrackObject(int32_t track_idx) const;

  void RemoveLostTrack();

  void UpdateMotSupplementState(uint64_t meas_time, std::vector<bool>* is_fill);

  void PostProcess(const std::vector<bool>& is_fill,
                   std::vector<FusedObject>* output);

  int32_t track_cnt_;
  uint8_t cursor_;
  std::vector<FusedObject> matrix_;
  std::vector<uint8_t> lst_flg_;
  std::vector<TrackStatus> status_;
  std::vector<CvKalmanFilter> kalman_filters_;
  std::vector<FusedObject> estimated_;
  std::vector<FusedObject> output_;
  std::vector<uint8_t> meas_flags_;
};

}  // namespace fusion
}  // namespace perception
