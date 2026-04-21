#pragma once

#include <vector>
#include <cstdint>
#include <Eigen/Dense>
#include <iostream>
#include "od_fusion/base/obstacle_constant.h"
#include "od_fusion/lib/track.h"

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
  // Data association
  void AssociateTracks(const std::vector<FusedObject>& observations,
                     uint64_t meas_time,
                     SensorType sensor_type,
                     Eigen::MatrixXi* match_result);

  // Update tracks that received measurements
  void UpdateAssignedTracks(const std::vector<FusedObject>& observations,
                           const Eigen::MatrixXi& match_result,
                           SensorType sensor_type,
                           uint64_t meas_time,
                           std::vector<int32_t>& meas_assoc_flag);

  // Update tracks that did NOT receive measurements (prediction only)
  void UpdateUnassignedTracks(SensorType sensor_type, uint64_t meas_time);

  // Create new tracks from unmatched observations
  void CreateNewTracks(const std::vector<FusedObject>& observations,
                      const std::vector<int>& meas_valid_flag);

  void UpdateMotSupplementState(uint64_t meas_time, std::vector<bool>* is_fill);

  void PostProcess(const std::vector<bool>& is_fill,
                   std::vector<FusedObject>* output);

  int32_t FindEmptyTrackSlot() const;

  int32_t FindReplaceableTrack(const FusedObject& obs) const;

  void RemoveLostTrack();

  int32_t track_cnt_;
  std::vector<Track> tracks_;
};

}  // namespace fusion
}  // namespace perception
