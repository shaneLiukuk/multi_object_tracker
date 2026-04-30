#pragma once

#include <memory>
#include <vector>
#include <cstdint>
#include <iostream>
#include <list>
#include <mutex>
#include <iomanip>
#include "od_fusion/base/obstacle_constant.h"
#include "od_fusion/lib/tracker_processor.h"
#include "od_fusion/lib/coordinate_transform.h"
#include "Rte_Type.h"

namespace perception {
namespace fusion {


class MultiObjectTrackerInput {
 public:
  ObjectSet svs_object_in;
  GlobalPoseEstimation global_pose_in;
  GlobalPoseBuffer global_pose_buffer_in;
  PerceptionBEVObject front_bev_object_in;
  BarriercadeSet front_barriercase_in;
  RoadLaneSet front_lane_in;
  LRRObjects radar_object_in;
  ParkingSlotSet fusion_parking_slot_in;
  CarInfoH car_info_h_in; // vehicle spd
  PerceptionCommand perception_command_in;
  IMUInfo imu_info_in;
};

class MultiObjectTrackerOutput {
 public:
  TrackedObjects fused_objects_output;
  WorkingStatus work_status;
};

class MultiObjectTracker {
 public:
  struct Config {
    bool enable_svs = true;
    bool enable_bev = false;
    bool enable_radar = false;
  };

  explicit MultiObjectTracker(const Config& config);
  ~MultiObjectTracker() = default;

  bool Init();

  bool runProcess(const MultiObjectTrackerInput& input, MultiObjectTrackerOutput* output);

  void GetFusionResults(std::vector<FusedObject>* results);

  void GetFrameData(FrameData* frame_data);

  const Config& GetConfig() const { return config_; }

 private:
  void ProcessSvs(const SvsFrame& svs_frame, const GlobalPose& svs_pose);

  void ProcessBev(const BevFrame& bev_frame,
                  const GlobalPose& bev_pose,
                  float veh_head_rear_wheel);

  void ProcessRadar(const RadarFrame& radar_frame,
                    const GlobalPose& radar_pose,
                    float veh_spd);
  bool msg2InterFrame(const MultiObjectTrackerInput& input, FrameData& frame);
  SvsFrame ConvertObjectSetToSvsFrame(const ObjectSet& msg, const GlobalPose& pose);
  BevFrame ConvertPerceptionBEVToBevFrame(const PerceptionBEVObject& msg);
  bool QueryNearestLocalization(const double& timestamp, GlobalPose& localization);
  void AddGlobalPoseToCache(const GlobalPoseEstimation& input);
  void AddGlobalPoseBufferToCache(const GlobalPoseBuffer& pose_buffer);

  void Inter2msg(const std::vector<FusedObject>& results, const GlobalPose& pose, MultiObjectTrackerOutput* output);

  std::vector<FusedObject> ConvertSvsToObservations(const SvsFrame& svs_frame);

  std::vector<FusedObject> ConvertBevToObservations(const BevFrame& bev_frame,
                                                     float veh_head_rear_wheel);

  std::vector<FusedObject> ConvertRadarToObservations(const RadarFrame& radar_frame,
                                                      float veh_spd);

  Config config_;
  int32_t fusion_counter_;
  std::unique_ptr<TrackerProcessor> tracker_processor_;
  std::list<GlobalPose> global_pose_cache_;
  GlobalPose lastest_global_pose_;
  perception::FrameData frame_data_;
  std::mutex mutex_;
};

}  // namespace fusion
}  // namespace perception
