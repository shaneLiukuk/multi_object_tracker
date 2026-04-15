#pragma once

#include <memory>
#include <vector>
#include <cstdint>
#include "od_fusion/base/obstacle_constant.h"
#include "od_fusion/lib/tracker_processor.h"

namespace perception {
namespace fusion {

class OdFusionComponent {
 public:
  struct Config {
    bool enable_svs = true;
    bool enable_bev = false;
    bool enable_radar = false;
  };

  explicit OdFusionComponent(const Config& config);
  ~OdFusionComponent() = default;

  bool Init();

  void Process(const FrameData& frame_data, uint64_t meas_time);

  void ProcessSvsFrame(const SvsFrame& svs_frame, const GlobalPose& svs_pose);

  void ProcessBevFrame(const BevFrame& bev_frame,
                       const GlobalPose& bev_pose,
                       float veh_head_rear_wheel);

  void ProcessRadarFrame(const RadarFrame& radar_frame,
                         const GlobalPose& radar_pose,
                         float veh_spd);

  void GetFusionResults(std::vector<FusedObject>* results);

 private:
  FusedObject ConvertSvsObject(const SvsObject& svs_obj);

  FusedObject ConvertBevObject(const BevObject& bev_obj, float veh_head_rear_wheel);

  FusedObject ConvertRadarObject(const RadarObject& radar_obj, float veh_spd);

  Config config_;
  int32_t fusion_counter_;
  std::unique_ptr<TrackerProcessor> tracker_processor_;
};

}  // namespace fusion
}  // namespace perception
