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

  void GetFusionResults(std::vector<FusedObject>* results);

 private:
  void ProcessSvs(const SvsFrame& svs_frame, const GlobalPose& svs_pose);

  void ProcessBev(const BevFrame& bev_frame,
                  const GlobalPose& bev_pose,
                  float veh_head_rear_wheel);

  void ProcessRadar(const RadarFrame& radar_frame,
                    const GlobalPose& radar_pose,
                    float veh_spd);

  std::vector<FusedObject> ConvertSvsToObservations(const SvsFrame& svs_frame);

  std::vector<FusedObject> ConvertBevToObservations(const BevFrame& bev_frame,
                                                     float veh_head_rear_wheel);

  std::vector<FusedObject> ConvertRadarToObservations(const RadarFrame& radar_frame,
                                                      float veh_spd);

  Config config_;
  int32_t fusion_counter_;
  std::unique_ptr<TrackerProcessor> tracker_processor_;
};

}  // namespace fusion
}  // namespace perception
