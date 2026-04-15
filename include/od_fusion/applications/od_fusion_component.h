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
  OdFusionComponent();
  ~OdFusionComponent() = default;

  bool Init();

  void Process(const FrameData& frame_data, uint64_t meas_time);

  void GetTrackedObjects(std::vector<FusedObject>* tracked_objects);

  void GetFusionResults(std::vector<FusedObject>* results);

 private:
  void ConvertToFrameData(const FrameData& input,
                          FrameData* frame_data,
                          uint64_t* meas_time);

  int32_t fusion_counter_;
  std::unique_ptr<TrackerProcessor> tracker_processor_;
};

}  // namespace fusion
}  // namespace perception
