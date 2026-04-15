#include "od_fusion/applications/od_fusion_component.h"

#include <cmath>

namespace perception {
namespace fusion {

namespace {

constexpr float kSensorRangeYMin = -8.0f;
constexpr float kSensorRangeYMax = 20.0f;
constexpr float kSensorRangeXMin = -10.0f;
constexpr float kSensorRangeXMax = 10.0f;

void CartesianToIso8855(float x, float y, float* iso_x, float* iso_y) {
  *iso_x = -y;
  *iso_y = x;
}

bool IsInValidRange(float x, float y) {
  return (y > kSensorRangeYMin && y < kSensorRangeYMax &&
          x > kSensorRangeXMin && x < kSensorRangeXMax);
}

}  // namespace

OdFusionComponent::OdFusionComponent(const Config& config)
    : config_(config),
      fusion_counter_(0) {
  tracker_processor_ = std::make_unique<TrackerProcessor>();
}

bool OdFusionComponent::Init() {
  if (tracker_processor_) {
    tracker_processor_->Init();
  }
  fusion_counter_ = 0;
  return true;
}

std::vector<FusedObject> OdFusionComponent::ConvertSvsToObservations(
    const SvsFrame& svs_frame) {
  std::vector<FusedObject> observations;
  for (const auto& svs_obj : svs_frame.svs_object_list) {
    if (svs_obj.object.flag == 1 &&
        IsInValidRange(svs_obj.object.x, svs_obj.object.y)) {
      FusedObject fobj;
      fobj.object.timestamp = svs_obj.timestamp_raw;
      fobj.object.id = svs_obj.object.id + 1;
      fobj.object.x = svs_obj.object.x;
      fobj.object.y = svs_obj.object.y;
      fobj.object.vx = svs_obj.object.vx;
      fobj.object.vy = svs_obj.object.vy;
      fobj.object.yaw = svs_obj.object.yaw;
      fobj.object.length = svs_obj.object.length;
      fobj.object.width = svs_obj.object.width;
      fobj.object.height = svs_obj.object.height;
      fobj.object.type = svs_obj.object.type;
      fobj.object.motion_status = svs_obj.object.motion_status;
      fobj.object.flag = 1;
      fobj.svs_match_id = svs_obj.object.id + 1;
      fobj.obj_det_prop = ObjDetProp::kSoleSvs;
      observations.push_back(fobj);
    }
  }
  return observations;
}

std::vector<FusedObject> OdFusionComponent::ConvertBevToObservations(
    const BevFrame& bev_frame, float veh_head_rear_wheel) {
  std::vector<FusedObject> observations;
  for (const auto& bev_obj : bev_frame.bev_object_list) {
    if (bev_obj.object.flag == 1 &&
        IsInValidRange(bev_obj.object.x, bev_obj.object.y)) {
      FusedObject fobj;
      fobj.object.timestamp = bev_obj.timestamp_raw;
      fobj.object.id = bev_obj.object.id + 1;
      fobj.object.x = bev_obj.object.x + veh_head_rear_wheel;
      fobj.object.y = bev_obj.object.y;
      fobj.object.vx = bev_obj.object.vx;
      fobj.object.vy = bev_obj.object.vy;
      fobj.object.ax = bev_obj.object.ax;
      fobj.object.ay = bev_obj.object.ay;
      fobj.object.length = bev_obj.object.length;
      fobj.object.width = bev_obj.object.width;
      fobj.object.height = bev_obj.object.height;
      fobj.object.type = bev_obj.object.type;
      fobj.object.motion_status = bev_obj.object.motion_status;
      fobj.object.flag = 1;
      fobj.bev_match_id = bev_obj.object.id + 1;
      fobj.obj_det_prop = ObjDetProp::kSoleBev;
      observations.push_back(fobj);
    }
  }
  return observations;
}

std::vector<FusedObject> OdFusionComponent::ConvertRadarToObservations(
    const RadarFrame& radar_frame, float veh_spd) {
  std::vector<FusedObject> observations;
  for (const auto& radar_obj : radar_frame.radar_object_list) {
    if (radar_obj.object.flag == 1) {
      FusedObject fobj;
      fobj.object.timestamp = radar_obj.timestamp_raw;
      fobj.object.id = radar_obj.object.id + 1;

      float iso_x = 0.0f;
      float iso_y = 0.0f;
      CartesianToIso8855(radar_obj.object.x, radar_obj.object.y, &iso_x, &iso_y);
      fobj.object.x = iso_x;
      fobj.object.y = iso_y;

      fobj.object.vx = radar_obj.object.vx - veh_spd;
      fobj.object.vy = radar_obj.object.vy;
      fobj.object.yaw = 0.0f;
      fobj.object.length = 0.2f;
      fobj.object.width = 0.2f;
      fobj.object.height = 0.2f;
      fobj.object.type = ObjectType::kUnknown;
      fobj.object.flag = 1;
      fobj.radar_match_id = radar_obj.object.id + 1;
      fobj.obj_det_prop = ObjDetProp::kSoleRadar;
      observations.push_back(fobj);
    }
  }
  return observations;
}

void OdFusionComponent::ProcessSvs(const SvsFrame& svs_frame,
                                     const GlobalPose& svs_pose) {
  if (!config_.enable_svs || svs_frame.svs_object_list.empty()) {
    return;
  }

  std::vector<FusedObject> observations = ConvertSvsToObservations(svs_frame);
  if (observations.empty()) {
    return;
  }

  uint64_t meas_time = svs_pose.time_stamp;
  std::vector<FusedObject> results;
  tracker_processor_->Process(observations, svs_pose, meas_time,
                               SensorType::kSvs, &results);
}

void OdFusionComponent::ProcessBev(const BevFrame& bev_frame,
                                    const GlobalPose& bev_pose,
                                    float veh_head_rear_wheel) {
  if (!config_.enable_bev || bev_frame.bev_object_list.empty()) {
    return;
  }

  std::vector<FusedObject> observations =
      ConvertBevToObservations(bev_frame, veh_head_rear_wheel);
  if (observations.empty()) {
    return;
  }

  uint64_t meas_time = bev_pose.time_stamp;
  std::vector<FusedObject> results;
  tracker_processor_->Process(observations, bev_pose, meas_time,
                               SensorType::kBev, &results);
}

void OdFusionComponent::ProcessRadar(const RadarFrame& radar_frame,
                                      const GlobalPose& radar_pose,
                                      float veh_spd) {
  if (!config_.enable_radar || radar_frame.radar_object_list.empty()) {
    return;
  }

  std::vector<FusedObject> observations =
      ConvertRadarToObservations(radar_frame, veh_spd);
  if (observations.empty()) {
    return;
  }

  uint64_t meas_time = radar_pose.time_stamp;
  std::vector<FusedObject> results;
  tracker_processor_->Process(observations, radar_pose, meas_time,
                               SensorType::kRadar, &results);
}

void OdFusionComponent::Process(const FrameData& frame_data, uint64_t meas_time) {
  if (!tracker_processor_) {
    return;
  }

  ProcessSvs(frame_data.svs_frame, frame_data.svs_pose);

  ProcessBev(frame_data.bev_frame, frame_data.bev_pose, 0.0f);

  ProcessRadar(frame_data.radar_frame, frame_data.radar_pose, 0.0f);
}

void OdFusionComponent::GetFusionResults(std::vector<FusedObject>* results) {
  if (tracker_processor_) {
    tracker_processor_->GetResults(results);
  }
}

}  // namespace fusion
}  // namespace perception