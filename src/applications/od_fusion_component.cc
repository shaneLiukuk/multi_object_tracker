#include "od_fusion/applications/od_fusion_component.h"

#include <cmath>

namespace perception {
namespace fusion {

namespace {

constexpr float kSensorRangeYMin = -8.0f;
constexpr float kSensorRangeYMax = 20.0f;
constexpr float kSensorRangeXMin = -10.0f;
constexpr float kSensorRangeXMax = 10.0f;
constexpr float kKphToMs = 1.0f / 3.6f;
constexpr float kTwoPi = 6.283185307179586f;

void LocalToGlobal(const GlobalPose& pose, float local_x, float local_y,
                   float* global_x, float* global_y) {
  float cos_heading = std::cos(pose.heading);
  float sin_heading = std::sin(pose.heading);
  *global_x = pose.x + cos_heading * local_x - sin_heading * local_y;
  *global_y = pose.y + sin_heading * local_x + cos_heading * local_y;
}

void CartesianToIso8855(float x, float y, float* iso_x, float* iso_y) {
  *iso_x = -y;
  *iso_y = x;
}

float YawCs1ToIso8855(float yaw_cs1) {
  return -yaw_cs1;
}

void LocalToGlobalVel(const GlobalPose& pose, float local_vx, float local_vy,
                      float* global_vx, float* global_vy) {
  float cos_heading = std::cos(pose.heading);
  float sin_heading = std::sin(pose.heading);
  *global_vx = cos_heading * local_vx - sin_heading * local_vy;
  *global_vy = sin_heading * local_vx + cos_heading * local_vy;
}

bool IsInValidRange(float x, float y) {
  return (y > kSensorRangeYMin && y < kSensorRangeYMax &&
          x > kSensorRangeXMin && x < kSensorRangeXMax);
}

FusedObject MapSvsToFusedObject(const Object& meas, uint64_t timestamp_raw) {
  FusedObject fobj;
  fobj.object.timestamp = timestamp_raw;
  fobj.object.id = meas.id + 1;
  fobj.object.x = meas.x;
  fobj.object.y = meas.y;
  fobj.object.vx = meas.vx;
  fobj.object.vy = meas.vy;
  fobj.object.yaw = meas.yaw;
  fobj.object.length = meas.length;
  fobj.object.width = meas.width;
  fobj.object.height = meas.height;
  fobj.object.type = meas.type;
  fobj.object.motion_status = meas.motion_status;
  fobj.object.flag = 1;
  fobj.svs_match_id = meas.id + 1;
  fobj.obj_det_prop = ObjDetProp::kSoleSvs;
  return fobj;
}

FusedObject MapBevToFusedObject(const Object& meas, uint64_t timestamp_raw,
                                 float veh_head_rear_wheel) {
  FusedObject fobj;
  fobj.object.timestamp = timestamp_raw;
  fobj.object.id = meas.id + 1;
  fobj.object.x = meas.x + veh_head_rear_wheel;
  fobj.object.y = meas.y;
  fobj.object.vx = meas.vx;
  fobj.object.vy = meas.vy;
  fobj.object.ax = meas.ax;
  fobj.object.ay = meas.ay;
  fobj.object.length = meas.length;
  fobj.object.width = meas.width;
  fobj.object.height = meas.height;
  fobj.object.type = meas.type;
  fobj.object.motion_status = meas.motion_status;
  fobj.object.flag = 1;
  fobj.bev_match_id = meas.id + 1;
  fobj.obj_det_prop = ObjDetProp::kSoleBev;
  return fobj;
}

FusedObject MapRadarToFusedObject(const Object& meas, uint64_t timestamp_raw,
                                   float veh_spd) {
  FusedObject fobj;
  fobj.object.timestamp = timestamp_raw;
  fobj.object.id = meas.id + 1;

  float iso_x = 0.0f;
  float iso_y = 0.0f;
  CartesianToIso8855(meas.x, meas.y, &iso_x, &iso_y);
  fobj.object.x = iso_x;
  fobj.object.y = iso_y;

  fobj.object.vx = meas.vx - veh_spd;
  fobj.object.vy = meas.vy;
  fobj.object.yaw = 0.0f;
  fobj.object.length = 0.2f;
  fobj.object.width = 0.2f;
  fobj.object.height = 0.2f;
  fobj.object.type = ObjectType::kUnknown;
  fobj.object.flag = 1;
  fobj.radar_match_id = meas.id + 1;
  fobj.obj_det_prop = ObjDetProp::kSoleRadar;
  return fobj;
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

FusedObject OdFusionComponent::ConvertSvsObject(const SvsObject& svs_obj) {
  return MapSvsToFusedObject(svs_obj.object, svs_obj.timestamp_raw);
}

FusedObject OdFusionComponent::ConvertBevObject(const BevObject& bev_obj,
                                                  float veh_head_rear_wheel) {
  return MapBevToFusedObject(bev_obj.object, bev_obj.timestamp_raw,
                             veh_head_rear_wheel);
}

FusedObject OdFusionComponent::ConvertRadarObject(const RadarObject& radar_obj,
                                                    float veh_spd) {
  return MapRadarToFusedObject(radar_obj.object, radar_obj.timestamp_raw,
                               veh_spd);
}

void OdFusionComponent::ProcessSvsFrame(const SvsFrame& svs_frame,
                                         const GlobalPose& svs_pose) {
  if (!config_.enable_svs || !tracker_processor_) {
    return;
  }

  std::vector<FusedObject> svs_observations;
  for (const auto& svs_obj : svs_frame.svs_object_list) {
    if (svs_obj.object.flag == 1 &&
        IsInValidRange(svs_obj.object.x, svs_obj.object.y)) {
      svs_observations.push_back(ConvertSvsObject(svs_obj));
    }
  }

  uint64_t meas_time = svs_pose.time_stamp;
  std::vector<FusedObject> results;
  tracker_processor_->Process(svs_observations, svs_pose, meas_time,
                              SensorType::kSvs, &results);
}

void OdFusionComponent::ProcessBevFrame(const BevFrame& bev_frame,
                                         const GlobalPose& bev_pose,
                                         float veh_head_rear_wheel) {
  if (!config_.enable_bev || !tracker_processor_) {
    return;
  }

  std::vector<FusedObject> bev_observations;
  for (const auto& bev_obj : bev_frame.bev_object_list) {
    if (bev_obj.object.flag == 1 &&
        IsInValidRange(bev_obj.object.x, bev_obj.object.y)) {
      bev_observations.push_back(ConvertBevObject(bev_obj, veh_head_rear_wheel));
    }
  }

  uint64_t meas_time = bev_pose.time_stamp;
  std::vector<FusedObject> results;
  tracker_processor_->Process(bev_observations, bev_pose, meas_time,
                              SensorType::kBev, &results);
}

void OdFusionComponent::ProcessRadarFrame(const RadarFrame& radar_frame,
                                          const GlobalPose& radar_pose,
                                          float veh_spd) {
  if (!config_.enable_radar || !tracker_processor_) {
    return;
  }

  std::vector<FusedObject> radar_observations;
  for (const auto& radar_obj : radar_frame.radar_object_list) {
    if (radar_obj.object.flag == 1) {
      radar_observations.push_back(ConvertRadarObject(radar_obj, veh_spd));
    }
  }

  uint64_t meas_time = radar_pose.time_stamp;
  std::vector<FusedObject> results;
  tracker_processor_->Process(radar_observations, radar_pose, meas_time,
                              SensorType::kRadar, &results);
}

void OdFusionComponent::Process(const FrameData& frame_data, uint64_t meas_time) {
  if (!tracker_processor_) {
    return;
  }

  if (config_.enable_svs && !frame_data.svs_frame.svs_object_list.empty()) {
    ProcessSvsFrame(frame_data.svs_frame, frame_data.svs_pose);
  }

  if (config_.enable_bev && !frame_data.bev_frame.bev_object_list.empty()) {
    ProcessBevFrame(frame_data.bev_frame, frame_data.bev_pose, 0.0f);
  }

  if (config_.enable_radar && !frame_data.radar_frame.radar_object_list.empty()) {
    ProcessRadarFrame(frame_data.radar_frame, frame_data.radar_pose, 0.0f);
  }
}

void OdFusionComponent::GetFusionResults(std::vector<FusedObject>* results) {
  if (tracker_processor_) {
    tracker_processor_->GetResults(results);
  }
}

}  // namespace fusion
}  // namespace perception