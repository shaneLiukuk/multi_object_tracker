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

OdFusionComponent::OdFusionComponent()
    : fusion_counter_(0) {
  tracker_processor_ = std::make_unique<TrackerProcessor>();
}

bool OdFusionComponent::Init() {
  if (tracker_processor_) {
    tracker_processor_->Init();
  }
  fusion_counter_ = 0;
  return true;
}

void OdFusionComponent::Process(const FrameData& frame_data, uint64_t meas_time) {
  if (!tracker_processor_) {
    return;
  }

  std::vector<FusedObject> svs_observations;
  for (const auto& svs_obj : frame_data.svs_frame.svs_object_list) {
    if (svs_obj.object.flag == 1 && IsInValidRange(svs_obj.object.x, svs_obj.object.y)) {
      svs_observations.push_back(MapSvsToFusedObject(
          svs_obj.object, svs_obj.timestamp_raw));
    }
  }

  std::vector<FusedObject> results;
  tracker_processor_->Process(svs_observations, frame_data.svs_pose,
                               meas_time, SensorType::kSvs, &results);
}

void OdFusionComponent::GetTrackedObjects(std::vector<FusedObject>* tracked_objects) {
  if (tracker_processor_) {
    tracker_processor_->GetResults(tracked_objects);
  }
}

void OdFusionComponent::GetFusionResults(std::vector<FusedObject>* results) {
  if (tracker_processor_) {
    tracker_processor_->GetResults(results);
  }
}

}  // namespace fusion
}  // namespace perception
