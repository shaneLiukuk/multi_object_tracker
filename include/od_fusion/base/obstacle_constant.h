#pragma once

#include <cstdint>
#include <vector>
#include <Eigen/Dense>

namespace perception {

constexpr uint8_t kMaxObsSvs = 64;
constexpr uint8_t kMaxObsBev = 20;
constexpr uint8_t kMaxObsRadar = 64;
constexpr uint8_t kMaxObsFuse = 64;
constexpr uint8_t kTrackDepth = 30;
constexpr uint8_t kTrackWidth = 64;
constexpr uint8_t kMaxOutputTracks = 64;
constexpr uint8_t kLen1st = 3;
constexpr int32_t kThresStable = 3;
constexpr int32_t kThresUnstable = 2;
constexpr int32_t kThresLost = 10;
constexpr int32_t kThresOutput = 7;

constexpr float kProcessNoisePos = 0.001f;
constexpr float kProcessNoiseVel = 0.02f;
constexpr float kDistGate = 20.0f;
constexpr float kVelWeight = 0.1f;
constexpr float kPosWeight = 1.0f;

constexpr float kSensorRangeYMin = -8.0f;
constexpr float kSensorRangeYMax = 20.0f;
constexpr float kSensorRangeXMin = -10.0f;
constexpr float kSensorRangeXMax = 10.0f;

constexpr float kStaticCntLow = 20.0f;
constexpr float kStaticDisThres = 15.0f;
constexpr float kVelocityThreshold = 1e-4f;
constexpr float kPi = 3.14159265358979f;

constexpr float kTimeDiffMin = 0.05f;
constexpr float kTimeDiffMax = 0.5f;

enum class ObjectType : uint8_t {
  kUnknown = 0,
  kCar = 1,
  kBus = 2,
  kTruck = 3,
  kPerson = 4,
  kCyclist = 5,
  kMotorcyclist = 6,
  kVehicle = 7
};

enum class MotionStatus : uint8_t {
  kUnknown = 0,
  kMoving = 1,
  kOncoming = 2,
  kStationary = 3,
  kStationaryCandidate = 4,
  kCrossingStationary = 5,
  kCrossingMoving = 6,
  kStopped = 7
};

enum class TrackState : uint8_t {
  kUntracking = 0,
  kInitial = 1,
  kGrowing = 2,
  kStable = 3,
  kUnstable = 4
};

enum class SensorType : uint8_t {
  kRadar = 1,
  kCamera = 2,
  kSvs = 3,
  kBev = 4
};

enum class ObjDetProp : uint8_t {
  kUndefined = 0,
  kSoleRadar = 1,
  kSoleCamera = 2,
  kSoleSvs = 3,
  kSoleBev = 4,
  kFused = 5
};

struct Object {
  uint64_t timestamp = 0;
  uint8_t id = 0;
  float x = 0.0f;
  float y = 0.0f;
  float vx = 0.0f;
  float vy = 0.0f;
  float ax = 0.0f;
  float ay = 0.0f;
  float yaw = 0.0f;
  float length = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
  ObjectType type = ObjectType::kUnknown;
  MotionStatus motion_status = MotionStatus::kUnknown;
  uint8_t flag = 0;
};

struct SvsObject {
  uint64_t timestamp_raw = 0;
  Object object;
};

struct BevObject {
  uint64_t timestamp_raw = 0;
  Object object;
};

struct RadarObject {
  uint64_t timestamp_raw = 0;
  Object object;
};

struct SvsFrame {
  uint64_t time_ns = 0;
  std::vector<SvsObject> svs_object_list;
};

struct BevFrame {
  uint64_t time_ns = 0;
  std::vector<BevObject> bev_object_list;
};

struct RadarFrame {
  uint64_t time_ns = 0;
  std::vector<RadarObject> radar_object_list;
};

struct GlobalPose {
  uint64_t time_stamp = 0;
  uint64_t time_stamp_can = 0;
  float x = 0.0f;
  float y = 0.0f;
  float heading = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
};

struct FrameData {
  SvsFrame svs_frame;
  BevFrame bev_frame;
  RadarFrame radar_frame;
  GlobalPose svs_pose;
  GlobalPose bev_pose;
  GlobalPose radar_pose;
};

struct FusedObject {
  Object object;
  float x_error = 0.0f;
  float y_error = 0.0f;
  ObjDetProp obj_det_prop = ObjDetProp::kUndefined;
  TrackState state = TrackState::kUntracking;
  uint8_t svs_match_id = 0;
  uint8_t bev_match_id = 0;
  uint8_t radar_match_id = 0;
};

struct TrackStatus {
  int32_t cnt = 0;
  int32_t lst = 0;
  uint64_t last_tracking_time = 0;
  int32_t lst_svs = 0;
  int32_t lst_bev = 0;
  int32_t lst_radar = 0;
  int32_t cnt_svs = 0;
  int32_t cnt_bev = 0;
  int32_t cnt_radar = 0;
  int32_t cnt_fused = 0;
  int32_t fused_status_cnt = 0;
  ObjectType fused_type = ObjectType::kUnknown;
  uint8_t used = 0;
  TrackState state = TrackState::kUntracking;
  uint8_t svs_match_id_history[10] = {0};
  uint8_t bev_match_id_history[10] = {0};
  uint8_t radar_match_id_history[10] = {0};
  uint8_t flag = 0;
};

struct Track {
  std::vector<FusedObject> matrix;
  std::vector<uint8_t> lst_flg;
  std::vector<TrackStatus> status;
  Eigen::Matrix4f x_est;
  Eigen::Matrix4f p_est;
  std::vector<FusedObject> estimated;
  std::vector<FusedObject> output;
  int32_t cnt = 0;
  uint8_t cursor = 1;
};

}  // namespace perception
