#include "od_fusion/applications/multi_object_tracker_core.h"

#include <cmath>

namespace perception {
namespace fusion {

namespace {

constexpr float kSensorRangeYMin = -8.0f;
constexpr float kSensorRangeYMax = 40.0f;
constexpr float kSensorRangeXMin = -10.0f;
constexpr float kSensorRangeXMax = 10.0f;

bool IsInValidRange(float x, float y) {
  return (y > kSensorRangeYMin && y < kSensorRangeYMax &&
          x > kSensorRangeXMin && x < kSensorRangeXMax);
}

}  // namespace

MultiObjectTracker::MultiObjectTracker(const Config& config)
    : config_(config),
      fusion_counter_(0) {
  tracker_processor_ = std::make_unique<TrackerProcessor>();
}

bool MultiObjectTracker::Init() {
  if (tracker_processor_) {
    tracker_processor_->Init();
  }
  fusion_counter_ = 0;
  return true;
}

std::vector<FusedObject> MultiObjectTracker::ConvertSvsToObservations(
    const SvsFrame& svs_frame) {
  std::vector<FusedObject> observations;
  for (const auto& svs_obj : svs_frame.svs_object_list) {
    if (svs_obj.object.flag == 1) {
      FusedObject fobj;
      fobj.object.timestamp = svs_obj.timestamp_raw;
      fobj.object.id = svs_obj.object.id;
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
      fobj.svs_match_id = svs_obj.object.id;
      fobj.obj_det_prop = ObjDetProp::kSoleSvs;
      observations.push_back(fobj);
    }
  }
  return observations;
}

std::vector<FusedObject> MultiObjectTracker::ConvertBevToObservations(
    const BevFrame& bev_frame, float veh_head_rear_wheel) {
  std::vector<FusedObject> observations;
  for (const auto& bev_obj : bev_frame.bev_object_list) {
    if (bev_obj.object.flag == 1) {
      FusedObject fobj;
      fobj.object.timestamp = bev_obj.timestamp_raw;
      fobj.object.id = bev_obj.object.id;
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
      fobj.bev_match_id = bev_obj.object.id;
      fobj.obj_det_prop = ObjDetProp::kSoleBev;
      observations.push_back(fobj);
    }
  }
  return observations;
}

std::vector<FusedObject> MultiObjectTracker::ConvertRadarToObservations(
    const RadarFrame& radar_frame, float veh_spd) {
  std::vector<FusedObject> observations;
  for (const auto& radar_obj : radar_frame.radar_object_list) {
    if (radar_obj.object.flag == 1) {
      FusedObject fobj;
      fobj.object.timestamp = radar_obj.timestamp_raw;
      fobj.object.id = radar_obj.object.id;

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
      fobj.radar_match_id = radar_obj.object.id;
      fobj.obj_det_prop = ObjDetProp::kSoleRadar;
      observations.push_back(fobj);
    }
  }
  return observations;
}

void MultiObjectTracker::ProcessSvs(const SvsFrame& svs_frame,
                                     const GlobalPose& svs_pose) {
  if (!config_.enable_svs) {
    return;
  }

  std::vector<FusedObject> observations = ConvertSvsToObservations(svs_frame);

  double meas_time = svs_pose.time_stamp;
  std::vector<FusedObject> results;
  tracker_processor_->Process(observations, svs_pose, meas_time,
                               SensorType::kSvs, &results);
}

void MultiObjectTracker::ProcessBev(const BevFrame& bev_frame,
                                    const GlobalPose& bev_pose,
                                    float veh_head_rear_wheel) {
  if (!config_.enable_bev) {
    return;
  }

  std::vector<FusedObject> observations =
      ConvertBevToObservations(bev_frame, veh_head_rear_wheel);
  // if (observations.empty()) {
  //   return;
  // }

  double meas_time = bev_pose.time_stamp;
  std::vector<FusedObject> results;
  tracker_processor_->Process(observations, bev_pose, meas_time,
                               SensorType::kBev, &results);
}

void MultiObjectTracker::ProcessRadar(const RadarFrame& radar_frame,
                                      const GlobalPose& radar_pose,
                                      float veh_spd) {
  if (!config_.enable_radar) {
    return;
  }

  std::vector<FusedObject> observations =
      ConvertRadarToObservations(radar_frame, veh_spd);
  // if (observations.empty()) {
  //   return;
  // }

  double meas_time = radar_pose.time_stamp;
  std::vector<FusedObject> results;
  tracker_processor_->Process(observations, radar_pose, meas_time,
                               SensorType::kRadar, &results);
}

bool MultiObjectTracker::runProcess(const MultiObjectTrackerInput& input,
                                    MultiObjectTrackerOutput* output) {
  if (!tracker_processor_) {
    return false;
  }
  static double lastest_global_pose_time = 0.0;
  static double lastest_svs_time = 0.0;
  // uint64 -> ms
  const uint64_t min_time_interavl = 10;
  if (input.global_pose_in.time_stamp_can - lastest_global_pose_time >
          min_time_interavl &&
      input.svs_object_in.time_stamp_raw > min_time_interavl) {
    // AddGlobalPoseBufferToCache(input.global_pose_buffer_in);
    AddGlobalPoseToCache(input.global_pose_in);

    frame_data_.Reset();
    if (!msg2InterFrame(input, frame_data_)) {
      std::cout << "ERROR:Valid frame is empty." << std::endl;
      return false;
    }
    /*
    不进行航迹处理的约束：
    1.未找到合适的global pose
    2.enable_未开
    3.主传感器一直未收到测量更新
    */
    /* Process Svs Vision */
    ProcessSvs(frame_data_.svs_frame, frame_data_.svs_pose);

    /* Process Front Bev Vision */
    // ProcessBev(frame_data_.bev_frame, frame_data_.bev_pose, 0.0f);

    /* Process Radar */
    // ProcessRadar(frame_data_.radar_frame, frame_data_.radar_pose, 0.0f);
    
    // update lastest timestamp
    lastest_global_pose_time = input.global_pose_in.time_stamp_can;
    lastest_svs_time = input.svs_object_in.time_stamp_raw;
    return true;
  }
  return false;
}

void MultiObjectTracker::GetFusionResults(std::vector<FusedObject>* results) {
  if (tracker_processor_) {
    tracker_processor_->GetResults(results);
  }
}

void MultiObjectTracker::GetFrameData(FrameData* frame_data) {
  if (frame_data == nullptr) {
    return;
  }
  frame_data->Reset();
  frame_data->svs_frame = frame_data_.svs_frame;
  frame_data->svs_pose = frame_data_.svs_pose;
}

bool MultiObjectTracker::msg2InterFrame(const MultiObjectTrackerInput& input, FrameData& frame) {

  // TODO(Shane Liu): object num is 0 ? timestamo is not update?
  SvsFrame svs_f;
  GlobalPose svs_pose;

  if (!QueryNearestLocalization(input.svs_object_in.time_stamp * 1e-3, svs_pose)) {
    std::cout << "WARNNING:svs no sync pose." << std::endl;
    return false;
  }
  svs_f = ConvertObjectSetToSvsFrame(input.svs_object_in, svs_pose);
  frame.svs_frame = svs_f;
  frame.svs_pose = svs_pose;

  return true;

}

void MultiObjectTracker::AddGlobalPoseToCache(const GlobalPoseEstimation& input) {
  // 1. 构造要缓存的 pose
  if (input.time_stamp_can - 0 < kEpsilon) {
    std::cout << "AddGlobalPoseToCache:time " << std::fixed << input.time_stamp_can * 1e-3 << std::endl;
    return;
  }
  perception::GlobalPose pose;
  pose.time_stamp = input.time_stamp * 1e-3;
  pose.time_stamp_can = input.time_stamp_can * 1e-3;
  pose.x = input.x;
  pose.y = input.y;
  pose.heading = input.heading;
  pose.pitch = input.pitch;
  pose.roll = input.roll;

  std::lock_guard<std::mutex> lock(mutex_);

  if (global_pose_cache_.empty()) {
    global_pose_cache_.push_back(pose);
    return;
  }

  // 4. 时间戳重复 → 不添加
  if (lastest_global_pose_.time_stamp == pose.time_stamp) {
    return;
  }

  global_pose_cache_.push_back(pose);

  // 6. 限制缓存大小
  if (global_pose_cache_.size() > 200) {
    global_pose_cache_.pop_front();
  }

  lastest_global_pose_ = pose;
}

void MultiObjectTracker::AddGlobalPoseBufferToCache(const GlobalPoseBuffer& pose_buffer) {
  std::lock_guard<std::mutex> lock(mutex_);

  // 遍历数组里的 10 个 pose
  for (int i = 0; i < 10; ++i) {
    const auto& src_pose = pose_buffer.global_pose[i];

    // 转成你要的 GlobalPose
    perception::GlobalPose pose;
    pose.time_stamp = src_pose.time_stamp * 1e-3 ;
    pose.time_stamp_can = src_pose.time_stamp_can * 1e-3;
    pose.x = src_pose.x;
    pose.y = src_pose.y;
    pose.heading = src_pose.heading;
    pose.pitch = src_pose.pitch;
    pose.roll = src_pose.roll;

    // 去重：和最后一条时间戳相同就跳过
    if (!global_pose_cache_.empty() && global_pose_cache_.back().time_stamp == pose.time_stamp) {
      continue;
    }

    // 加入缓存
    global_pose_cache_.push_back(pose);

    // 限制最大长度 200
    if (global_pose_cache_.size() > 200) {
      global_pose_cache_.pop_front();
    }
  }
}

bool MultiObjectTracker::QueryNearestLocalization(const double& timestamp, GlobalPose& localization) {
  if (global_pose_cache_.empty()) {
    // std::cout << "QueryNearestLocalization: Localization message NOT received." << std::endl;
    return false;
  }
  // reduce timestamp by time delay of sensor data transmission and perception
  // consuming. now 0.0
  double stamp = timestamp - 0.0;
  double loc_ts;
  for (auto it = global_pose_cache_.begin(); it != global_pose_cache_.end(); it++) {
    localization = *it;
    loc_ts = localization.time_stamp_can;
    if (loc_ts < stamp)
      continue;
    else
      break;
  }

  if (abs(stamp - loc_ts) > 0.3) {
    std::cout << "QueryNearestLocalization: Time distance " << std::setprecision(3) << abs(stamp - loc_ts)
           << " between fusion objects: " << std::setprecision(18) << stamp << " and localization: " << std::setprecision(18) << loc_ts
           << " is too long." << std::endl;
    return false;
  }
  return true;
}

SvsFrame MultiObjectTracker::ConvertObjectSetToSvsFrame(const ObjectSet& msg, const GlobalPose& pose) {
  SvsFrame svs_frame;
  svs_frame.time_ns = msg.time_stamp * 1e-3;

  for (uint8_t i = 0; i < msg.object_cnt && i < 64; ++i) {
    const auto& obj_info = msg.object_set[i];
    SvsObject svs_obj;
    svs_obj.timestamp_raw = msg.time_stamp_raw * 1e-3;
    svs_obj.object.timestamp = msg.time_stamp * 1e-3;
    svs_obj.object.id = static_cast<uint8_t>(obj_info.tracking_id);
    if (!(IsInValidRange(obj_info.distance_x, obj_info.distance_y))) {
      // std::cout << "X Range: [ " << kSensorRangeXMin << ", " << kSensorRangeXMax << " ]\n";
      // std::cout << "Y Range: [ " << kSensorRangeYMin << ", " << kSensorRangeYMax << " ]\n";
      continue;
    }
    // if (obj_info.class_id != 1) {
    //   continue;
    // }
    LocalToGlobal(pose, obj_info.distance_x, obj_info.distance_y, &svs_obj.object.x, &svs_obj.object.y);
    // svs_obj.object.x = obj_info.distance_x;
    // svs_obj.object.y = obj_info.distance_y;
    std::cout << "ConvertObjectSetToSvsFrame:" << std::fixed << " abs_vx:" << obj_info.relative_velocity_x << " abs_vy:" << obj_info.relative_velocity_y << std::endl;
    svs_frame.svs_object_list.push_back(svs_obj);
    LocalToGlobalVel(pose, obj_info.relative_velocity_x, obj_info.relative_velocity_y, &svs_obj.object.vx, &svs_obj.object.vy);
    // svs_obj.object.vx = obj_info.relative_velocity_x;
    // svs_obj.object.vy = obj_info.relative_velocity_y;
    svs_obj.object.ax = obj_info.relative_acceleration_x;
    svs_obj.object.ay = obj_info.relative_acceleration_y;
    svs_obj.object.yaw = YawCs1ToIso8855(obj_info.yaw);
    svs_obj.object.length = obj_info.length;
    svs_obj.object.width = obj_info.width;
    svs_obj.object.height = obj_info.height;
    // svs_obj.object.flag = (obj_info.valid_status > 0) ? 1 : 0;
    svs_obj.object.flag = 1;
    // Map class_id to ObjectType
    switch (obj_info.class_id) {
      case 1: svs_obj.object.type = ObjectType::kCar; break;
      case 2: svs_obj.object.type = ObjectType::kBus; break;
      case 3: svs_obj.object.type = ObjectType::kTruck; break;
      case 4: svs_obj.object.type = ObjectType::kPerson; break;
      case 5: svs_obj.object.type = ObjectType::kCyclist; break;
      case 6: svs_obj.object.type = ObjectType::kMotorcyclist; break;
      default: svs_obj.object.type = ObjectType::kUnknown; break;
    }

    // Map motion_status
    switch (obj_info.motion_status) {
      case 1: svs_obj.object.motion_status = MotionStatus::kMoving; break;
      case 2: svs_obj.object.motion_status = MotionStatus::kOncoming; break;
      case 3: svs_obj.object.motion_status = MotionStatus::kStationary; break;
      case 4: svs_obj.object.motion_status = MotionStatus::kStopped; break;
      default: svs_obj.object.motion_status = MotionStatus::kUnknown; break;
    }
    std::cout << "ConvertObjectSetToSvsFrame:" << std::fixed << i << " ID:" << static_cast<int>(svs_obj.object.id) << " YAW:" <<  svs_obj.object.yaw << " abs_vx:" << svs_obj.object.vx << " abs_vy:" << svs_obj.object.vy << std::endl;
    svs_frame.svs_object_list.push_back(svs_obj);
  }

  return svs_frame;
}

BevFrame MultiObjectTracker::ConvertPerceptionBEVToBevFrame(
    const PerceptionBEVObject& msg) {
  BevFrame bev_frame;
  bev_frame.time_ns = msg.timestamp * 1e-3;

  for (uint8_t i = 0; i < msg.num_of_objects && i < 40; ++i) {
    const auto& obj_info = msg.cam_objects[i];
    BevObject bev_obj;
    bev_obj.timestamp_raw = static_cast<uint64_t>(obj_info.timestamp_raw);

    bev_obj.object.timestamp = msg.timestamp * 1e-3;
    bev_obj.object.id = obj_info.id;
    // BEV coordinates: lat_distance is Y (lateral), long_distance is X (longitudinal)
    bev_obj.object.x = obj_info.long_distance;
    bev_obj.object.y = obj_info.lat_distance;
    // Convert absolute velocities
    bev_obj.object.vx = obj_info.abs_long_velocity;
    bev_obj.object.vy = obj_info.abs_lat_velocity;
    // Convert accelerations
    bev_obj.object.ax = obj_info.abs_long_acc;
    bev_obj.object.ay = obj_info.abs_lat_acc;
    bev_obj.object.yaw = obj_info.heading;
    bev_obj.object.length = obj_info.length;
    bev_obj.object.width = obj_info.width;
    bev_obj.object.height = 1.5f;  // Default height
    bev_obj.object.flag = 1;

    // Map class_a to ObjectType
    switch (obj_info.class_a) {
      case 1: bev_obj.object.type = ObjectType::kCar; break;
      case 2: bev_obj.object.type = ObjectType::kBus; break;
      case 3: bev_obj.object.type = ObjectType::kTruck; break;
      case 4: bev_obj.object.type = ObjectType::kPerson; break;
      case 5: bev_obj.object.type = ObjectType::kCyclist; break;
      default: bev_obj.object.type = ObjectType::kUnknown; break;
    }

    // Map motion_status
    switch (obj_info.motion_status) {
      case 1: bev_obj.object.motion_status = MotionStatus::kMoving; break;
      case 2: bev_obj.object.motion_status = MotionStatus::kOncoming; break;
      case 3: bev_obj.object.motion_status = MotionStatus::kStationary; break;
      case 4: bev_obj.object.motion_status = MotionStatus::kStopped; break;
      default: bev_obj.object.motion_status = MotionStatus::kUnknown; break;
    }

    bev_frame.bev_object_list.push_back(bev_obj);
  }

  return bev_frame;
}

}  // namespace fusion
}  // namespace perception