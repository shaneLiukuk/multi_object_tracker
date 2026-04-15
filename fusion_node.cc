#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "od_fusion/applications/od_fusion_component.h"
#include "od_fusion/lib/rviz_display.h"
#include "calmcar/msg/object_set.hpp"
#include "calmcar/msg/perception_bev_object.hpp"
#include "calmcar/msg/perception_command.hpp"
#include "calmcar/msg/global_pose_estimation.hpp"

namespace perception {
namespace fusion {

// Global pose buffer for callbacks
static GlobalPose g_vehicle_pose;
static bool g_pose_updated = false;
static uint8_t g_enable_svs = 0;
static uint8_t g_enable_bev = 0;
static uint8_t g_enable_radar = 0;

class FusionNode : public rclcpp::Node {
 public:
  explicit FusionNode(const OdFusionComponent::Config& config);

 private:
  void TimerCallback();

  void SvsCallback(const calmcar::msg::ObjectSet::SharedPtr msg);
  void BevCallback(const calmcar::msg::PerceptionBEVObject::SharedPtr msg);
  void PerceptionCommandCallback(const calmcar::msg::PerceptionCommand::SharedPtr msg);
  void GlobalPoseCallback(const calmcar::msg::GlobalPoseEstimation::SharedPtr msg);

  SvsFrame ConvertObjectSetToSvsFrame(const calmcar::msg::ObjectSet& msg);
  BevFrame ConvertPerceptionBEVToBevFrame(const calmcar::msg::PerceptionBEVObject& msg);

  OdFusionComponent fusion_component_;
  std::unique_ptr<RvizDisplay> rviz_display_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<calmcar::msg::ObjectSet>::SharedPtr svs_sub_;
  rclcpp::Subscription<calmcar::msg::PerceptionBEVObject>::SharedPtr bev_sub_;
  rclcpp::Subscription<calmcar::msg::PerceptionCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<calmcar::msg::GlobalPoseEstimation>::SharedPtr pose_sub_;

  int32_t frame_count_;
  SvsFrame last_svs_frame_;
  BevFrame last_bev_frame_;
  bool svs_updated_;
  bool bev_updated_;
};

FusionNode::FusionNode(const OdFusionComponent::Config& config)
    : Node("od_fusion_node"),
      fusion_component_(config),
      frame_count_(0),
      svs_updated_(false),
      bev_updated_(false) {
  if (!fusion_component_.Init()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to init fusion component");
    return;
  }

  rviz_display_ = std::make_unique<RvizDisplay>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}),
      "/od_fusion/visualization");

  // Subscriptions
  svs_sub_ = this->create_subscription<calmcar::msg::ObjectSet>(
      "/perception/svs/object_set",
      10,
      [this](const calmcar::msg::ObjectSet::SharedPtr msg) {
        this->SvsCallback(msg);
      });

  bev_sub_ = this->create_subscription<calmcar::msg::PerceptionBEVObject>(
      "/perception/bev/object",
      10,
      [this](const calmcar::msg::PerceptionBEVObject::SharedPtr msg) {
        this->BevCallback(msg);
      });

  cmd_sub_ = this->create_subscription<calmcar::msg::PerceptionCommand>(
      "/perception/command",
      10,
      [this](const calmcar::msg::PerceptionCommand::SharedPtr msg) {
        this->PerceptionCommandCallback(msg);
      });

  pose_sub_ = this->create_subscription<calmcar::msg::GlobalPoseEstimation>(
      "/localization/global_pose",
      10,
      [this](const calmcar::msg::GlobalPoseEstimation::SharedPtr msg) {
        this->GlobalPoseCallback(msg);
      });

  constexpr int32_t kTimerPeriodMs = 50;
  auto timer_period = std::chrono::milliseconds(kTimerPeriodMs);
  timer_ = this->create_wall_timer(
      timer_period,
      [this]() { this->TimerCallback(); });

  RCLCPP_INFO(this->get_logger(), "OdFusionNode started with %d ms period", kTimerPeriodMs);
}

void FusionNode::SvsCallback(const calmcar::msg::ObjectSet::SharedPtr msg) {
  if (!fusion_component_.GetConfig().enable_svs) {
    return;
  }

  last_svs_frame_ = ConvertObjectSetToSvsFrame(*msg);
  svs_updated_ = true;
}

void FusionNode::BevCallback(const calmcar::msg::PerceptionBEVObject::SharedPtr msg) {
  if (!fusion_component_.GetConfig().enable_bev) {
    return;
  }

  last_bev_frame_ = ConvertPerceptionBEVToBevFrame(*msg);
  bev_updated_ = true;
}

void FusionNode::PerceptionCommandCallback(
    const calmcar::msg::PerceptionCommand::SharedPtr msg) {
  g_enable_svs = msg->enable_object_detection;
  g_enable_bev = msg->enable_radar;  // Note: BEV uses radar flag in this message
  RCLCPP_INFO(this->get_logger(),
              "Perception command: svs=%d, bev=%d",
              g_enable_svs, g_enable_bev);
}

void FusionNode::GlobalPoseCallback(
    const calmcar::msg::GlobalPoseEstimation::SharedPtr msg) {
  g_vehicle_pose.time_stamp = msg->time_stamp;
  g_vehicle_pose.time_stamp_can = msg->time_stamp_can;
  g_vehicle_pose.x = msg->x;
  g_vehicle_pose.y = msg->y;
  g_vehicle_pose.heading = msg->heading;
  g_vehicle_pose.pitch = msg->pitch;
  g_vehicle_pose.roll = msg->roll;
  g_pose_updated = true;
}

SvsFrame FusionNode::ConvertObjectSetToSvsFrame(const calmcar::msg::ObjectSet& msg) {
  SvsFrame svs_frame;
  svs_frame.time_ns = msg.time_stamp;

  for (uint8_t i = 0; i < msg.object_cnt && i < 64; ++i) {
    const auto& obj_info = msg.object_set[i];
    SvsObject svs_obj;
    svs_obj.timestamp_raw = msg.time_stamp_raw;

    svs_obj.object.timestamp = msg.time_stamp;
    svs_obj.object.id = static_cast<uint8_t>(obj_info.tracking_id);
    svs_obj.object.x = obj_info.distance_x;
    svs_obj.object.y = obj_info.distance_y;
    svs_obj.object.vx = obj_info.relative_velocity_x;
    svs_obj.object.vy = obj_info.relative_velocity_y;
    svs_obj.object.ax = obj_info.relative_acceleration_x;
    svs_obj.object.ay = obj_info.relative_acceleration_y;
    svs_obj.object.yaw = obj_info.yaw;
    svs_obj.object.length = obj_info.length;
    svs_obj.object.width = obj_info.width;
    svs_obj.object.height = obj_info.height;
    svs_obj.object.flag = (obj_info.valid_status > 0) ? 1 : 0;

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

    svs_frame.svs_object_list.push_back(svs_obj);
  }

  return svs_frame;
}

BevFrame FusionNode::ConvertPerceptionBEVToBevFrame(
    const calmcar::msg::PerceptionBEVObject& msg) {
  BevFrame bev_frame;
  bev_frame.time_ns = msg.timestamp;

  for (uint8_t i = 0; i < msg.num_of_objects && i < 40; ++i) {
    const auto& obj_info = msg.cam_objects[i];
    BevObject bev_obj;
    bev_obj.timestamp_raw = static_cast<uint64_t>(obj_info.timestamp_raw);

    bev_obj.object.timestamp = msg.timestamp;
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

void FusionNode::TimerCallback() {
  frame_count_++;

  FrameData frame_data;

  if (g_pose_updated) {
    frame_data.svs_pose = g_vehicle_pose;
    frame_data.bev_pose = g_vehicle_pose;
    frame_data.radar_pose = g_vehicle_pose;
  } else {
    // Use default pose if not updated
    frame_data.svs_pose.time_stamp = static_cast<uint64_t>(frame_count_) * 50 * 1000000;
    frame_data.svs_pose.time_stamp_can = frame_data.svs_pose.time_stamp;
    frame_data.bev_pose = frame_data.svs_pose;
    frame_data.radar_pose = frame_data.svs_pose;
  }

  if (svs_updated_ && fusion_component_.GetConfig().enable_svs) {
    frame_data.svs_frame = last_svs_frame_;
    svs_updated_ = false;
  }

  if (bev_updated_ && fusion_component_.GetConfig().enable_bev) {
    frame_data.bev_frame = last_bev_frame_;
    bev_updated_ = false;
  }

  uint64_t meas_time = frame_data.svs_pose.time_stamp;
  fusion_component_.Process(frame_data, meas_time);

  std::vector<FusedObject> results;
  fusion_component_.GetFusionResults(&results);

  // Publish visualization
  if (rviz_display_) {
    rviz_display_->PublishAll(frame_data, results);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Frame %d: SVS %zu, BEV %zu, output %zu tracks",
                       frame_count_,
                       frame_data.svs_frame.svs_object_list.size(),
                       frame_data.bev_frame.bev_object_list.size(),
                       results.size());
}

}  // namespace fusion
}  // namespace perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  perception::fusion::OdFusionComponent::Config config;
  config.enable_svs = true;
  config.enable_bev = true;
  config.enable_radar = false;

  auto node = std::make_shared<perception::fusion::FusionNode>(config);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}