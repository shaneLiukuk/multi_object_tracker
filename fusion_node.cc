#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "od_fusion/applications/od_fusion_component.h"

namespace perception {
namespace fusion {

class FusionNode : public rclcpp::Node {
 public:
  explicit FusionNode(const OdFusionComponent::Config& config);

 private:
  void TimerCallback();

  OdFusionComponent fusion_component_;
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t frame_count_;
};

FusionNode::FusionNode(const OdFusionComponent::Config& config)
    : Node("od_fusion_node"),
      fusion_component_(config),
      frame_count_(0) {
  if (!fusion_component_.Init()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to init fusion component");
    return;
  }

  constexpr int32_t kTimerPeriodMs = 50;
  auto timer_period = std::chrono::milliseconds(kTimerPeriodMs);
  timer_ = this->create_wall_timer(
      timer_period,
      [this]() { this->TimerCallback(); });

  RCLCPP_INFO(this->get_logger(), "OdFusionNode started with %d ms period", kTimerPeriodMs);
}

void FusionNode::TimerCallback() {
  frame_count_++;

  FrameData frame_data;
  GlobalPose svs_pose;

  svs_pose.time_stamp = static_cast<uint64_t>(frame_count_) * 50 * 1000000;
  svs_pose.time_stamp_can = svs_pose.time_stamp;
  svs_pose.x = 0.0f;
  svs_pose.y = 0.0f;
  svs_pose.heading = 0.0f;
  svs_pose.pitch = 0.0f;
  svs_pose.roll = 0.0f;

  SvsFrame svs_frame;
  svs_frame.time_ns = svs_pose.time_stamp;

  const int32_t kObjectsPerFrame = 3;
  for (int32_t i = 0; i < kObjectsPerFrame; ++i) {
    SvsObject svs_obj;
    svs_obj.timestamp_raw = svs_pose.time_stamp;

    float base_angle = static_cast<float>(frame_count_) * 0.1f;
    float obj_angle = base_angle + static_cast<float>(i) * 0.5f;
    float radius = 8.0f + static_cast<float>(i) * 2.0f;

    svs_obj.object.timestamp = svs_pose.time_stamp;
    svs_obj.object.id = static_cast<uint8_t>(i);
    svs_obj.object.x = radius * std::cos(obj_angle);
    svs_obj.object.y = radius * std::sin(obj_angle);
    svs_obj.object.vx = -radius * std::sin(obj_angle) * 0.1f;
    svs_obj.object.vy = radius * std::cos(obj_angle) * 0.1f;
    svs_obj.object.yaw = obj_angle + 3.14159f;
    svs_obj.object.length = 4.5f;
    svs_obj.object.width = 1.8f;
    svs_obj.object.height = 1.5f;
    svs_obj.object.type = ObjectType::kCar;
    svs_obj.object.motion_status = MotionStatus::kMoving;
    svs_obj.object.flag = 1;

    svs_frame.svs_object_list.push_back(svs_obj);
  }

  frame_data.svs_frame = svs_frame;
  frame_data.svs_pose = svs_pose;

  uint64_t meas_time = svs_pose.time_stamp;
  fusion_component_.Process(frame_data, meas_time);

  std::vector<FusedObject> results;
  fusion_component_.GetFusionResults(&results);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Frame %d: processed %zu objects, output %zu tracks",
                       frame_count_, svs_frame.svs_object_list.size(), results.size());
}

}  // namespace fusion
}  // namespace perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  perception::fusion::OdFusionComponent::Config config;
  config.enable_svs = true;
  config.enable_bev = false;
  config.enable_radar = false;

  auto node = std::make_shared<perception::fusion::FusionNode>(config);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}