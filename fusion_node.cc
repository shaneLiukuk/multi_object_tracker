#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "od_fusion/applications/multi_object_tracker_core.h"
#include "od_fusion/lib/rviz_display.h"
#include "calmcar/msg/object_set.hpp"
#include "calmcar/msg/perception_bev_object.hpp"
#include "calmcar/msg/perception_command.hpp"
#include "calmcar/msg/global_pose_estimation.hpp"
#include "Rte_Type.h"

namespace perception {
namespace fusion {

// Global pose buffer for callbacks
static GlobalPose g_vehicle_pose;
static bool g_pose_updated = false;
static uint8_t g_enable_svs = 0;
static uint8_t g_enable_bev = 0;
static uint8_t g_enable_radar = 0;
MultiObjectTrackerInput multi_object_fusion_input{};
MultiObjectTrackerOutput multi_object_fusion_output{};
class FusionNode : public rclcpp::Node {
 public:
  explicit FusionNode(const MultiObjectTracker::Config& config);

 private:
  void TimerCallback();

  void SvsCallback(const calmcar::msg::ObjectSet::SharedPtr svs_msg);
  void BevCallback(const calmcar::msg::PerceptionBEVObject::SharedPtr msg);
  void PerceptionCommandCallback(const calmcar::msg::PerceptionCommand::SharedPtr msg);
  void GlobalPoseCallback(const calmcar::msg::GlobalPoseEstimation::SharedPtr msg);

  MultiObjectTracker fusion_component_;
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

FusionNode::FusionNode(const MultiObjectTracker::Config& config)
    : Node("od_fusion_node"),
      fusion_component_(config),
      frame_count_(0),
      svs_updated_(false),
      bev_updated_(false) {
  if (!fusion_component_.Init()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to init fusion component");
    return;
  }
  rclcpp::QoS qos(100);
  qos.reliable();

  rviz_display_ = std::make_unique<RvizDisplay>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}),
      "/od_fusion/visualization");

  // Subscriptions
  svs_sub_ = this->create_subscription<calmcar::msg::ObjectSet>(
      "/svsobjectset", qos, std::bind(&FusionNode::SvsCallback, this, std::placeholders::_1));
  bev_sub_ = this->create_subscription<calmcar::msg::PerceptionBEVObject>(
    "/PerceptionBEVObject", qos, std::bind(&FusionNode::BevCallback, this, std::placeholders::_1));
  cmd_sub_ = this->create_subscription<calmcar::msg::PerceptionCommand>(
    "/perception_command_pub", qos, std::bind(&FusionNode::PerceptionCommandCallback, this, std::placeholders::_1));
  // svs_sub_ = this->create_subscription<calmcar::msg::ObjectSet>(
  //     "/calmcar/msg/ObjectSet",
  //     10,
  //     [this](const calmcar::msg::ObjectSet::SharedPtr msg) {
  //       this->SvsCallback(msg);
  //     });

  pose_sub_ = this->create_subscription<calmcar::msg::GlobalPoseEstimation>(
      "/global_pose_estimation_pub",
      10,
      [this](const calmcar::msg::GlobalPoseEstimation::SharedPtr msg) {
        this->GlobalPoseCallback(msg);
      });

  constexpr int32_t kTimerPeriodMs = 100;
  auto timer_period = std::chrono::milliseconds(kTimerPeriodMs);
  timer_ = this->create_wall_timer(
      timer_period,
      [this]() { this->TimerCallback(); });

  RCLCPP_INFO(this->get_logger(), "OdFusionNode started with %d ms period", kTimerPeriodMs);
}

void FusionNode::SvsCallback(const calmcar::msg::ObjectSet::SharedPtr svs_msg) {
  if (!fusion_component_.GetConfig().enable_svs) {
    return;
  }
  //
  std::vector<FusedObject> svs_fobjects;
  svs_fobjects.clear();
  //
  ObjectSet obj_set{};
  obj_set.time_stamp = svs_msg->time_stamp;
  obj_set.time_stamp_raw = svs_msg->time_stamp_raw;
  obj_set.frame_index = svs_msg->frame_index;
  obj_set.object_cnt = svs_msg->object_cnt;
  if (svs_msg->object_cnt < obj_set.object_cnt) {
    obj_set.object_cnt = svs_msg->object_cnt;
  }
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                      "Svs Cnt %d",obj_set.object_cnt);
  for (uint8_t i = 0; i < obj_set.object_cnt; ++i) {
    // if (static_cast<float>(svs_msg->object_set[i].distance_x) > 2 || static_cast<float>(svs_msg->object_set[i].distance_x) < -2) {
    //   // std::cout << "Svs object " << i << " distance_x out of range: " << svs_msg->object_set[i].distance_x << std::endl;
    //   continue;
    // }
    obj_set.object_set[i].distance_x = svs_msg->object_set[i].distance_x;
    obj_set.object_set[i].distance_y = svs_msg->object_set[i].distance_y;
    obj_set.object_set[i].tracking_id = svs_msg->object_set[i].tracking_id;
    obj_set.object_set[i].class_id = svs_msg->object_set[i].class_id;
    obj_set.object_set[i].angle_view = svs_msg->object_set[i].angle_view;
    obj_set.object_set[i].confidence = svs_msg->object_set[i].confidence;
    obj_set.object_set[i].width = svs_msg->object_set[i].width;
    obj_set.object_set[i].height = svs_msg->object_set[i].height;
    obj_set.object_set[i].length = svs_msg->object_set[i].length;
    obj_set.object_set[i].yaw = svs_msg->object_set[i].yaw;
    obj_set.object_set[i].relative_velocity_x = svs_msg->object_set[i].relative_velocity_x;
    obj_set.object_set[i].relative_velocity_y = svs_msg->object_set[i].relative_velocity_y;
    obj_set.object_set[i].relative_acceleration_x =
        svs_msg->object_set[i].relative_acceleration_x;
    obj_set.object_set[i].motion_status = svs_msg->object_set[i].motion_status;
    obj_set.object_set[i].valid_status = svs_msg->object_set[i].valid_status;
    obj_set.object_set[i].target_source = svs_msg->object_set[i].target_source;

        // RCLCPP_INFO_STREAM(this->get_logger(), "=====================================");
    // RCLCPP_INFO_STREAM(this->get_logger(), "[SVS Object Assign Info]");
    // RCLCPP_INFO_STREAM(this->get_logger(), "timestamp_raw: " << svs_obj.timestamp_raw);
    // RCLCPP_INFO_STREAM(this->get_logger(), "timestamp    : " << svs_obj.object.timestamp);
    // RCLCPP_INFO_STREAM(this->get_logger(), "id           : " << static_cast<int>(svs_obj.object.id));
    // RCLCPP_INFO_STREAM(this->get_logger(), "x            : " << svs_obj.object.x);
    // RCLCPP_INFO_STREAM(this->get_logger(), "y            : " << svs_obj.object.y);
    // RCLCPP_INFO_STREAM(this->get_logger(), "vx           : " << svs_obj.object.vx);
    // RCLCPP_INFO_STREAM(this->get_logger(), "vy           : " << svs_obj.object.vy);
    // RCLCPP_INFO_STREAM(this->get_logger(), "ax           : " << svs_obj.object.ax);
    // RCLCPP_INFO_STREAM(this->get_logger(), "ay           : " << svs_obj.object.ay);
    // RCLCPP_INFO_STREAM(this->get_logger(), "yaw          : " << svs_obj.object.yaw);
    // RCLCPP_INFO_STREAM(this->get_logger(), "length       : " << svs_obj.object.length);
    // RCLCPP_INFO_STREAM(this->get_logger(), "width        : " << svs_obj.object.width);
    // RCLCPP_INFO_STREAM(this->get_logger(), "height       : " << svs_obj.object.height);
    // RCLCPP_INFO_STREAM(this->get_logger(), "flag         : " << static_cast<int>(svs_obj.object.flag));
    // RCLCPP_INFO_STREAM(this->get_logger(), "valid_status : " << static_cast<int>(obj_info.valid_status));
    // RCLCPP_INFO_STREAM(this->get_logger(), "=====================================");
    // RCLCPP_INFO_STREAM_THROTTLE(
    //     this->get_logger(), *this->get_clock(),
    //     1000,  // 👈 改这里：间隔毫秒数
    //     "====================================="
    //         << "\n"
    //         << "[SVS Object Throttled (10Hz)]" << "\n"
    //         << "timestamp_raw: " << svs_obj.timestamp_raw << "\n"
    //         << "timestamp    : " << svs_obj.object.timestamp << "\n"
    //         << "id           : " << static_cast<int>(svs_obj.object.id) << "\n"
    //         << "x            : " << svs_obj.object.x << "\n"
    //         << "y            : " << svs_obj.object.y << "\n"
    //         << "vx           : " << svs_obj.object.vx << "\n"
    //         << "vy           : " << svs_obj.object.vy << "\n"
    //         << "flag         : " << static_cast<int>(svs_obj.object.flag) << "\n"
    //         << "=====================================");

    FusedObject tmp;
    tmp.object.x = obj_set.object_set[i].distance_x;
    tmp.object.y = obj_set.object_set[i].distance_y;
    tmp.object.length = obj_set.object_set[i].length;
    tmp.object.width = obj_set.object_set[i].width;
    tmp.object.height = obj_set.object_set[i].height;
    tmp.object.yaw = obj_set.object_set[i].yaw;
    // std::cout << "x: " << tmp.object.x << ", y: " << tmp.object.y << ", length: " << tmp.object.length
    //           << ", width: " << tmp.object.width << ", height: " << tmp.object.height
    //           << ", yaw: " << tmp.object.yaw << std::endl;

    svs_fobjects.push_back(tmp);
  }
  std::memcpy(&multi_object_fusion_input.svs_object_in, &obj_set, sizeof(obj_set));
  // rviz_display_->PublishSVS(svs_fobjects);
  svs_updated_ = true;
}

void FusionNode::BevCallback(const calmcar::msg::PerceptionBEVObject::SharedPtr msg) {
  if (!fusion_component_.GetConfig().enable_bev) {
    return;
  }

  PerceptionBEVObject bev_obj_set{};

  bev_obj_set.timestamp = msg->timestamp;
  bev_obj_set.vd_count = msg->vd_count;
  bev_obj_set.vru_count = msg->vru_count;
  bev_obj_set.num_of_objects = msg->num_of_objects;
  bev_obj_set.cipv_id = msg->cipv_id;
  bev_obj_set.vd_niv_left = msg->vd_niv_left;
  bev_obj_set.vd_niv_right = msg->vd_niv_right;
  bev_obj_set.cipv_lost = msg->cipv_lost;
  bev_obj_set.allow_acc = msg->allow_acc;

  for (int i = 0; i < 40; i++) {
    bev_obj_set.cam_objects[i].timestamp_raw = msg->cam_objects[i].timestamp_raw;
    bev_obj_set.cam_objects[i].frame_index = msg->cam_objects[i].frame_index;
    bev_obj_set.cam_objects[i].camera_position = msg->cam_objects[i].camera_position;
    bev_obj_set.cam_objects[i].id = msg->cam_objects[i].id;
    bev_obj_set.cam_objects[i].class_a = msg->cam_objects[i].class_a;
    bev_obj_set.cam_objects[i].class_name = msg->cam_objects[i].class_name;
    bev_obj_set.cam_objects[i].subclass = msg->cam_objects[i].subclass;
    bev_obj_set.cam_objects[i].subclass_name = msg->cam_objects[i].subclass_name;
    bev_obj_set.cam_objects[i].confidence = msg->cam_objects[i].confidence;

    bev_obj_set.cam_objects[i].bbox.x = msg->cam_objects[i].bbox.x;
    bev_obj_set.cam_objects[i].bbox.y = msg->cam_objects[i].bbox.y;
    bev_obj_set.cam_objects[i].bbox.width = msg->cam_objects[i].bbox.width;
    bev_obj_set.cam_objects[i].bbox.height = msg->cam_objects[i].bbox.height;

    // bev_obj_set.cam_objects[i].length = msg->cam_objects[i].length;
    // bev_obj_set.cam_objects[i].length_std = msg->cam_objects[i].length_std;
    // bev_obj_set.cam_objects[i].width = msg->cam_objects[i].width;
    // bev_obj_set.cam_objects[i].width_std = msg->cam_objects[i].width_std;
    // bev_obj_set.cam_objects[i].height = msg->cam_objects[i].height;
    // bev_obj_set.cam_objects[i].height_std = msg->cam_objects[i].height_std;
    // bev_obj_set.cam_objects[i].age_count = msg->cam_objects[i].age_count;
    // bev_obj_set.cam_objects[i].age_seconds = msg->cam_objects[i].age_seconds;
    // bev_obj_set.cam_objects[i].visibility_side = msg->cam_objects[i].visibility_side;
    // bev_obj_set.cam_objects[i].heading = msg->cam_objects[i].heading;
    // bev_obj_set.cam_objects[i].heading_std = msg->cam_objects[i].heading_std;
    // bev_obj_set.cam_objects[i].inverse_ttc = msg->cam_objects[i].inverse_ttc;
    // bev_obj_set.cam_objects[i].inverse_ttc_std = msg->cam_objects[i].inverse_ttc_std;
    // bev_obj_set.cam_objects[i].angle_left = msg->cam_objects[i].angle_left;
    // bev_obj_set.cam_objects[i].angle_left_std = msg->cam_objects[i].angle_left_std;
    // bev_obj_set.cam_objects[i].angle_right = msg->cam_objects[i].angle_right;
    // bev_obj_set.cam_objects[i].angle_right_std = msg->cam_objects[i].angle_right_std;
    // bev_obj_set.cam_objects[i].angle_side = msg->cam_objects[i].angle_side;
    // bev_obj_set.cam_objects[i].angle_side_std = msg->cam_objects[i].angle_side_std;
    // bev_obj_set.cam_objects[i].angle_rate = msg->cam_objects[i].angle_rate;
    // bev_obj_set.cam_objects[i].top_out_of_image = msg->cam_objects[i].top_out_of_image;
    // bev_obj_set.cam_objects[i].bottom_out_of_image = msg->cam_objects[i].bottom_out_of_image;
    // bev_obj_set.cam_objects[i].left_out_of_image = msg->cam_objects[i].left_out_of_image;
    // bev_obj_set.cam_objects[i].right_out_of_image = msg->cam_objects[i].right_out_of_image;
    // bev_obj_set.cam_objects[i].brake_light = msg->cam_objects[i].brake_light;
    // bev_obj_set.cam_objects[i].turn_indicator_left = msg->cam_objects[i].turn_indicator_left;
    // bev_obj_set.cam_objects[i].turn_indicator_right = msg->cam_objects[i].turn_indicator_right;
    // bev_obj_set.cam_objects[i].measuring_status = msg->cam_objects[i].measuring_status;
    // bev_obj_set.cam_objects[i].motion_orientation = msg->cam_objects[i].motion_orientation;
    // bev_obj_set.cam_objects[i].motion_category = msg->cam_objects[i].motion_category;
    // bev_obj_set.cam_objects[i].motion_status = msg->cam_objects[i].motion_status;
    // bev_obj_set.cam_objects[i].cutin_cutout = msg->cam_objects[i].cutin_cutout;
    // bev_obj_set.cam_objects[i].lat_distance = msg->cam_objects[i].lat_distance;
    // bev_obj_set.cam_objects[i].lat_distance_std = msg->cam_objects[i].lat_distance_std;
    // bev_obj_set.cam_objects[i].long_distance = msg->cam_objects[i].long_distance;
    // bev_obj_set.cam_objects[i].long_distance_std = msg->cam_objects[i].long_distance_std;
    // bev_obj_set.cam_objects[i].relative_lat_velocity = msg->cam_objects[i].relative_lat_velocity;
    // bev_obj_set.cam_objects[i].relative_lat_velocity_std =
    //     msg->cam_objects[i].relative_lat_velocity_std;
    // bev_obj_set.cam_objects[i].relative_long_velocity = msg->cam_objects[i].relative_long_velocity;
    // bev_obj_set.cam_objects[i].relative_long_velocity_std =
    //     msg->cam_objects[i].relative_long_velocity_std;
    // bev_obj_set.cam_objects[i].abs_lat_velocity = msg->cam_objects[i].abs_lat_velocity;
    // bev_obj_set.cam_objects[i].abs_lat_velocity_std = msg->cam_objects[i].abs_lat_velocity_std;
    // bev_obj_set.cam_objects[i].abs_long_velocity = msg->cam_objects[i].abs_long_velocity;
    // bev_obj_set.cam_objects[i].abs_long_velocity_std = msg->cam_objects[i].abs_long_velocity_std;
    // bev_obj_set.cam_objects[i].relative_lat_acc = msg->cam_objects[i].relative_lat_acc;
    // bev_obj_set.cam_objects[i].relative_lat_acc_std = msg->cam_objects[i].relative_lat_acc_std;
    // bev_obj_set.cam_objects[i].relative_long_acc = msg->cam_objects[i].relative_long_acc;
    // bev_obj_set.cam_objects[i].relative_long_acc_std = msg->cam_objects[i].relative_long_acc_std;
    // bev_obj_set.cam_objects[i].abs_lat_acc = msg->cam_objects[i].abs_lat_acc;
    // bev_obj_set.cam_objects[i].abs_lat_acc_std = msg->cam_objects[i].abs_lat_acc_std;
    // bev_obj_set.cam_objects[i].abs_long_acc = msg->cam_objects[i].abs_long_acc;
    // bev_obj_set.cam_objects[i].abs_long_acc_std = msg->cam_objects[i].abs_long_acc_std;
    // bev_obj_set.cam_objects[i].abs_speed = msg->cam_objects[i].abs_speed;
    // bev_obj_set.cam_objects[i].abs_speed_std = msg->cam_objects[i].abs_speed_std;
    // bev_obj_set.cam_objects[i].abs_acceleration = msg->cam_objects[i].abs_acceleration;
    // bev_obj_set.cam_objects[i].abs_acceleration_std = msg->cam_objects[i].abs_acceleration_std;
    // bev_obj_set.cam_objects[i].obj_lane_assignment = msg->cam_objects[i].obj_lane_assignment;
    // bev_obj_set.cam_objects[i].is_bev_object = msg->cam_objects[i].is_bev_object;
  }
  std::memcpy(&multi_object_fusion_input.front_bev_object_in, &bev_obj_set,
              sizeof(PerceptionBEVObject));

  bev_updated_ = true;
}

void FusionNode::PerceptionCommandCallback(
    const calmcar::msg::PerceptionCommand::SharedPtr msg) {
  g_enable_svs = msg->enable_object_detection;
  g_enable_bev = msg->enable_radar;  // Note: BEV uses radar flag in this message
  // RCLCPP_INFO(this->get_logger(),
  //             "Perception command: svs=%d, bev=%d",
  //             g_enable_svs, g_enable_bev);
}

void FusionNode::GlobalPoseCallback(
    const calmcar::msg::GlobalPoseEstimation::SharedPtr msg) {
  GlobalPoseEstimation pose;
  pose.time_stamp = msg->time_stamp;
  pose.time_stamp_can = msg->time_stamp_can;
  pose.x = msg->x;
  pose.y = msg->y;
  pose.z = msg->z;
  pose.heading = msg->heading;
  pose.pitch = msg->pitch;
  pose.roll = msg->roll;
  std::memcpy(&multi_object_fusion_input.global_pose_in,
              &pose,
              sizeof(GlobalPoseEstimation));
  // RCLCPP_INFO(this->get_logger(),
  //           "Global Pose: x=%f, y=%f",
  //           g_vehicle_pose.x, g_vehicle_pose.y);
  g_pose_updated = true;
}

void FusionNode::TimerCallback() {
  frame_count_++;
  std::vector<FusedObject> results;
  FrameData frame_data;

  // Algo Interface
  if (fusion_component_.runProcess(multi_object_fusion_input, &multi_object_fusion_output)) {
    // Display Data
    fusion_component_.GetFusionResults(&results);
    fusion_component_.GetFrameData(&frame_data);
  } else {
    RCLCPP_WARN(this->get_logger(), "Donot Rcv New Data....");
  }

  // RCLCPP_WARN(this->get_logger(),
  //             "Global pose time updated: %f",
  //             multi_object_fusion_input.global_pose_in.time_stamp_can * 1e-3);

  // Publish visualization
  if (rviz_display_) {
    GlobalPose pose;
    pose.time_stamp = multi_object_fusion_input.global_pose_in.time_stamp * 1e-3;
    pose.time_stamp_can = multi_object_fusion_input.global_pose_in.time_stamp_can * 1e-3;
    pose.x = multi_object_fusion_input.global_pose_in.x;
    pose.y = multi_object_fusion_input.global_pose_in.y;
    pose.heading = multi_object_fusion_input.global_pose_in.heading;
    pose.pitch = multi_object_fusion_input.global_pose_in.pitch;
    pose.roll = multi_object_fusion_input.global_pose_in.roll;
    rviz_display_->PublishAll(frame_data, results, pose);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Frame %d: SVS %zu, BEV %zu, output %zu tracks", frame_count_,
                       frame_data.svs_frame.svs_object_list.size(),
                       frame_data.bev_frame.bev_object_list.size(), results.size());
}

}  // namespace fusion
}  // namespace perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  perception::fusion::MultiObjectTracker::Config config;
  config.enable_svs = true;
  config.enable_bev = true;
  config.enable_radar = false;

  auto node = std::make_shared<perception::fusion::FusionNode>(config);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}