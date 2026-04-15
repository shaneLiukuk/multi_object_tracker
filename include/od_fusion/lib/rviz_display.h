#pragma once

#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "od_fusion/base/obstacle_constant.h"

namespace perception {
namespace fusion {

class RvizDisplay {
 public:
  RvizDisplay(rclcpp::Node::SharedPtr node, const std::string& topic);

  ~RvizDisplay() = default;

  void PublishSvsObservations(const std::vector<SvsObject>& svs_objects);

  void PublishBevObservations(const std::vector<BevObject>& bev_objects);

  void PublishRadarObservations(const std::vector<RadarObject>& radar_objects);

  void PublishFusionResults(const std::vector<FusedObject>& results);

  void PublishAll(const FrameData& frame_data, const std::vector<FusedObject>& results);

 private:
  void PublishMarkers(visualization_msgs::msg::MarkerArray::SharedPtr markers);

  std::vector<visualization_msgs::msg::Marker> CreateBoxMarkers(
      const std::vector<FusedObject>& objects,
      const std::string& ns,
      const std::vector<std_msgs::msg::ColorRGBA>& colors);

  visualization_msgs::msg::Marker CreateBoxMarker(
      const FusedObject& obj,
      const std::string& ns,
      int32_t id,
      const std_msgs::msg::ColorRGBA& color);

  visualization_msgs::msg::Marker CreateTextMarker(
      const FusedObject& obj,
      const std::string& ns,
      int32_t id);

  void GetObjectCorners(float x, float y, float length, float width, float yaw,
                        std::vector<geometry_msgs::msg::Point>* corners);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;

  std::vector<std_msgs::msg::ColorRGBA> svs_colors_;
  std::vector<std_msgs::msg::ColorRGBA> bev_colors_;
  std::vector<std_msgs::msg::ColorRGBA> radar_colors_;
  std::vector<std_msgs::msg::ColorRGBA> fusion_colors_;

  int32_t marker_id_;
};

}  // namespace fusion
}  // namespace perception
