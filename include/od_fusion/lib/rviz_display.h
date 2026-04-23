#pragma once

#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "od_fusion/base/obstacle_constant.h"
#include "od_fusion/lib/coordinate_transform.h"
#include "od_fusion/lib/coordinate_transform.h"

template <typename T>
std::string NumToStr(T num, int precision) {
  std::stringstream ss;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(precision);
  std::string st;
  ss << num;
  ss >> st;

  return st;
}

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

  void PublishAll(const FrameData& frame_data, const std::vector<FusedObject>& results, const GlobalPose& pose);
  void PublishSVS(const std::vector<FusedObject>& svs_fobjects);
 private:
  void PublishMarkers(visualization_msgs::msg::MarkerArray::SharedPtr markers);
  // 核心：直接传入 pub + color + frame_id + 目标列表
  void PublishObjectsToRviz(
      const std::vector<FusedObject>& objects,
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& marker_pub,
      const std_msgs::msg::ColorRGBA& box_color, const std::string& frame_id, const std::string& ns,
      int& last_id);

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

  visualization_msgs::msg::Marker DrawRotatedBoxMarker(int id, const std::string& frame_id,
                                                       double cx, double cy, double yaw,
                                                       double line_width, double length,
                                                       double width, double height,
                                                       const std_msgs::msg::ColorRGBA& color);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr svs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr svs2_pub_;

  std::vector<std_msgs::msg::ColorRGBA> svs_colors_;
  std::vector<std_msgs::msg::ColorRGBA> bev_colors_;
  std::vector<std_msgs::msg::ColorRGBA> radar_colors_;
  std::vector<std_msgs::msg::ColorRGBA> fusion_colors_;

  int32_t marker_id_;
};

}  // namespace fusion
}  // namespace perception
