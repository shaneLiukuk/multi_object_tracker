#include "od_fusion/lib/rviz_display.h"

#include <cmath>
#include <string>

namespace perception {
namespace fusion {

namespace {

constexpr float kDefaultSize = 1.0f;
constexpr float kMarkerLifetimeSec = 0.1f;

std_msgs::msg::ColorRGBA MakeColor(float r, float g, float b, float a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

}  // namespace

RvizDisplay::RvizDisplay(rclcpp::Node::SharedPtr node, const std::string& topic)
    : node_(node), marker_id_(0) {
  publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);

  // SVS colors - Blue
  svs_colors_.push_back(MakeColor(0.0f, 0.0f, 1.0f, 0.8f));

  // BEV colors - Green
  bev_colors_.push_back(MakeColor(0.0f, 1.0f, 0.0f, 0.8f));

  // Radar colors - Red
  radar_colors_.push_back(MakeColor(1.0f, 0.0f, 0.0f, 0.8f));

  // Fusion results - Yellow
  fusion_colors_.push_back(MakeColor(1.0f, 1.0f, 0.0f, 1.0f));
}

void RvizDisplay::PublishSvsObservations(const std::vector<SvsObject>& svs_objects) {
  std::vector<FusedObject> fobjects;
  for (const auto& obj : svs_objects) {
    FusedObject fobj;
    fobj.object = obj.object;
    fobj.svs_match_id = obj.object.id + 1;
    fobj.obj_det_prop = ObjDetProp::kSoleSvs;
    fobjects.push_back(fobj);
  }

  auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();
  auto box_markers = CreateBoxMarkers(fobjects, "svs", svs_colors_);
  markers->markers.insert(markers->markers.end(), box_markers.begin(), box_markers.end());

  int32_t text_id = marker_id_;
  for (const auto& obj : fobjects) {
    markers->markers.push_back(CreateTextMarker(obj, "svs_text", text_id++));
  }

  PublishMarkers(markers);
}

void RvizDisplay::PublishBevObservations(const std::vector<BevObject>& bev_objects) {
  std::vector<FusedObject> fobjects;
  for (const auto& obj : bev_objects) {
    FusedObject fobj;
    fobj.object = obj.object;
    fobj.bev_match_id = obj.object.id + 1;
    fobj.obj_det_prop = ObjDetProp::kSoleBev;
    fobjects.push_back(fobj);
  }

  auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();
  auto box_markers = CreateBoxMarkers(fobjects, "bev", bev_colors_);
  markers->markers.insert(markers->markers.end(), box_markers.begin(), box_markers.end());

  int32_t text_id = marker_id_;
  for (const auto& obj : fobjects) {
    markers->markers.push_back(CreateTextMarker(obj, "bev_text", text_id++));
  }

  PublishMarkers(markers);
}

void RvizDisplay::PublishRadarObservations(const std::vector<RadarObject>& radar_objects) {
  std::vector<FusedObject> fobjects;
  for (const auto& obj : radar_objects) {
    FusedObject fobj;
    fobj.object = obj.object;
    fobj.radar_match_id = obj.object.id + 1;
    fobj.obj_det_prop = ObjDetProp::kSoleRadar;
    fobjects.push_back(fobj);
  }

  auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();
  auto box_markers = CreateBoxMarkers(fobjects, "radar", radar_colors_);
  markers->markers.insert(markers->markers.end(), box_markers.begin(), box_markers.end());

  int32_t text_id = marker_id_;
  for (const auto& obj : fobjects) {
    markers->markers.push_back(CreateTextMarker(obj, "radar_text", text_id++));
  }

  PublishMarkers(markers);
}

void RvizDisplay::PublishFusionResults(const std::vector<FusedObject>& results) {
  auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();
  auto box_markers = CreateBoxMarkers(results, "fusion", fusion_colors_);
  markers->markers.insert(markers->markers.end(), box_markers.begin(), box_markers.end());

  int32_t text_id = marker_id_;
  for (const auto& obj : results) {
    markers->markers.push_back(CreateTextMarker(obj, "fusion_text", text_id++));
  }

  PublishMarkers(markers);
}

void RvizDisplay::PublishAll(const FrameData& frame_data,
                              const std::vector<FusedObject>& results) {
  auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();

  // SVS observations - Blue
  std::vector<FusedObject> svs_fobjects;
  for (const auto& obj : frame_data.svs_frame.svs_object_list) {
    FusedObject fobj;
    fobj.object = obj.object;
    fobj.svs_match_id = obj.object.id + 1;
    fobj.obj_det_prop = ObjDetProp::kSoleSvs;
    svs_fobjects.push_back(fobj);
  }
  auto svs_markers = CreateBoxMarkers(svs_fobjects, "svs", svs_colors_);
  markers->markers.insert(markers->markers.end(), svs_markers.begin(), svs_markers.end());

  // BEV observations - Green
  std::vector<FusedObject> bev_fobjects;
  for (const auto& obj : frame_data.bev_frame.bev_object_list) {
    FusedObject fobj;
    fobj.object = obj.object;
    fobj.bev_match_id = obj.object.id + 1;
    fobj.obj_det_prop = ObjDetProp::kSoleBev;
    bev_fobjects.push_back(fobj);
  }
  auto bev_markers = CreateBoxMarkers(bev_fobjects, "bev", bev_colors_);
  markers->markers.insert(markers->markers.end(), bev_markers.begin(), bev_markers.end());

  // Radar observations - Red
  std::vector<FusedObject> radar_fobjects;
  for (const auto& obj : frame_data.radar_frame.radar_object_list) {
    FusedObject fobj;
    fobj.object = obj.object;
    fobj.radar_match_id = obj.object.id + 1;
    fobj.obj_det_prop = ObjDetProp::kSoleRadar;
    radar_fobjects.push_back(fobj);
  }
  auto radar_markers = CreateBoxMarkers(radar_fobjects, "radar", radar_colors_);
  markers->markers.insert(markers->markers.end(), radar_markers.begin(), radar_markers.end());

  // Fusion results - Yellow
  auto fusion_markers = CreateBoxMarkers(results, "fusion", fusion_colors_);
  markers->markers.insert(markers->markers.end(), fusion_markers.begin(), fusion_markers.end());

  // Text labels for all
  int32_t text_id = 0;

  for (const auto& obj : svs_fobjects) {
    markers->markers.push_back(CreateTextMarker(obj, "svs_text", text_id++));
  }
  for (const auto& obj : bev_fobjects) {
    markers->markers.push_back(CreateTextMarker(obj, "bev_text", text_id++));
  }
  for (const auto& obj : radar_fobjects) {
    markers->markers.push_back(CreateTextMarker(obj, "radar_text", text_id++));
  }
  for (const auto& obj : results) {
    markers->markers.push_back(CreateTextMarker(obj, "fusion_text", text_id++));
  }

  PublishMarkers(markers);
}

void RvizDisplay::PublishMarkers(visualization_msgs::msg::MarkerArray::SharedPtr markers) {
  // Set header timestamps
  for (auto& marker : markers->markers) {
    marker.header.stamp = node_->now();
    marker.header.frame_id = "odom";
  }
  publisher_->publish(*markers);
}

std::vector<visualization_msgs::msg::Marker> RvizDisplay::CreateBoxMarkers(
    const std::vector<FusedObject>& objects,
    const std::string& ns,
    const std::vector<std_msgs::msg::ColorRGBA>& colors) {
  std::vector<visualization_msgs::msg::Marker> result;
  int32_t id = marker_id_;

  for (size_t i = 0; i < objects.size(); ++i) {
    const auto& obj = objects[i];
    const auto& color = colors.empty() ? fusion_colors_[0] : colors[i % colors.size()];
    result.push_back(CreateBoxMarker(obj, ns, id++, color));
  }

  marker_id_ = id;
  return result;
}

visualization_msgs::msg::Marker RvizDisplay::CreateBoxMarker(
    const FusedObject& obj,
    const std::string& ns,
    int32_t id,
    const std_msgs::msg::ColorRGBA& color) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Position
  marker.pose.position.x = obj.object.x;
  marker.pose.position.y = obj.object.y;
  marker.pose.position.z = 0.0;

  // Orientation from yaw
  float yaw = obj.object.yaw;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = std::sin(yaw * 0.5f);
  marker.pose.orientation.w = std::cos(yaw * 0.5f);

  // Size with default fallback
  float length = (obj.object.length > 0.0f) ? obj.object.length : kDefaultSize;
  float width = (obj.object.width > 0.0f) ? obj.object.width : kDefaultSize;
  float height = (obj.object.height > 0.0f) ? obj.object.height : kDefaultSize;

  marker.scale.x = length;
  marker.scale.y = width;
  marker.scale.z = height;

  marker.color = color;

  marker.lifetime = rclcpp::Duration::from_seconds(kMarkerLifetimeSec);

  return marker;
}

visualization_msgs::msg::Marker RvizDisplay::CreateTextMarker(
    const FusedObject& obj,
    const std::string& ns,
    int32_t id) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Position slightly above the box
  float height = (obj.object.height > 0.0f) ? obj.object.height : kDefaultSize;
  marker.pose.position.x = obj.object.x;
  marker.pose.position.y = obj.object.y;
  marker.pose.position.z = height + 0.5f;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Text content
  std::string type_str;
  switch (obj.object.type) {
    case ObjectType::kCar: type_str = "Car"; break;
    case ObjectType::kBus: type_str = "Bus"; break;
    case ObjectType::kTruck: type_str = "Truck"; break;
    case ObjectType::kPerson: type_str = "Person"; break;
    case ObjectType::kCyclist: type_str = "Cyclist"; break;
    case ObjectType::kMotorcyclist: type_str = "Moto"; break;
    case ObjectType::kVehicle: type_str = "Vehicle"; break;
    default: type_str = "Unknown"; break;
  }

  float speed = std::sqrt(obj.object.vx * obj.object.vx + obj.object.vy * obj.object.vy);
  std::string det_prop_str;
  switch (obj.obj_det_prop) {
    case ObjDetProp::kSoleSvs: det_prop_str = "SVS"; break;
    case ObjDetProp::kSoleBev: det_prop_str = "BEV"; break;
    case ObjDetProp::kSoleRadar: det_prop_str = "RADAR"; break;
    case ObjDetProp::kFused: det_prop_str = "FUSED"; break;
    default: det_prop_str = "UNDEF"; break;
  }

  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "ID:%d\nX:%.1f Y:%.1f\nVX:%.1f VY:%.1f\nSpd:%.1f\n%s\n%s",
           obj.object.id,
           obj.object.x, obj.object.y,
           obj.object.vx, obj.object.vy,
           speed,
           type_str.c_str(),
           det_prop_str.c_str());

  marker.text = buffer;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // White text
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.lifetime = rclcpp::Duration::from_seconds(kMarkerLifetimeSec);

  return marker;
}

void RvizDisplay::GetObjectCorners(float x, float y, float length, float width, float yaw,
                                    std::vector<geometry_msgs::msg::Point>* corners) {
  if (corners == nullptr) return;
  corners->clear();

  // Default values
  if (length <= 0.0f) length = kDefaultSize;
  if (width <= 0.0f) width = kDefaultSize;

  float cos_yaw = std::cos(yaw);
  float sin_yaw = std::sin(yaw);

  float half_l = length * 0.5f;
  float half_w = width * 0.5f;

  // 4 corners in object local frame (front, left is positive)
  // Front-Left, Front-Right, Rear-Right, Rear-Left
  std::vector<std::pair<float, float>> local_corners = {
    {half_l, half_w},   // Front-Left
    {half_l, -half_w},  // Front-Right
    {-half_l, -half_w}, // Rear-Right
    {-half_l, half_w}   // Rear-Left
  };

  geometry_msgs::msg::Point p;
  for (const auto& lc : local_corners) {
    // Rotate and translate
    p.x = x + lc.first * cos_yaw - lc.second * sin_yaw;
    p.y = y + lc.first * sin_yaw + lc.second * cos_yaw;
    p.z = 0.0;
    corners->push_back(p);
  }
}

}  // namespace fusion
}  // namespace perception
