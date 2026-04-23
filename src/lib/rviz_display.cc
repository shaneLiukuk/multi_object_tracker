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
  svs_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/svs_markers", 10);
  svs2_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/svs_markers_2", 10);
  // SVS colors - Blue
  svs_colors_.push_back(MakeColor(1.0f, 0.0f, 0.0f, 0.8f));

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

void RvizDisplay::PublishSVS(const std::vector<FusedObject>& svs_fobjects) {
  visualization_msgs::msg::MarkerArray marker_array;
  const std::string frame_id = "odom";  // 不要带 /

  int id = 0;  // 统一ID
  static int last_id = 0;

  // ==============================
  // 1. 先发布所有新 marker
  // ==============================
  for (size_t i = 0; i < svs_fobjects.size(); ++i) {
    const auto& obj = svs_fobjects[i];
    double vel_x = obj.object.vx;
    double vel_y = obj.object.vy;
    double abs_velocity = std::hypot(vel_x, vel_y);
    float dis_x = obj.object.x;
    float dis_y = obj.object.y;
    CartesianToIso8855(obj.object.x, obj.object.y, &dis_x, &dis_y);

    // --------------------------
    // Text Marker
    // --------------------------
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.ns = "svs_cam_text";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.orientation.w = 1.0;
    text_marker.pose.position.x = dis_x;
    text_marker.pose.position.y = dis_y;
    text_marker.pose.position.z = 0.5;
    text_marker.scale.z = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = "ID: " + std::to_string(obj.object.id) +
                       "\nx: " + NumToStr<float>(dis_x, 2) +
                       "; y: " + NumToStr<float>(dis_y, 2) +
                       "\nv: " + NumToStr<float>(abs_velocity, 2) + " km/h";
    marker_array.markers.push_back(text_marker);

    // --------------------------
    // Point Marker
    // --------------------------
    visualization_msgs::msg::Marker point_marker;
    point_marker.header.frame_id = frame_id;
    point_marker.ns = "svs_cam_pnt";
    point_marker.id = id++;
    point_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    point_marker.action = visualization_msgs::msg::Marker::ADD;
    point_marker.pose.orientation.w = 1.0;
    point_marker.pose.position.x = dis_x;
    point_marker.pose.position.y = dis_y;
    point_marker.pose.position.z = 0.0;
    point_marker.scale.x = 0.8;
    point_marker.scale.y = 0.8;
    point_marker.scale.z = 0.8;
    point_marker.color.r = 0.0;
    point_marker.color.g = 0.0;
    point_marker.color.b = 1.0;
    point_marker.color.a = 1.0;
    marker_array.markers.push_back(point_marker);

    // --------------------------
    // Box Marker
    // --------------------------
    std_msgs::msg::ColorRGBA blue;
    blue.r = 0.0;
    blue.g = 0.5;
    blue.b = 1.0;
    blue.a = 1.0;

    // float yaw = -obj.object.yaw + M_PI / 2;
    float yawIso8855 = YawCs1ToIso8855(obj.object.yaw);

    auto box_marker = DrawRotatedBoxMarker(
        id++, frame_id, dis_x, dis_y,
        yawIso8855, 0.3, 
        obj.object.length, obj.object.width, 0.1,blue);

    marker_array.markers.push_back(box_marker);
  }

  // ==============================
  // 2. 删除所有旧的、多余的 marker（关键修复）
  // ==============================
  for (int del_id = id; del_id < last_id; ++del_id) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.action = visualization_msgs::msg::Marker::DELETE;
    m.id = del_id;

    // 按 id 归属设置 ns（3种都要删！）
    int type = del_id % 3;
    if (type == 0) m.ns = "svs_cam_text";
    else if (type == 1) m.ns = "svs_cam_pnt";
    else m.ns = "svs_cam_box";

    marker_array.markers.push_back(m);
  }

  last_id = id;
  svs_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker RvizDisplay::DrawRotatedBoxMarker(
    int id, const std::string& frame_id, double cx, double cy, double yaw, double line_width, double length, double width, double z = 0.1,
    const std_msgs::msg::ColorRGBA& color = [] {
      std_msgs::msg::ColorRGBA c;
      c.r = 1.0;
      c.g = 1.0;
      c.b = 0.0;
      c.a = 1.0;
      return c;
    }()) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "box";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = line_width;  // 线宽
  marker.color = color;

  double c = std::cos(yaw);
  double s = std::sin(yaw);

  // 以中心为原点，局部四角点
  std::vector<std::pair<double, double>> corners = {
      {length / 2, width / 2}, {length / 2, -width / 2}, {-length / 2, -width / 2}, {-length / 2, width / 2}};

  for (const auto& corner : corners) {
    geometry_msgs::msg::Point p;
    p.x = cx + (corner.first * c - corner.second * s);
    p.y = cy + (corner.first * s + corner.second * c);
    p.z = z;
    marker.points.push_back(p);
  }

  // 闭合矩形（重复第一个点）
  marker.points.push_back(marker.points.front());

  return marker;
}

// 核心：直接传入 pub + color + frame_id + 目标列表
void RvizDisplay::PublishObjectsToRviz(
    const std::vector<FusedObject>& objects,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& marker_pub,
    const std_msgs::msg::ColorRGBA& box_color,
    const std::string& frame_id,
    const std::string& ns,
    int& last_id)  // 必须传引用，用于删除旧marker
{
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;
  auto now = rclcpp::Clock().now();

  // ==================== 1. 绘制当前目标 ====================
  for (const auto& fobj : objects) {
    const auto& obj = fobj.object;
    double vel_x = obj.vx;
    double vel_y = obj.vy;
    double abs_velocity = std::hypot(vel_x, vel_y);

    float dis_x = obj.x;
    float dis_y = obj.y;
    float yawIso8855 = obj.yaw;

    // -------------------- 文字 --------------------
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = now;
    text_marker.ns = ns + "_text";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.orientation.w = 1.0;
    text_marker.pose.position.x = dis_x;
    text_marker.pose.position.y = dis_y;
    text_marker.pose.position.z = 0.5;
    text_marker.scale.z = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = "ID: " + std::to_string(obj.id) +
                       "\nx: " + NumToStr(dis_x, 2) +
                       "\ny: " + NumToStr(dis_y, 2) +
                       "\nv: " + NumToStr(abs_velocity, 2) + " km/h";
    marker_array.markers.push_back(text_marker);

    // -------------------- 圆柱点 --------------------
    visualization_msgs::msg::Marker point_marker;
    point_marker.header.frame_id = frame_id;
    point_marker.header.stamp = now;
    point_marker.ns = ns + "_point";
    point_marker.id = id++;
    point_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    point_marker.action = visualization_msgs::msg::Marker::ADD;
    point_marker.pose.orientation.w = 1.0;
    point_marker.pose.position.x = dis_x;
    point_marker.pose.position.y = dis_y;
    point_marker.pose.position.z = 0.0;
    point_marker.scale.x = 0.8;
    point_marker.scale.y = 0.8;
    point_marker.scale.z = 0.8;
    point_marker.color.r = 0.0;
    point_marker.color.g = 0.0;
    point_marker.color.b = 1.0;
    point_marker.color.a = 1.0;
    marker_array.markers.push_back(point_marker);

    // -------------------- 旋转包围盒（使用传入的颜色） --------------------
    auto box_marker = DrawRotatedBoxMarker(id++, frame_id, dis_x, dis_y, yawIso8855, 0.1,
                                           obj.length, obj.width, 0.1, box_color);
    marker_array.markers.push_back(box_marker);
  }

  // ==================== 2. 删除旧marker ====================
  for (int del_id = id; del_id < last_id; ++del_id) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = now;
    m.action = visualization_msgs::msg::Marker::DELETE;
    m.id = del_id;

    int type = del_id % 3;
    if (type == 0) m.ns = ns + "_text";
    else if (type == 1) m.ns = ns + "_point";
    else m.ns = ns + "_box";  // 这里修复！不再写死"box"

    marker_array.markers.push_back(m);
  }

  last_id = id;
  marker_pub->publish(marker_array);
}

void RvizDisplay::PublishAll(const FrameData& frame_data,
                              const std::vector<FusedObject>& results, const GlobalPose& pose) {
  auto markers = std::make_shared<visualization_msgs::msg::MarkerArray>();

  std::string frame_id = "odom";
  static int last_id = 0;
  // SVS observations - Blue
  std::vector<FusedObject> svs_fobjects;
  for (const auto& obj : frame_data.svs_frame.svs_object_list) {
    FusedObject fobj;
    fobj.object = obj.object;
    GlobalToLocal(pose, obj.object.x, obj.object.y, &fobj.object.x, &fobj.object.y);
    float dis_x = fobj.object.x;
    float dis_y = fobj.object.y;
    CartesianToIso8855(dis_x, dis_y, &fobj.object.x, &fobj.object.y);
    fobj.svs_match_id = obj.object.id + 1;
    fobj.obj_det_prop = ObjDetProp::kSoleSvs;
    svs_fobjects.push_back(fobj);
  }
  const auto& color = svs_colors_.empty() ? fusion_colors_[0] : svs_colors_[0];
  PublishObjectsToRviz(svs_fobjects, svs2_pub_, color, frame_id, "svs", last_id);
  // auto svs_markers = CreateBoxMarkers(svs_fobjects, "svs", svs_colors_);
  // markers->markers.insert(markers->markers.end(), svs_markers.begin(), svs_markers.end());

  // // BEV observations - Green
  // std::vector<FusedObject> bev_fobjects;
  // for (const auto& obj : frame_data.bev_frame.bev_object_list) {
  //   FusedObject fobj;
  //   fobj.object = obj.object;
  //   fobj.bev_match_id = obj.object.id + 1;
  //   fobj.obj_det_prop = ObjDetProp::kSoleBev;
  //   bev_fobjects.push_back(fobj);
  // }
  // auto bev_markers = CreateBoxMarkers(bev_fobjects, "bev", bev_colors_);
  // markers->markers.insert(markers->markers.end(), bev_markers.begin(), bev_markers.end());

  // // Radar observations - Red
  // std::vector<FusedObject> radar_fobjects;
  // for (const auto& obj : frame_data.radar_frame.radar_object_list) {
  //   FusedObject fobj;
  //   fobj.object = obj.object;
  //   fobj.radar_match_id = obj.object.id + 1;
  //   fobj.obj_det_prop = ObjDetProp::kSoleRadar;
  //   radar_fobjects.push_back(fobj);
  // }
  // auto radar_markers = CreateBoxMarkers(radar_fobjects, "radar", radar_colors_);
  // markers->markers.insert(markers->markers.end(), radar_markers.begin(), radar_markers.end());

  // // Fusion results - Yellow
  // std::vector<FusedObject> fused_res;
  // for (const auto& obj : results) {
  //   FusedObject fobj;
  //   fobj.object = obj.object;
  //   GlobalToLocal(pose, obj.object.x, obj.object.y, &fobj.object.x, &fobj.object.y);
  //   fused_res.push_back(fobj);
  // }
  // auto fusion_markers = CreateBoxMarkers(fused_res, "fusion", fusion_colors_);
  // markers->markers.insert(markers->markers.end(), fusion_markers.begin(), fusion_markers.end());

  // // Text labels for all
  // int32_t text_id = 0;

  // for (const auto& obj : svs_fobjects) {
  //   markers->markers.push_back(CreateTextMarker(obj, "svs_text", text_id++));
  // }
  // for (const auto& obj : bev_fobjects) {
  //   markers->markers.push_back(CreateTextMarker(obj, "bev_text", text_id++));
  // }
  // for (const auto& obj : radar_fobjects) {
  //   markers->markers.push_back(CreateTextMarker(obj, "radar_text", text_id++));
  // }
  // for (const auto& obj : results) {
  //   markers->markers.push_back(CreateTextMarker(obj, "fusion_text", text_id++));
  // }

  // PublishMarkers(markers);
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
// yaw = 绕 Z 轴（竖直向上轴）的旋转角（偏航角），单位弧度（rad）
// 旋转顺序：ROS 默认 Z-Y-X（yaw→pitch→roll） 固定轴旋转
// 零点：yaw = 0 时，物体正朝向 X 轴正方向（朝前）
// 正负方向（右手定则，ROS 铁则）
// yaw > 0：逆时针（向左）旋转
// yaw < 0：顺时针（向右）旋转
// 角度范围：
// [−π, π]
// （-180° ~ +180°）
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
